use bitvec::{order::Lsb0, vec::BitVec};

use crate::{
    AccessMask, Error,
    ErrorKind::WrongPinType,
    PinIdIter,
    jtag::{
        JtagInterface, JtagState,
        Register::{Data, Instruction},
    },
    types::{PinId, PinState, TapCmd, TapInfo, TapState},
};

/// Basic structure for doing boundary scanning. Expects a fixed chain of devices.
pub struct PinScanner {
    sm: Box<dyn JtagInterface>,
    tap_infos: Vec<TapInfo>,
    tap_cmds: Vec<TapCmd>,
    dr_in: BitVec<u8, Lsb0>,
    dr_out: BitVec<u8, Lsb0>,
    extest_keep_present: bool,
}

impl PinScanner {
    pub(crate) fn build(
        mut sm: Box<dyn JtagInterface>,
        tap_infos: Vec<TapInfo>,
        ir_len: usize,
        extest_keep_present: bool,
    ) -> Result<Self, Error> {
        // make sure all TAPs are in bypass mode
        sm.write_repeated(Instruction, 1, ir_len + 1)?;
        sm.switch_mode(JtagState::Idle, true)?;
        let cnt_taps = tap_infos.len();
        Ok(Self {
            sm,
            tap_infos,
            tap_cmds: vec![TapCmd::Bypass; cnt_taps],
            dr_in: BitVec::repeat(false, cnt_taps),
            dr_out: BitVec::repeat(false, cnt_taps),
            extest_keep_present,
        })
    }

    /// Count of TAPs on the chain.
    pub fn tap_count(&self) -> usize {
        self.tap_infos.len()
    }

    /// Information for TAPs on the chain.
    pub fn tap_infos(&self) -> &[TapInfo] {
        &self.tap_infos
    }

    /// [TapInfo] of a TAP; the index is of [Self::tap_infos].
    pub fn get_tap_info(&self, i: usize) -> Result<TapInfo, Error> {
        ChainView::get_tap_info(self, i)
    }

    /// Current state of a TAP; the index is of [Self::tap_infos].
    pub fn get_tap_state(&self, i: usize) -> Result<TapState, Error> {
        ChainView::get_tap_state(self, i)
    }

    /// Switches the state of a TAP with index `i`; the index can be determined by
    /// finding in [Self::tap_infos].
    pub fn set_tap_state(&mut self, i: usize, new_state: TapState) -> Result<(), Error> {
        // NOTE: `TapCmd` is not always `TapState`, because `TapCmd` can be a transition
        // value; the transition should always be done here, not elsewhere.

        let state = self.get_tap_state(i)?;
        if state == new_state {
            return Ok(());
        }
        if state == TapState::Bypass && new_state == TapState::Extest {
            // do it, so the `match` below don't need to handle Bypass -> Extest
            self.set_tap_state(i, TapState::Sample)?;
        }

        match (self.get_tap_state(i)?, new_state) {
            (TapState::Bypass, TapState::Sample) => {
                // check IDCODEs again
                self.command(i, TapCmd::Idcode)?;
                let raw_idcodes = self.split_dr_data(&self.dr_in);
                let raw_idcode = raw_idcodes[i].as_raw_slice();
                let idcode = (raw_idcode[0] as u32)
                    | ((raw_idcode[1] as u32) << 8)
                    | ((raw_idcode[2] as u32) << 16)
                    | ((raw_idcode[3] as u32) << 24);
                if !self.tap_infos[i].matches_idcode(idcode) {
                    self.command(i, TapCmd::Bypass)?;
                    return Err("IDCODE check failed".into());
                }
                if !self.tap_cmds.contains(&TapCmd::Extest) {
                    // check DR len
                    self.command(i, TapCmd::Sample)?;
                    if self.sm.measure_dr_len()? != self.dr_in.len() {
                        self.command(i, TapCmd::Bypass)?;
                        return Err("IDCODE check failed".into());
                    }
                }
            }
            (TapState::Sample, TapState::Extest) => {
                // do PRELOAD first
                self.command(i, TapCmd::Preload)?;
                let mut splited_dr_out = self.split_dr_data(&self.dr_out);
                let preload_values = if self.extest_keep_present {
                    // try to preserve previous output and output enable status
                    &self.split_dr_data(&self.dr_in)[i]
                } else {
                    &self.tap_infos[i].safe_bsr_value()
                };
                splited_dr_out[i].copy_from_bitslice(preload_values);
                self.dr_out = self.merge_dr_data(&mut splited_dr_out);
                self.update()?;
            }
            _ => (),
        }
        self.command(i, new_state.into())?;
        Ok(())
    }

    pub(crate) fn is_extesting(&self) -> bool {
        self.tap_cmds.contains(&TapCmd::Extest)
    }

    /// Iterates through all [PinId]s for a TAP in the chain.
    /// `i_tap` can be determined by finding in [Self::tap_infos].
    pub fn pin_ids_for_tap(&self, i_tap: usize) -> Result<PinIdIter, Error> {
        let TapInfo::WithBsdl(bsdl) = self.get_tap_info(i_tap)? else {
            return Err("not loaded BSDL of this TAP".into());
        };
        Ok(PinIdIter::new(i_tap, bsdl))
    }

    /// Gets a pin identifier by its port name in the BSDL file.
    /// `i_tap` can be determined by finding in [Self::tap_infos].
    ///
    /// For a pin in a bus port, `pin_name` should be "PORT_NAME(i)".
    pub fn get_pin_id(&self, i_tap: usize, pin_name: &str) -> Result<PinId, Error> {
        let TapInfo::WithBsdl(bsdl) = self.get_tap_info(i_tap)? else {
            return Err("not loaded BSDL of this TAP".into());
        };
        PinId::from_name(i_tap, bsdl, pin_name)
    }

    /// Gets the pin state at the timepoint before the last [Self::update] call
    /// applied newly set pin states.
    pub fn get_pin_state(&self, pin_id: &PinId) -> Result<PinState, Error> {
        BsrView::new(self, None).get(pin_id)
    }

    /// Sets the new pin state that will be applyed on the next [Self::update] call.
    /// Only for setting pins with output cell in EXTEST mode.
    ///
    /// If `new_state` is `Some(state)`, `state` will be set in the output cell,
    /// and `true` will be set in the output enable cell if the pin is tristated.
    ///
    /// If `new_state` is `None`, `false` will be set in the output enable cell
    /// if the pin is tristated, otherwise an error is returned.
    pub fn set_pin_state(&mut self, pin_id: &PinId, new_state: Option<bool>) -> Result<(), Error> {
        BsrEdit::new(self, None).set(pin_id, new_state)
    }

    /// Applies [Self::set_pin_state] calls since last [Self::update] and refreshes
    /// the capture buffer for later [Self::get_pin_state] calls with the values
    /// before setting new pin states.
    pub fn update(&mut self) -> Result<(), Error> {
        self.sm.switch_mode(JtagState::CaptureDR, false)?;
        self.sm
            .read_write_bits(Data, &mut self.dr_in, &self.dr_out)?;
        if self.is_extesting() {
            // flush it (which passes through UpdateDR), otherwise it may be
            // executed in the next cycle.
            self.sm.switch_mode(JtagState::Idle, true)?;
        }
        Ok(())
    }

    /// Applies [Self::set_pin_state] calls and then does [Self::update], so that
    /// later [Self::get_pin_state] calls will be influenced by the newly set values.
    /// This makes no sense if no TAP is in EXTEST mode.
    pub fn write_update(&mut self) -> Result<(), Error> {
        if !self.is_extesting() {
            return self.update();
        }
        // apply written bits.
        self.sm.write_bits(Data, &self.dr_out, false)?;
        self.sm.switch_mode(JtagState::Idle, false)?;
        // flush and capture DR again after updating DR.
        self.sm
            .read_write_bits(Data, &mut self.dr_in, &self.dr_out)?;
        Ok(())
    }

    /// Switches all TAPs to BYPASS state.
    pub fn release(&mut self) -> Result<(), Error> {
        let ir_len: usize = self.tap_infos().iter().map(|t| t.ir_len()).sum();
        self.sm.write_repeated(Instruction, 1, ir_len + 1)?;
        self.sm.switch_mode(JtagState::UpdateIR, true)?;
        self.sm.switch_mode(JtagState::Idle, true)?;
        self.tap_cmds = vec![TapCmd::Bypass; self.tap_count()];
        self.dr_in.truncate(self.tap_count());
        self.dr_out.truncate(self.tap_count());
        self.sm.mode_reset()?;
        Ok(())
    }

    /// Switches the IR command for the TAP of index `i`.
    ///
    /// XXX: currently `dr_in` and `dr_out` are simply being resized here
    /// when switching to a DR of different length, extended with 0s if needed.
    fn command(&mut self, i: usize, tap_cmd: TapCmd) -> Result<(), Error> {
        let old_dr_len = self.tap_infos[i].dr_len(self.tap_cmds[i]);
        let new_dr_len = self.tap_infos[i].dr_len(tap_cmd);
        if old_dr_len != new_dr_len {
            let mut splited_dr_in = self.split_dr_data(&self.dr_in);
            splited_dr_in[i].resize(new_dr_len, false);
            self.dr_in = self.merge_dr_data(&mut splited_dr_in);
            let mut splited_dr_out = self.split_dr_data(&self.dr_out);
            splited_dr_out[i].resize(new_dr_len, false);
            self.dr_out = self.merge_dr_data(&mut splited_dr_out);
        }
        self.tap_cmds[i] = tap_cmd;
        let mut ir_data = BitVec::new();
        // `i` is the index starting from TDI, the order of pushing opcodes
        // needs to be reversed so that it starts from TDO.
        for (i, &cmd) in self.tap_cmds.iter().enumerate().rev() {
            self.tap_infos[i].push_opcode(cmd, &mut ir_data);
        }
        self.sm.write_bits(Instruction, &ir_data, true)?;
        self.sm.switch_mode(JtagState::Idle, true)?;
        self.update()?;
        Ok(())
    }

    fn split_dr_data(&self, dr: &BitVec<u8, Lsb0>) -> Vec<BitVec<u8, Lsb0>> {
        let mut result = Vec::new();
        let mut slice_rem = &dr[..];
        // `i` counts from TDI, reverse it to make the loop start from TDO.
        for (i, &cmd) in self.tap_cmds.iter().enumerate().rev() {
            let dr_len = self.tap_infos()[i].dr_len(cmd);
            let splited = &slice_rem[..dr_len];
            slice_rem = &slice_rem[dr_len..];
            let mut splited = BitVec::from_bitslice(splited);
            splited.force_align();
            result.push(splited);
        }
        // after this, the `result` is from TDI to TDO.
        result.reverse();
        result
    }

    fn merge_dr_data(&self, splited_dr: &mut [BitVec<u8, Lsb0>]) -> BitVec<u8, Lsb0> {
        let mut result = BitVec::new();
        // reverse the order in `splited_dr`, so the loop starts from TDO.
        for dr in splited_dr.iter_mut().rev() {
            dr.force_align();
            result.extend_from_bitslice(dr);
        }
        result
    }
}

impl PinScanner {
    /// Executes queued read + write operations for `count` times.
    ///
    /// This can be used to speed up a group of transfers if each item
    /// in the group does not depend on the result of the previous item.
    ///
    /// `writer` and `reader` will both be called for `count` times.
    /// The cycle counter counts towards `count` starting from 0.
    /// On startup, the current BSR output buffer is copied as the
    /// buffer for `writer`.
    ///
    /// At the beginning of each cycle, `writer` is called with the output
    /// buffer and the cycle counter value; any change to the buffer made by
    /// `writer` will be captured and queued for writing to the device, and
    /// preserved for the next `writer` call. When it is actually executed,
    /// the captured states before this write is passed to the corresponding
    /// call to `reader`, along with the same cycle counter value.
    ///
    /// Returns successfully when the last write is finished; before returning,
    /// the output buffer (possibly modified by the last `writer` call) is
    /// applied back to `self`, and [Self::update] is called.
    ///
    /// ```rust,ignore
    /// let pins = [
    ///     "PB0", "PB1", "PB5", "PB6", "PB7", "PB8", "PB9", "PB10", "PB11", "PB12", "PB13", "PB14",
    ///     "PB15", "PC13", "PC14", "PC15",
    /// ];
    /// let pins: Vec<_> = pins
    ///     .iter()
    ///     .map(|n| scanner.get_pin_id(i_c6t6, *n).unwrap())
    ///     .collect();
    ///
    /// scanner.set_tap_state(i_c6t6, TapState::Extest)?;
    /// scanner.batch(
    ///     65536,
    ///     |i, edit| {
    ///         // sets value `i` to the 16-bit "bus".
    ///         for (i_pin, pin) in pins.iter().enumerate() {
    ///             let val = ((i >> i_pin) & 1) == 1;
    ///             edit.set(pin, Some(val))?;
    ///         }
    ///         Ok(())
    ///     },
    ///     |i, view| {
    ///         let i_prev = if i > 0 {
    ///             i - 1
    ///         } else {
    ///             return Ok(());
    ///         };
    ///         // `view` provides pin states before the `i`th edit is applied,
    ///         // which equals the values set in `i - 1`th edit.
    ///         let mut val = 0;
    ///         for (i_pin, pin) in pins.iter().enumerate() {
    ///             if view.get(pin)?.input().unwrap() {
    ///                 val |= 1 << i_pin;
    ///             }
    ///         }
    ///         assert_eq!(val, i_prev);
    ///         Ok(())
    ///     },
    /// )?;
    /// ```
    pub fn batch(
        &mut self,
        count: usize,
        writer: impl FnMut(usize, &mut BsrEdit) -> Result<(), Error>,
        reader: impl FnMut(usize, &BsrView) -> Result<(), Error>,
    ) -> Result<(), Error> {
        self.batch_with_mask(count, writer, reader, None)
    }

    pub(crate) fn batch_with_mask(
        &mut self,
        count: usize,
        mut writer: impl FnMut(usize, &mut BsrEdit) -> Result<(), Error>,
        mut reader: impl FnMut(usize, &BsrView) -> Result<(), Error>,
        access_mask: Option<&AccessMask>,
    ) -> Result<(), Error> {
        self.write_update()?;
        let mut dr_out = self.dr_out.clone();
        let mut bsr_edit = BsrEdit {
            tap_infos: &self.tap_infos,
            tap_cmds: &self.tap_cmds,
            dr_out: &mut dr_out,
            access_mask,
        };
        let res = self.sm.batch(
            count,
            &mut |i, write_buf| {
                writer(i, &mut bsr_edit)?;
                write_buf.resize(bsr_edit.dr_out.len(), false);
                write_buf.copy_from_bitslice(bsr_edit.dr_out);
                Ok(())
            },
            &mut |i, read_buf| {
                let bsr_view = BsrView {
                    tap_infos: &self.tap_infos,
                    tap_cmds: &self.tap_cmds,
                    dr_in: read_buf,
                    access_mask,
                };
                reader(i, &bsr_view)
            },
        );
        self.dr_out.copy_from_bitslice(&dr_out);
        self.update()?;
        res
    }
}

impl Drop for PinScanner {
    fn drop(&mut self) {
        let _ = self
            .release()
            .inspect_err(|e| eprintln!("failed to set taps to bypass mode: {e}"));
    }
}

/// Read-only BSR buffer. Used in the batch operation API.
pub struct BsrView<'a, 'm> {
    tap_infos: &'a [TapInfo],
    tap_cmds: &'a [TapCmd],
    dr_in: &'a BitVec<u8, Lsb0>,
    access_mask: Option<&'m AccessMask>,
}

/// Write-only BSR buffer. Used in the batch operation API.
pub struct BsrEdit<'a, 'm> {
    tap_infos: &'a [TapInfo],
    tap_cmds: &'a [TapCmd],
    dr_out: &'a mut BitVec<u8, Lsb0>,
    access_mask: Option<&'m AccessMask>,
}

impl<'a, 'm> BsrView<'a, 'm> {
    pub(crate) fn new(scanner: &'a PinScanner, access_mask: Option<&'m AccessMask>) -> Self {
        Self {
            tap_infos: scanner.tap_infos(),
            tap_cmds: scanner.tap_cmds(),
            dr_in: &scanner.dr_in,
            access_mask,
        }
    }

    /// Gets the pin state at the timepoint when the BSR is captured.
    pub fn get(&self, pin_id: &PinId) -> Result<PinState, Error> {
        self.verify_pin_id(pin_id, false)?;
        let tap_state = self.get_tap_state(pin_id.tap_index())?;
        if tap_state == TapState::Bypass {
            return Err("the chip that owns the pin is bypassed".into());
        }
        let get_val = |cell: Option<usize>| {
            cell.map(|i| self.index_in_dr(pin_id.tap_index(), i))
                .map(|i| self.dr_in[i])
        };
        Ok(PinState {
            input: get_val(pin_id.cell_i()),
            output: get_val(pin_id.cell_o()),
            output_enable: get_val(pin_id.cell_oe()).map(|v| v != pin_id.cell_oe_disabler()),
        })
    }
}

impl<'a, 'm> BsrEdit<'a, 'm> {
    pub(crate) fn new(scanner: &'a mut PinScanner, access_mask: Option<&'m AccessMask>) -> Self {
        Self {
            tap_infos: &scanner.tap_infos,
            tap_cmds: &scanner.tap_cmds,
            dr_out: &mut scanner.dr_out,
            access_mask,
        }
    }

    /// Sets the new pin state that will be applied when this queued item is finished.
    /// Only for setting pins with output cell in EXTEST mode.
    ///
    /// If `new_state` is `Some(state)`, `state` will be set in the output cell,
    /// and `true` will be set in the output enable cell if the pin is tristated.
    ///
    /// If `new_state` is `None`, `false` will be set in the output enable cell
    /// if the pin is tristated, otherwise an error is returned.
    pub fn set(&mut self, pin_id: &PinId, new_state: Option<bool>) -> Result<(), Error> {
        self.verify_pin_id(pin_id, true)?;
        if self.get_tap_state(pin_id.tap_index())? != TapState::Extest {
            return Err("the chip that owns the pin is not in EXTEST mode".into());
        }
        if let Some(new_state) = new_state {
            let Some(i) = pin_id
                .cell_o()
                .map(|i| self.index_in_dr(pin_id.tap_index(), i))
            else {
                return Error::result(WrongPinType, "not output pin");
            };
            // println!("setting dr[{}] to {:?}", i, new_state);
            *self.dr_out.get_mut(i).unwrap() = new_state;
        }
        if let Some(i) = pin_id
            .cell_oe()
            .map(|i| self.index_in_dr(pin_id.tap_index(), i))
        {
            // disable OE if new_state is None; enable otherwise
            let new_state_oe = if pin_id.cell_oe_disabler() {
                new_state.is_none()
            } else {
                new_state.is_some()
            };
            // println!("setting dr[{}] to {:?}", i, new_state_oe);
            *self.dr_out.get_mut(i).unwrap() = new_state_oe
        } else if new_state.is_none() {
            return Error::result(WrongPinType, "not tristated pin");
        }
        Ok(())
    }
}

pub(crate) trait ChainView {
    /// Information for TAPs on the chain.
    fn tap_infos(&self) -> &[TapInfo];

    /// States of TAPs on the chain.
    fn tap_cmds(&self) -> &[TapCmd];

    /// Checker for pins allowed to use, `None` if all allowed.
    fn access_mask(&self) -> Option<&AccessMask>;

    /// Count of TAPs on the chain.
    fn tap_count(&self) -> usize {
        self.tap_infos().len()
    }

    /// [TapInfo] of a TAP; the index is of [Self::tap_infos].
    fn get_tap_info(&self, i: usize) -> Result<TapInfo, Error> {
        if i >= self.tap_count() {
            return Err("given TAP index exceeds TAP count".into());
        }
        Ok(self.tap_infos()[i].clone())
    }

    /// Current state of a TAP; the index is of [Self::tap_infos].
    fn get_tap_state(&self, i: usize) -> Result<TapState, Error> {
        if i >= self.tap_count() {
            return Err("given TAP index exceeds TAP count".into());
        }
        Ok(match self.tap_cmds()[i] {
            TapCmd::Bypass => TapState::Bypass,
            TapCmd::Sample => TapState::Sample,
            TapCmd::Extest => TapState::Extest,
            _ => unreachable!(),
        })
    }

    /// Checks if a given `PinId` belongs to the current `PinScanner` instance,
    /// and writing on the pin is allowed.
    fn verify_pin_id(&self, pin_id: &PinId, for_write: bool) -> Result<(), Error> {
        if for_write && !pin_id.is_output() {
            return Error::result(WrongPinType, "not output pin");
        }
        let tap_info = self.get_tap_info(pin_id.tap_index())?;
        if tap_info.sessioned_tap_id() != pin_id.sessioned_tap_id() {
            return Err("bad pin ID".into());
        }
        if let Some(mask) = self.access_mask() {
            mask.check_access(pin_id, for_write)?;
        }
        Ok(())
    }

    /// Converts cell index in a DR to the index in the DR chain of TAPs.
    /// NOTE: `i_tap` and `i_cell` is unchecked here.
    fn index_in_dr(&self, i_tap: usize, i_cell: usize) -> usize {
        // reverse it so it starts from TDO.
        self.tap_infos()
            .iter()
            .enumerate()
            .rev()
            .take(self.tap_count() - i_tap - 1)
            .map(|(i, tap)| tap.dr_len(self.tap_cmds()[i]))
            .sum::<usize>()
            + i_cell
    }
}

impl<'a, 'm> ChainView for BsrView<'a, 'm> {
    fn tap_infos(&self) -> &[TapInfo] {
        self.tap_infos
    }
    fn tap_cmds(&self) -> &[TapCmd] {
        self.tap_cmds
    }
    fn access_mask(&self) -> Option<&AccessMask> {
        self.access_mask
    }
}

impl<'a, 'm> ChainView for BsrEdit<'a, 'm> {
    fn tap_infos(&self) -> &[TapInfo] {
        self.tap_infos
    }
    fn tap_cmds(&self) -> &[TapCmd] {
        self.tap_cmds
    }
    fn access_mask(&self) -> Option<&AccessMask> {
        self.access_mask
    }
}

impl ChainView for PinScanner {
    fn tap_infos(&self) -> &[TapInfo] {
        &self.tap_infos
    }
    fn tap_cmds(&self) -> &[TapCmd] {
        &self.tap_cmds
    }
    fn access_mask(&self) -> Option<&AccessMask> {
        None
    }
}
