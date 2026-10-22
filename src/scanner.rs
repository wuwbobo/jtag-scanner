use bitvec::{order::Lsb0, vec::BitVec};

use crate::{
    PinIdIter,
    jtag::{
        JtagInterface, JtagState,
        Register::{Data, Instruction},
    },
    types::{PinId, PinState, TapCmd, TapInfo, TapState},
};

/// Basic structure for doing boundary scanning. Expects a fixed chain of devices.
pub struct PinScanner<T: JtagInterface> {
    sm: T,
    tap_infos: Vec<TapInfo>,
    tap_cmds: Vec<TapCmd>,
    dr_in: BitVec<u8, Lsb0>,
    dr_out: BitVec<u8, Lsb0>,
    extest_keep_present: bool,
}

impl<T: JtagInterface> PinScanner<T> {
    pub(crate) fn build(
        mut sm: T,
        tap_infos: Vec<TapInfo>,
        ir_len: usize,
        extest_keep_present: bool,
    ) -> Result<Self, String> {
        // make sure all TAPs are in bypass mode
        sm.write_repeated(Instruction, 1, ir_len + 1)?;
        sm.switch_mode(JtagState::Idle)?;
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
    pub fn get_tap_info(&self, i: usize) -> Result<TapInfo, String> {
        if i >= self.tap_count() {
            return Err("given TAP index exceeds TAP count".into());
        }
        Ok(self.tap_infos[i].clone())
    }

    /// Current state of a TAP; the index is of [Self::tap_infos].
    pub fn get_tap_state(&self, i: usize) -> Result<TapState, String> {
        if i >= self.tap_count() {
            return Err("given TAP index exceeds TAP count".into());
        }
        Ok(match self.tap_cmds[i] {
            TapCmd::Bypass => TapState::Bypass,
            TapCmd::Sample => TapState::Sample,
            TapCmd::Extest => TapState::Extest,
            _ => unreachable!(),
        })
    }

    /// Switches the state of a TAP with index `i`; the index can be determined by
    /// finding in [Self::tap_infos].
    pub fn set_tap_state(&mut self, i: usize, new_state: TapState) -> Result<(), String> {
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

    /// Iterates through all [PinId]s for a TAP in the chain.
    /// `i_tap` can be determined by finding in [Self::tap_infos].
    pub fn pin_ids_for_tap(&self, i_tap: usize) -> Result<PinIdIter, String> {
        let TapInfo::WithBsdl(bsdl) = self.get_tap_info(i_tap)? else {
            return Err("not loaded BSDL of this TAP".into());
        };
        Ok(PinIdIter::new(i_tap, bsdl))
    }

    /// Gets a pin identifier by its port name in the BSDL file.
    /// `i_tap` can be determined by finding in [Self::tap_infos].
    ///
    /// For a pin in a bus port, `pin_name` should be "PORT_NAME(i)".
    pub fn get_pin_id(&self, i_tap: usize, pin_name: &str) -> Result<PinId, String> {
        let TapInfo::WithBsdl(bsdl) = self.get_tap_info(i_tap)? else {
            return Err("not loaded BSDL of this TAP".into());
        };
        PinId::from_name(i_tap, bsdl, pin_name)
    }

    /// Checks if a given `PinId` belongs to the current `PinScanner` instance.
    pub(crate) fn verify_pin_id(&self, pin_id: &PinId) -> Result<(), String> {
        let tap_info = self.get_tap_info(pin_id.tap_index())?;
        if tap_info.sessioned_tap_id() != pin_id.sessioned_tap_id() {
            return Err("bad pin ID".into());
        }
        Ok(())
    }

    /// Gets the pin state at the timepoint before the last [Self::update] call
    /// applied newly set pin states.
    pub fn get_pin_state(&self, pin_id: &PinId) -> Result<PinState, String> {
        self.verify_pin_id(pin_id)?;
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

    /// Sets the new pin state that will be applyed on the next [Self::update] call.
    /// Only for setting pins with output cell in EXTEST mode.
    ///
    /// If `new_state` is `Some(state)`, `state` will be set in the output cell,
    /// and `true` will be set in the output enable cell if the pin is tristated.
    ///
    /// If `new_state` is `None`, `false` will be set in the output enable cell
    /// if the pin is tristated, otherwise an error is returned.
    pub fn set_pin_state(&mut self, pin_id: &PinId, new_state: Option<bool>) -> Result<(), String> {
        self.verify_pin_id(pin_id)?;
        if self.get_tap_state(pin_id.tap_index())? != TapState::Extest {
            return Err("the chip that owns the pin is not in EXTEST mode".into());
        }
        if let Some(new_state) = new_state {
            let Some(i) = pin_id
                .cell_o()
                .map(|i| self.index_in_dr(pin_id.tap_index(), i))
            else {
                return Err("the pin does not support output".into());
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
            return Err("the pin does not support output enable".into());
        }
        Ok(())
    }

    /// Applies [Self::set_pin_state] calls since last [Self::update] and refreshes
    /// the capture buffer for later [Self::get_pin_state] calls with the values
    /// before setting new pin states.
    pub fn update(&mut self) -> Result<(), String> {
        self.sm.switch_mode(JtagState::CaptureDR)?;
        self.sm
            .read_write_bits(Data, &mut self.dr_in, &self.dr_out)?;
        self.sm.switch_mode(JtagState::Idle)?;
        Ok(())
    }

    /// Applies [Self::set_pin_state] calls and then does [Self::update], so that
    /// later [Self::get_pin_state] calls will be influenced by the newly set values.
    /// This makes no sense if no TAP is in EXTEST mode.
    pub fn write_update(&mut self) -> Result<(), String> {
        // apply written bits.
        self.sm.write_bits(Data, &self.dr_out)?;
        self.sm.switch_mode(JtagState::Idle)?;
        // this is just for reading.
        self.sm
            .read_write_bits(Data, &mut self.dr_in, &self.dr_out)?;
        Ok(())
    }

    /// Switches all TAPs to BYPASS state.
    pub fn release(&mut self) -> Result<(), String> {
        let ir_len: usize = self.tap_infos().iter().map(|t| t.ir_len()).sum();
        self.sm.write_repeated(Instruction, 1, ir_len + 1)?;
        self.sm.switch_mode(JtagState::UpdateIR)?;
        self.sm.switch_mode(JtagState::Idle)?;
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
    fn command(&mut self, i: usize, tap_cmd: TapCmd) -> Result<(), String> {
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
        self.sm.write_bits(Instruction, &ir_data)?;
        self.sm.switch_mode(JtagState::Idle)?;
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

    /// Converts cell index in a DR to the index in the DR chain of TAPs.
    /// NOTE: `i_tap` and `i_cell` is unchecked here.
    fn index_in_dr(&self, i_tap: usize, i_cell: usize) -> usize {
        // reverse it so it starts from TDO.
        self.tap_infos()
            .iter()
            .enumerate()
            .rev()
            .take(self.tap_count() - i_tap - 1)
            .map(|(i, tap)| tap.dr_len(self.tap_cmds[i]))
            .sum::<usize>()
            + i_cell
    }
}

impl<T: JtagInterface> Drop for PinScanner<T> {
    fn drop(&mut self) {
        let _ = self
            .release()
            .inspect_err(|e| eprintln!("failed to set taps to bypass mode: {e}"));
    }
}
