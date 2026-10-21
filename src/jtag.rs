//! Defines the JTAG state machine interface that the rest of the code operates on.

use std::borrow::Cow;

use bitvec::{order::Lsb0, vec::BitVec};

use jtag_taps::{
    cable::Cable,
    statemachine::{
        JtagSM,
        Register::{Data, Instruction},
    },
};

pub use jtag_taps::statemachine::{JtagState, Register};

/// JTAG state machine interface.
///
/// General notes for using the read/write functions:
/// - The length of any bit vector passed should match the length of
///   the current IR/DR chain of all TAPs.
/// - These functions all make sure the state is in ShiftIR or ShiftDR
///   before actually shifting data.
pub trait JtagInterface {
    /// Changes the mode of the state machine through the shortest path.
    ///
    /// TDI line is held high while clocking out TMS bits.
    fn switch_mode(&mut self, new_state: JtagState) -> Result<(), String>;

    /// Resets the state machine.
    fn mode_reset(&mut self) -> Result<(), String> {
        self.switch_mode(JtagState::Reset)
    }

    /// Writes `bits` to IR or DR (chosen by `reg`), ignoring data on TDO.
    ///
    /// Goes to PauseIR or PauseDR afterwards.
    fn write_bits(&mut self, reg: Register, bits: &BitVec<u8, Lsb0>) -> Result<(), String>;

    /// Reads IR or DR (chosen by `reg`) into `bits`, clocking out ones on TDI.
    ///
    /// Remains in ShiftIR or ShiftDR afterwards.
    fn read_bits(&mut self, reg: Register, bits: &mut BitVec<u8, Lsb0>) -> Result<(), String>;

    /// Writes `write_bits` to IR or DR (chosen by `reg`), reads previous data
    /// into `read_bits`.
    ///
    /// Read length always equals write length; length of `read_bits` may be
    /// adjusted to match `write_bits`.
    ///
    /// Goes to PauseIR or PauseDR afterwards.
    fn read_write_bits(
        &mut self,
        reg: Register,
        read_bits: &mut BitVec<u8, Lsb0>,
        write_bits: &BitVec<u8, Lsb0>,
    ) -> Result<(), String>;

    /// Shifts `bits` of ones or zeros, without automatic UpdateIR or UpdateDR;
    /// however it may goto PauseIR or PauseDR afterwards. `one_or_zero` can be 0 or 1.
    fn write_repeated(
        &mut self,
        reg: Register,
        one_or_zero: u8,
        bits: usize,
    ) -> Result<(), String> {
        let data = BitVec::repeat(one_or_zero > 0, bits);
        self.write_bits(reg, &data)
    }

    /// Measures the total IR length of the scan chain, puts all TAPs
    /// in BYPASS mode afterwards.
    fn measure_ir_len(&mut self) -> Result<usize, String> {
        // fill IR chain with 0s, 1024 bits should be enough for any case
        self.write_repeated(Instruction, 0, 1024)?;

        let buf_one = BitVec::repeat(true, 1); // [1]
        let mut buf_read = BitVec::repeat(false, 1); // [0]

        // enter 1s one by one, stop counting when first 1 reaches the other side
        let mut ir_len = 0;
        loop {
            self.read_write_bits(Instruction, &mut buf_read, &buf_one)?;
            if buf_read[0] {
                break;
            }
            ir_len += 1;
            if ir_len > 1024 {
                return Err("broken chain: TDO holds 0".into());
            }
        }
        if ir_len < 2 {
            return Err("broken chain: TDO holds 1".into());
        }
        Ok(ir_len)
    }

    /// Measures the total DR length of the scan chain, fills them with 0s afterwards.
    ///
    /// NOTE: this can be used only if updating the current DR chain with all 0s
    /// is safe to do, e.g. when measuring the chain of IDCODE registers.
    fn measure_dr_len(&mut self) -> Result<usize, String> {
        // fill DR chain with 1s, 65536 bits should be enough for any case
        self.write_repeated(Data, 1, 65536)?;

        let buf_zero = BitVec::repeat(false, 1); // [0]
        let mut buf_read = BitVec::repeat(true, 1); // [1]

        // enter 0s one by one, stop counting when first 0 reaches the other side
        let mut dr_len = 0;
        loop {
            self.read_write_bits(Data, &mut buf_read, &buf_zero)?;
            if !buf_read[0] {
                break;
            }
            dr_len += 1;
            if dr_len > 1024 {
                return Err("broken chain: TDO holds 1".into());
            }
        }
        if dr_len < 1 {
            return Err("broken chain: TDO holds 0".into());
        }
        Ok(dr_len)
    }
}

// Implementation notes for `jtag_taps::statemachine::JtagSM`:
//
// If some data is written by `write_reg` with `pause_after: false`, the `Cable` implementation
// will shift the last bit of data and stay in ShiftDR; then it will shift another bit if
// `change_mode(UpdateDR)` is called right after that. So this is a problematic usage;
// `pause_after: false` is just for multiple writes, and the last write before changing mode to
// UpdateDR should have `pause_after: true`, to make the last bit of data shift by the Exit1DR
// internally. So, `write_bits` and `read_write_bits` sets `pause_after` to true internally.
//
// The name `tdo` is used throughout `jtag_taps` cable implementations, referring to the host
// output line, which is actually the TDI signal of the device.

impl<T, U> JtagInterface for JtagSM<T>
where
    T: std::ops::DerefMut<Target = U>,
    U: Cable + ?Sized,
{
    fn switch_mode(&mut self, new_state: JtagState) -> Result<(), String> {
        self.change_mode(new_state);
        self.cable.flush();
        Ok(())
    }

    fn write_bits(&mut self, reg: Register, bits: &BitVec<u8, Lsb0>) -> Result<(), String> {
        if bits.is_empty() {
            return Ok(());
        }
        let bits = get_aligned(bits);
        self.write_reg(reg, bits.as_raw_slice(), last_byte_bits(&bits), true);
        Ok(())
    }

    fn read_bits(&mut self, reg: Register, bits: &mut BitVec<u8, Lsb0>) -> Result<(), String> {
        if bits.is_empty() {
            return Ok(());
        }
        let bits_read = self.read_reg(reg, bits.len());
        bits.force_align();
        bits.as_raw_mut_slice().copy_from_slice(&bits_read);
        Ok(())
    }

    fn read_write_bits(
        &mut self,
        reg: Register,
        read_bits: &mut BitVec<u8, Lsb0>,
        write_bits: &BitVec<u8, Lsb0>,
    ) -> Result<(), String> {
        if write_bits.is_empty() {
            return Ok(());
        }
        let write_bits = get_aligned(write_bits);
        let bits_read = self.read_write_reg(
            reg,
            write_bits.as_raw_slice(),
            last_byte_bits(&write_bits),
            true,
        );
        read_bits.resize(write_bits.len(), false);
        read_bits.force_align();
        read_bits.as_raw_mut_slice().copy_from_slice(&bits_read);
        Ok(())
    }
}

fn get_aligned<'a>(bv: &'a BitVec<u8, Lsb0>) -> Cow<'a, BitVec<u8, Lsb0>> {
    use bitvec::domain::Domain::*;
    if matches!(bv.domain(), Enclave(_) | Region { head: Some(_), .. }) {
        let mut bv = bv.to_bitvec();
        bv.force_align();
        Cow::Owned(bv)
    } else {
        Cow::Borrowed(bv)
    }
}

fn last_byte_bits(bv: &BitVec<u8, Lsb0>) -> u8 {
    if bv.is_empty() {
        0
    } else if bv.len().is_multiple_of(8) {
        8
    } else {
        (bv.len() % 8) as u8
    }
}
