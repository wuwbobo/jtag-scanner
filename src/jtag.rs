//! Defines the JTAG state machine interface that the rest of the code operates on.

use bitvec::{order::Lsb0, vec::BitVec};

use crate::{
    Error,
    ErrorKind::BrokenChain,
    jtag::Register::{Data, Instruction},
};

#[cfg(feature = "backend-jtag-taps")]
mod jtag_taps;
#[cfg(feature = "backend-jtag-taps")]
pub use ::jtag_taps::statemachine::{JtagState, Register};

#[cfg(not(feature = "backend-jtag-taps"))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Register {
    Data,
    Instruction,
}

#[cfg(not(feature = "backend-jtag-taps"))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum JtagState {
    Reset = 0,
    Idle = 1,
    SelectDR = 2,
    CaptureDR = 3,
    ShiftDR = 4,
    Exit1DR = 5,
    PauseDR = 6,
    Exit2DR = 7,
    UpdateDR = 8,
    SelectIR = 9,
    CaptureIR = 10,
    ShiftIR = 11,
    Exit1IR = 12,
    PauseIR = 13,
    Exit2IR = 14,
    UpdateIR = 15,
}

/// Trait for supported JTAG adapter drivers (currently all of them are from `jtag-taps`).
pub trait Backend {
    #[doc(hidden)]
    fn into_interface(self) -> Result<Box<dyn JtagInterface>, Error>;
}

/// JTAG state machine interface.
///
/// General notes for using `read_bits`, `write_bits`, `read_write_bits`:
/// - The length of any bit vector passed to them should match the length
///   of the current IR/DR chain of all TAPs.
/// - These functions all make sure the state is in ShiftIR or ShiftDR
///   before actually shifting data.
pub trait JtagInterface {
    /// Changes the mode of the state machine through the shortest path.
    /// Does flushing if `flush` is true.
    ///
    /// This does not need to be used in closures for [Self::batch].
    ///
    /// TDI line is held high while clocking out TMS bits.
    fn switch_mode(&mut self, new_state: JtagState, flush: bool) -> Result<(), Error>;

    /// (Flushing) Resets the state machine to Test-Logic-Reset.
    fn mode_reset(&mut self) -> Result<(), Error> {
        self.switch_mode(JtagState::Reset, true)
    }

    /// Writes `bits` to IR or DR (chosen by `reg`), ignoring data on TDO.
    /// Does flushing if `flush` is true.
    ///
    /// Goes to PauseIR or PauseDR afterwards.
    fn write_bits(
        &mut self,
        reg: Register,
        bits: &BitVec<u8, Lsb0>,
        flush: bool,
    ) -> Result<(), Error>;

    /// (Flushing) Reads IR or DR (chosen by `reg`) into `bits`, clocking out ones on TDI.
    ///
    /// Remains in ShiftIR or ShiftDR afterwards.
    fn read_bits(&mut self, reg: Register, bits: &mut BitVec<u8, Lsb0>) -> Result<(), Error>;

    /// (Flushing) Writes and `write_bits` to IR or DR (chosen by `reg`),
    /// reads previous data into `read_bits`.
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
    ) -> Result<(), Error>;

    /// Does queued read + write operations for `count` times. The counter passed
    /// to `writer` and `reader` counts in range `0..count`.
    ///
    /// Handling of each `writer` call requires knowing the current DR chain length,
    /// and resizing the data buffer (through `&mut BitVec`) to that length, filling
    /// it with new data.
    ///
    /// When the counter value is `i` in `reader`, it provides DR data captured
    /// *before* updating DR data with the data produced in `i`th `writer` call.
    #[allow(clippy::type_complexity)]
    fn batch(
        &mut self,
        count: usize,
        writer: &mut dyn FnMut(usize, &mut BitVec<u8, Lsb0>) -> Result<(), Error>,
        reader: &mut dyn FnMut(usize, &BitVec<u8, Lsb0>) -> Result<(), Error>,
    ) -> Result<(), Error>;

    /// Shifts `bits` of ones or zeros, without automatic UpdateIR or UpdateDR;
    /// however it may goto PauseIR or PauseDR afterwards. `one_or_zero` can be 0 or 1.
    fn write_repeated(&mut self, reg: Register, one_or_zero: u8, bits: usize) -> Result<(), Error> {
        // limits the length of each transfer to 512 bits to avoid the limits
        // in some `jtag_taps` cable implementations.
        let (cnt_blocks, cnt_rem) = (bits / 512, bits % 512);
        for _ in 0..cnt_blocks {
            let data = BitVec::repeat(one_or_zero > 0, 512);
            self.write_bits(reg, &data, true)?;
        }
        let data = BitVec::repeat(one_or_zero > 0, cnt_rem);
        self.write_bits(reg, &data, true)
    }

    /// Measures the total IR length of the scan chain, puts all TAPs
    /// in BYPASS mode afterwards.
    fn measure_ir_len(&mut self) -> Result<usize, Error> {
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
                return Error::result(BrokenChain, "TDO holds 0");
            }
        }
        if ir_len < 2 {
            return Error::result(BrokenChain, "TDO holds 1");
        }
        Ok(ir_len)
    }

    /// Measures the total DR length of the scan chain, fills them with 0s afterwards.
    ///
    /// NOTE: this can be used only if updating the current DR chain with all 0s
    /// is safe to do, e.g. when measuring the chain of IDCODE registers.
    fn measure_dr_len(&mut self) -> Result<usize, Error> {
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
                return Error::result(BrokenChain, "TDO holds 1");
            }
        }
        if dr_len < 1 {
            return Error::result(BrokenChain, "TDO holds 0");
        }
        Ok(dr_len)
    }
}
