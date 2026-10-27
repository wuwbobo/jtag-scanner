//! Defines the JTAG state machine interface that the rest of the code operates on.

use std::{
    borrow::Cow,
    collections::VecDeque,
    ops::{Deref, DerefMut},
};

use bitvec::{order::Lsb0, vec::BitVec};

use jtag_taps::{
    cable::Cable,
    statemachine::{
        JtagSM,
        Register::{Data, Instruction},
    },
};

pub use jtag_taps::statemachine::{JtagState, Register};

use crate::{Error, ErrorKind::BrokenChain};

/// JTAG state machine interface.
///
/// General notes for using `read_bits`, `write_bits`, `read_write_bits`:
/// - The length of any bit vector passed to them should match the length
///   of the current IR/DR chain of all TAPs.
/// - These functions all make sure the state is in ShiftIR or ShiftDR
///   before actually shifting data.
/// - These functions all make sure to finish the cable transfer instead
///   of just queueing the operations.
pub trait JtagInterface {
    /// Changes the mode of the state machine through the shortest path.
    /// Always flushes the command; not for queued operations.
    ///
    /// TDI line is held high while clocking out TMS bits.
    fn switch_mode(&mut self, new_state: JtagState) -> Result<(), Error>;

    /// Resets the state machine.
    fn mode_reset(&mut self) -> Result<(), Error> {
        self.switch_mode(JtagState::Reset)
    }

    /// Writes `bits` to IR or DR (chosen by `reg`), ignoring data on TDO.
    ///
    /// Goes to PauseIR or PauseDR afterwards.
    fn write_bits(&mut self, reg: Register, bits: &BitVec<u8, Lsb0>) -> Result<(), Error>;

    /// Reads IR or DR (chosen by `reg`) into `bits`, clocking out ones on TDI.
    ///
    /// Remains in ShiftIR or ShiftDR afterwards.
    fn read_bits(&mut self, reg: Register, bits: &mut BitVec<u8, Lsb0>) -> Result<(), Error>;

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
    ) -> Result<(), Error>;

    /// Does queued read + write operations for `count` times.
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
            self.write_bits(reg, &data)?;
        }
        let data = BitVec::repeat(one_or_zero > 0, cnt_rem);
        self.write_bits(reg, &data)
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

/// Trait for supported cable drivers (currently all of them are from `jtag-taps`).
pub trait Backend {
    #[doc(hidden)]
    fn into_interface(self) -> Result<Box<dyn JtagInterface>, Error>;
}

impl<T: jtag_taps::cable::Cable + 'static> Backend for T {
    fn into_interface(self) -> Result<Box<dyn JtagInterface>, Error> {
        let wrapper = CableWrapper(self);
        let mut sm: Box<dyn JtagInterface> =
            Box::new(jtag_taps::statemachine::JtagSM::new(wrapper));
        sm.mode_reset()?;
        Ok(sm)
    }
}

struct CableWrapper<T: jtag_taps::cable::Cable>(T);

impl<T: jtag_taps::cable::Cable> Deref for CableWrapper<T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<T: jtag_taps::cable::Cable> DerefMut for CableWrapper<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
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
    fn switch_mode(&mut self, new_state: JtagState) -> Result<(), Error> {
        self.change_mode(new_state);
        self.cable.flush();
        Ok(())
    }

    fn write_bits(&mut self, reg: Register, bits: &BitVec<u8, Lsb0>) -> Result<(), Error> {
        if bits.is_empty() {
            return Ok(());
        }
        let bits = get_aligned(bits);
        self.write_reg(reg, bits.as_raw_slice(), last_byte_bits(&bits), true);
        self.cable.flush();
        Ok(())
    }

    fn read_bits(&mut self, reg: Register, bits: &mut BitVec<u8, Lsb0>) -> Result<(), Error> {
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
    ) -> Result<(), Error> {
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

    fn batch(
        &mut self,
        count: usize,
        writer: &mut dyn FnMut(usize, &mut BitVec<u8, Lsb0>) -> Result<(), Error>,
        reader: &mut dyn FnMut(usize, &BitVec<u8, Lsb0>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        // Do this on error. this cannot be replaced by uses of `Cable::flush`
        // which is incompatible with the exposed queueing methods.
        let drain = |sm: &mut Self, queue: &mut VecDeque<(usize, usize)>| {
            while let Some((_, len)) = queue.pop_front() {
                let _ = sm.cable.finish_read(len); // discard
            }
        };

        let mut writer_buf = BitVec::new();
        let mut reader_buf = BitVec::new();
        let mut queue = VecDeque::new();
        let mut i_write = 0; // index of the next write
        let mut pending = false; // writer_buf holds a not yet queued transfer
        while i_write < count || !queue.is_empty() {
            if i_write < count {
                if !pending {
                    writer_buf.clear();
                    writer_buf.force_align();
                    if let Err(e) = writer(i_write, &mut writer_buf) {
                        drain(self, &mut queue);
                        return Err(e);
                    }
                    if writer_buf.is_empty() {
                        drain(self, &mut queue);
                        return Err("empty transfer".into());
                    }
                    writer_buf.force_align();
                    pending = true;
                }
                if self.queue_read_write(
                    Data,
                    writer_buf.as_raw_slice(),
                    last_byte_bits(&writer_buf),
                    true,
                ) {
                    self.change_mode(JtagState::Idle);
                    queue.push_back((i_write, writer_buf.len()));
                    i_write += 1;
                    pending = false;
                    continue;
                } else if queue.is_empty() {
                    return Err("transfer does not fit into the cable's queue".into());
                }
            }
            while let Some((i, len)) = queue.pop_front() {
                let res = self.cable.finish_read(len);
                reader_buf.resize(len, false);
                reader_buf.force_align();
                reader_buf.as_raw_mut_slice().copy_from_slice(&res);
                if let Err(e) = reader(i, &reader_buf) {
                    drain(self, &mut queue);
                    return Err(e);
                }
            }
        }
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
