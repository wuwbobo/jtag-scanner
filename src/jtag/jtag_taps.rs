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

use std::{
    borrow::Cow,
    collections::VecDeque,
    ops::{Deref, DerefMut},
};

use bitvec::{order::Lsb0, vec::BitVec};

use jtag_taps::{cable::Cable, statemachine::JtagSM};

use crate::{
    Error,
    jtag::{Backend, JtagInterface, JtagState, Register},
};

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

impl<T, U> JtagInterface for JtagSM<T>
where
    T: std::ops::DerefMut<Target = U>,
    U: Cable + ?Sized,
{
    fn switch_mode(&mut self, new_state: JtagState, flush: bool) -> Result<(), Error> {
        self.change_mode(new_state);
        if flush {
            self.cable.flush();
        }
        Ok(())
    }

    fn write_bits(
        &mut self,
        reg: Register,
        bits: &BitVec<u8, Lsb0>,
        flush: bool,
    ) -> Result<(), Error> {
        if bits.is_empty() {
            return Ok(());
        }
        let bits = get_aligned(bits);
        self.write_reg(reg, bits.as_raw_slice(), last_byte_bits(&bits), true);
        if flush {
            self.cable.flush();
        }
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
        self.cable.flush();

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
                    Register::Data,
                    writer_buf.as_raw_slice(),
                    last_byte_bits(&writer_buf),
                    true,
                ) {
                    self.change_mode(JtagState::Idle); // queued as well
                    queue.push_back((i_write, writer_buf.len()));
                    i_write += 1;
                    pending = false;
                    continue;
                } else if queue.is_empty() {
                    return Err("transfer does not fit into the cable's queue".into());
                }
            }
            // `Cable::finish_read` says: once you finish a read, finish all the queued reads.
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
