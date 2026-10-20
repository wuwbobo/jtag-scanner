// If some data is written by `write_reg` with `pause_after: false`, the `Cable` implementation
// will shift the last bit of data and stay in ShiftDR; then it will shift another bit if
// `change_mode(UpdateDR)` is called right after that. So this is a problematic usage;
// `pause_after: false` is just for multiple writes, and the last write before changing mode to
// UpdateDR should have `pause_after: true`, to make the last bit of data shift by the Exit1DR
// internally. So, `read_bits` and `read_write_bits` sets `pause_after` to true internally.

use bitvec::{order::Lsb0, vec::BitVec};
use jtag_taps::{
    cable::Cable,
    statemachine::{
        JtagSM,
        Register::{self, Data, Instruction},
    },
};

pub(crate) trait JtagSMExt {
    /// NOTE: this puts all TAPs in BYPASS mode afterwards.
    fn measure_ir_len(&mut self) -> usize;
    /// NOTE: this can be used only if updating the current DR chain with all 0s
    /// is safe to do, e.g. when measuring the chain of IDCODE registers.
    fn measure_dr_len(&mut self) -> usize;
    /// Shifts `bits` of ones or zeros, without automatic UpdateIR or UpdateDR.
    /// `one_or_zero` can be 0 or 1.
    fn write_repeated(&mut self, reg: Register, one_or_zero: u8, bits: usize);

    fn write_bits(&mut self, reg: Register, bits: &BitVec<u8, Lsb0>);
    fn read_bits(&mut self, reg: Register, bits: &mut BitVec<u8, Lsb0>);
    fn read_write_bits(
        &mut self,
        reg: Register,
        read_bits: &mut BitVec<u8, Lsb0>,
        write_bits: &BitVec<u8, Lsb0>,
    );
}

impl<T, U> JtagSMExt for JtagSM<T>
where
    T: std::ops::DerefMut<Target = U>,
    U: Cable + ?Sized,
{
    fn measure_ir_len(&mut self) -> usize {
        let mut ir_len = 0;
        // fill IR chain with 0s, 1024 bits should be enough for any case
        self.write_repeated(Instruction, 0, 1024);
        // enter 1s one by one, stop counting when first 1 reaches the other side
        loop {
            let b = self.read_write_reg(Instruction, &[1u8], 1, false);
            if b[0] == 1u8 {
                break;
            }
            ir_len += 1;
        }

        ir_len
    }

    fn measure_dr_len(&mut self) -> usize {
        let mut dr_len = 0;
        // fill DR chain with 1s, 65536 bits should be enough for any case
        self.write_repeated(Data, 1, 65536);
        // enter 0s one by one, stop counting when first 0 reaches the other side
        loop {
            let b = self.read_write_reg(Data, &[0u8], 1, false);
            if b[0] == 0u8 {
                break;
            }
            dr_len += 1;
        }
        dr_len
    }

    fn write_repeated(&mut self, reg: Register, one_or_zero: u8, bits: usize) {
        let data = BitVec::repeat(one_or_zero > 0, bits);
        self.write_bits(reg, &data);
    }

    fn write_bits(&mut self, reg: Register, bits: &BitVec<u8, Lsb0>) {
        let last_byte_bits = if bits.len().is_multiple_of(8) {
            8
        } else {
            (bits.len() % 8) as u8
        };
        self.write_reg(reg, bits.as_raw_slice(), last_byte_bits, true);
    }

    fn read_bits(&mut self, reg: Register, bits: &mut BitVec<u8, Lsb0>) {
        let bits_read = self.read_reg(reg, bits.len());
        bits.as_raw_mut_slice().copy_from_slice(&bits_read);
    }

    fn read_write_bits(
        &mut self,
        reg: Register,
        read_bits: &mut BitVec<u8, Lsb0>,
        write_bits: &BitVec<u8, Lsb0>,
    ) {
        let last_byte_bits = if write_bits.len().is_multiple_of(8) {
            8
        } else {
            (write_bits.len() % 8) as u8
        };
        let bits_read = self.read_write_reg(reg, write_bits.as_raw_slice(), last_byte_bits, true);
        read_bits.resize(write_bits.len(), false);
        read_bits.force_align();
        read_bits.as_raw_mut_slice().copy_from_slice(&bits_read);
    }
}
