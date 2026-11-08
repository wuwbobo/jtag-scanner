use embedded_hal::spi::{Mode, Phase, Polarity, SpiBus};

use crate::{Error, PinGroup, PinId, UpdateMode};

/// Adapter for [SpiBus].
pub struct Spi {
    pin_group: PinGroup,
    sck: PinId,
    mosi: PinId,
    miso: PinId,
    mode: Mode,
    lsb_first: bool,
}

impl Spi {
    /// Creates the adapter for `SpiBus`. Check `embedded-hal-bus` for
    /// how to turn it into an `SpiDevice`.
    ///
    /// Expects given pin names `SCK`, `MOSI`, `MISO` in the `pin_group`.
    pub fn new(mut pin_group: PinGroup, mode: Mode) -> Result<Self, Error> {
        let sck = pin_group.get_pin_id("SCK")?;
        let mosi = pin_group.get_pin_id("MOSI")?;
        let miso = pin_group.get_pin_id("MISO")?;
        if !sck.is_output()
            || !pin_group.pin_writable(&sck)?
            || !mosi.is_output()
            || !pin_group.pin_writable(&mosi)?
            || !miso.is_input()
        {
            return Error::result(crate::ErrorKind::WrongPinType, "bad SPI pin config");
        }
        pin_group.set_pin_state(&sck, Some(mode.polarity == Polarity::IdleHigh))?;
        if miso.is_output() && pin_group.pin_writable(&miso)? {
            pin_group.set_pin_state(&miso, None)?;
        }
        pin_group.update(UpdateMode::PostRead)?;
        Ok(Self {
            pin_group,
            sck,
            miso,
            mosi,
            mode,
            lsb_first: false,
        })
    }

    /// Set it to true if bits in a byte should be clocked out from LSB to MSB.
    /// False by default.
    pub fn set_lsb_first(&mut self, val: bool) {
        self.lsb_first = val;
    }
}

impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

impl embedded_hal::spi::ErrorType for Spi {
    type Error = Error;
}

impl SpiBus for Spi {
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let len = read.len().max(write.len());
        if len == 0 {
            return Ok(());
        }
        let cpha = self.mode.phase == Phase::CaptureOnSecondTransition;
        let cpol = self.mode.polarity == Polarity::IdleHigh;

        let mut set_bit = |bit_index: usize, val: bool| {
            let i_byte = bit_index / 8;
            if i_byte >= read.len() {
                return; // no-op
            }
            let i_bit = if self.lsb_first {
                bit_index % 8
            } else {
                7 - bit_index % 8
            };
            if val {
                read[i_byte] |= 1 << i_bit;
            } else {
                read[i_byte] &= 0xFF ^ (1 << i_bit);
            }
        };

        let get_bit = |bit_index: usize| -> bool {
            let i_byte = bit_index / 8;
            if i_byte >= write.len() {
                return false; // for padding 0s
            }
            let i_bit = if self.lsb_first {
                bit_index % 8
            } else {
                7 - bit_index % 8
            };
            ((write[i_byte] >> i_bit) & 1) == 1
        };

        // i: 0, 2..: writes CLK to idle(CPHA=0) or non-idle(CPHA=1), writes `i / 2`th bit to MOSI.
        // i: 1, 3..: writes CLK to non-idle(CPHA=0) or idle(CPHA=1), reads `i / 2`th bit from MISO.
        self.pin_group.batch(
            len * 8 * 2,
            UpdateMode::PostRead,
            |i, edit| {
                if i % 2 == 0 {
                    let bit_index = i / 2;
                    edit.set(&self.mosi, Some(get_bit(bit_index)))?;
                    let sck_val = if cpha { !cpol } else { cpol };
                    edit.set(&self.sck, Some(sck_val))?;
                } else {
                    let sck_val = if cpha { cpol } else { !cpol };
                    edit.set(&self.sck, Some(sck_val))?;
                }
                Ok(())
            },
            |i, view| {
                if i % 2 != 0 {
                    let bit_index = i / 2;
                    let bit_read = view.get(&self.miso)?.input().unwrap();
                    set_bit(bit_index, bit_read);
                }
                Ok(())
            },
        )?;
        if !cpha {
            self.pin_group.set_pin_state(&self.sck, Some(cpol))?;
            self.pin_group.update(UpdateMode::PostRead)?;
        }
        Ok(())
    }

    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for w in words.iter_mut() {
            *w = 0x00;
        }
        self.transfer_in_place(words)
    }
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut buf_read = vec![0x00; words.len()];
        self.transfer(&mut buf_read, words)
    }
    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let buf_write = Vec::from(&mut *words);
        self.transfer(words, &buf_write)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
