use embedded_hal::spi::{Mode, Polarity, SpiBus};

use crate::{Error, PinGroup, PinId};

#[allow(unused)] // TODO
pub struct Spi {
    pin_group: PinGroup,
    sck: PinId,
    mosi: PinId,
    miso: PinId,
    mode: Mode,
}

impl Spi {
    pub fn new(mut pin_group: PinGroup, mode: Mode) -> Result<Self, Error> {
        let sck = pin_group.get_pin_id("SCK")?;
        let mosi = pin_group.get_pin_id("MOSI")?;
        let miso = pin_group.get_pin_id("MISO")?;
        if !sck.is_output() || !mosi.is_output() || !miso.is_input() {
            return Error::result(crate::ErrorKind::WrongPinType, "bad SPI pin config");
        }
        pin_group.set_pin_state(&sck, Some(mode.polarity == Polarity::IdleHigh))?;
        if miso.is_output() {
            pin_group.set_pin_state(&miso, None)?;
        }
        pin_group.update()?;
        Ok(Self {
            pin_group,
            sck,
            miso,
            mosi,
            mode,
        })
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
    fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
        todo!()
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
