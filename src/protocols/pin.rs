//! Adapters for [embedded_hal::digital]. This is the only protocol module
//! that is not behind another feature gate; it is also the only protocol
//! module that does not use [PinGroup](crate::PinGroup).

use std::{cell::RefCell, rc::Rc};

use embedded_hal::digital::{
    OutputPin,
    PinState::{High, Low},
};

use crate::{Error, ErrorKind::WrongPinType, PinId, PinScanner};

impl embedded_hal::digital::Error for Error {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
}

/// Input-only pin that implements [embedded_hal::digital::InputPin].
pub struct In {
    scanner: Rc<RefCell<PinScanner>>,
    pin_id: PinId,
}

impl In {
    /// Creates the adapter.
    pub fn new(scanner: &Rc<RefCell<PinScanner>>, pin_id: PinId) -> Result<Self, Error> {
        if !pin_id.is_input() {
            return Error::result(WrongPinType, "not input pin");
        }
        Ok(Self {
            scanner: scanner.clone(),
            pin_id,
        })
    }
}

impl embedded_hal::digital::ErrorType for In {
    type Error = Error;
}

impl embedded_hal::digital::InputPin for In {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        let mut scanner = self.scanner.try_borrow_mut()?;
        scanner.update()?;
        scanner
            .get_pin_state(&self.pin_id)
            .map(|st| st.input.unwrap())
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.is_high().map(|v| !v)
    }
}

/// Output-only pin that implements [embedded_hal::digital::OutputPin].
pub struct Out {
    scanner: Rc<RefCell<PinScanner>>,
    pin_id: PinId,
}

impl Out {
    /// Creates the adapter.
    pub fn new(scanner: &Rc<RefCell<PinScanner>>, pin_id: PinId) -> Result<Self, Error> {
        if !pin_id.is_output() {
            return Error::result(WrongPinType, "not output pin");
        }
        Ok(Self {
            scanner: scanner.clone(),
            pin_id,
        })
    }
}

impl embedded_hal::digital::ErrorType for Out {
    type Error = Error;
}

impl embedded_hal::digital::OutputPin for Out {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_state(High)
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_state(Low)
    }
    fn set_state(&mut self, state: embedded_hal::digital::PinState) -> Result<(), Self::Error> {
        let mut scanner = self.scanner.try_borrow_mut()?;
        scanner.set_pin_state(&self.pin_id, Some(state == High))?;
        scanner.write_update()
    }
}

/// Input and output pin that implements [embedded_hal::digital::InputPin]
/// and [embedded_hal::digital::OutputPin].
pub struct InOut {
    scanner: Rc<RefCell<PinScanner>>,
    pin_id: PinId,

    // false if the output should keep disabled on the next `set_state` call.
    output_enable: bool,

    // the value for output, when output is enabled; it is the value to recover
    // when this pin is switched from disabled state to enabled state.
    output_value: Option<bool>,
}

impl InOut {
    /// Creates the adapter.
    /// - `init_val`: this is used to call `PinScanner::set_pin_state` for
    ///   making clear of the initial state. If it is `None`, the output will
    ///   be disabled until [Self::set_output_enable] is called with `true`.
    pub fn new(
        scanner: &Rc<RefCell<PinScanner>>,
        pin_id: PinId,
        init_val: Option<bool>,
    ) -> Result<Self, Error> {
        if !pin_id.is_input() || !pin_id.is_output() {
            return Error::result(WrongPinType, "not input + output pin");
        }

        let mut sc = scanner.try_borrow_mut()?;
        sc.set_pin_state(&pin_id, init_val)?;
        sc.write_update()?;

        Ok(Self {
            scanner: scanner.clone(),
            pin_id,
            output_enable: init_val.is_some(),
            output_value: init_val,
        })
    }

    /// If the pin is tristated, disables output immediately if `val` is false;
    /// otherwise enables output: it is enabled immediately if there is a
    /// previously remembered value; without a previously set value, it is
    /// enabled on the next `set_state` call (including `set_low`, `set_high`).
    ///
    /// If the pin is not tristated, returns error if `val` is false, otherwise
    /// does nothing.
    pub fn set_output_enable(&mut self, enable: bool) -> Result<(), Error> {
        if enable == self.output_enable {
            return Ok(());
        }
        if !self.pin_id.is_tristates() {
            if !enable {
                return Error::result(WrongPinType, "not tristated pin");
            }
            return Ok(());
        }
        if enable {
            self.output_enable = true;
            if let Some(&last_output) = self.output_value.as_ref() {
                self.set_state(if last_output { High } else { Low })?;
            }
        } else {
            let mut sc = self.scanner.try_borrow_mut()?;
            sc.set_pin_state(&self.pin_id, None)?;
            sc.write_update()?;
            self.output_enable = false;
        }
        Ok(())
    }
}

impl embedded_hal::digital::ErrorType for InOut {
    type Error = Error;
}

impl embedded_hal::digital::InputPin for InOut {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        let mut scanner = self.scanner.try_borrow_mut()?;
        scanner.update()?;
        scanner
            .get_pin_state(&self.pin_id)
            .map(|st| st.input.unwrap())
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.is_high().map(|v| !v)
    }
}

impl embedded_hal::digital::OutputPin for InOut {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_state(High)
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_state(Low)
    }
    fn set_state(&mut self, state: embedded_hal::digital::PinState) -> Result<(), Self::Error> {
        self.output_value = Some(state == High);
        if self.output_enable {
            let mut scanner = self.scanner.try_borrow_mut()?;
            scanner.set_pin_state(&self.pin_id, self.output_value)?;
            scanner.write_update()?;
        }
        Ok(())
    }
}
