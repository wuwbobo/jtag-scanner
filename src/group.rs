use std::{cell::RefCell, rc::Rc};

use smallvec::SmallVec;

use crate::{BsrEdit, BsrView, ChainView, PinId, PinScanner, PinState, jtag::JtagInterface};

/// Groups pins and assign given names to each pin, restricting
/// the usage of [PinScanner] within the group.
pub struct PinGroup<T: JtagInterface> {
    scanner: Rc<RefCell<PinScanner<T>>>,
    pins: SmallVec<[PinGroupItem; 64]>,
}

/// Items required during construction of [PinGroup].
/// - `String`: the given name for the pin, like `MISO`, `SCK`, or `D0`.
/// - `bool`: whether write access is allowed for this pin.
#[derive(Debug)]
pub struct PinGroupItem(pub String, pub PinId, pub bool);

impl<T: JtagInterface> PinGroup<T> {
    /// Builds the [PinGroup] with the scanner managed by `Rc` of `RefCell`.
    /// Multi-thread operation is unsupported.
    ///
    /// **WARNING**: This does not automatically check the added writable pins
    /// against any existing pin group that has the same pin for write access.
    /// This is intended for flexibility; check it manually if needed.
    pub fn build<'a>(
        scanner: &Rc<RefCell<PinScanner<T>>>,
        pins: impl IntoIterator<Item = PinGroupItem>,
    ) -> Result<Self, String> {
        let mut vec_pins: SmallVec<[PinGroupItem; 64]> = SmallVec::new();
        let sc = scanner.borrow();
        for mut item in pins.into_iter() {
            item.0 = item.0.trim().to_uppercase();
            let PinGroupItem(name, pin_id, writable) = &item;
            sc.verify_pin_id(pin_id, *writable)?;
            if vec_pins
                .iter()
                .any(|PinGroupItem(existing, existing_id, _)| {
                    name == existing || existing_id == pin_id
                })
            {
                return Err("duplicated pins in the supposed group".into());
            }
            vec_pins.push(item);
        }
        Ok(Self {
            scanner: scanner.clone(),
            pins: vec_pins,
        })
    }

    /// Iterates through all [PinId]s in the group, with their given names.
    pub fn pin_ids(&self) -> impl Iterator<Item = &PinGroupItem> {
        self.pins.iter()
    }

    /// Gets a pin identifier by its name given in the group.
    pub fn get_pin_id(&self, pin_name: &str) -> Result<PinId, String> {
        let pin_name = pin_name.trim().to_uppercase();
        self.pin_ids()
            .find(|PinGroupItem(name, _, _)| name.to_uppercase() == pin_name)
            .ok_or_else(|| "given name '{pin_name}' not found in group".into())
            .map(|PinGroupItem(_, id, _)| id.clone())
    }

    /// Similar to [PinScanner::get_pin_state], disabling access to pins
    /// outside the group.
    pub fn get_pin_state(&self, pin_id: &PinId) -> Result<PinState, String> {
        let scanner = self.scanner.try_borrow().map_err(|_| "scanner is busy")?;
        let bsr_view = BsrView::new(&scanner, Some(&self.pins));
        bsr_view.get(pin_id)
    }

    /// Similar to [PinScanner::set_pin_state], disabling access to pins
    /// outside the group, or pins without write access.
    pub fn set_pin_state(&mut self, pin_id: &PinId, new_state: Option<bool>) -> Result<(), String> {
        let mut scanner = self
            .scanner
            .try_borrow_mut()
            .map_err(|_| "scanner is busy")?;
        let mut bsr_edit = BsrEdit::new(&mut scanner, Some(&self.pins));
        bsr_edit.set(pin_id, new_state)
    }

    /// See [PinScanner::update].
    pub fn update(&mut self) -> Result<(), String> {
        let mut scanner = self
            .scanner
            .try_borrow_mut()
            .map_err(|_| "scanner is busy")?;
        scanner.update()
    }

    /// See [PinScanner::write_update].
    pub fn write_update(&mut self) -> Result<(), String> {
        let mut scanner = self
            .scanner
            .try_borrow_mut()
            .map_err(|_| "scanner is busy")?;
        scanner.write_update()
    }

    /// Similar to [PinScanner::batch], disabling access to pins outside
    /// the group; disables writes to pins without write access.
    pub fn batch<'a>(
        &'a mut self,
        count: usize,
        mut writer: impl FnMut(usize, &mut BsrEdit) -> Result<(), String> + 'a,
        mut reader: impl FnMut(usize, &BsrView) -> Result<(), String> + 'a,
    ) -> Result<(), String> {
        let mut scanner = self
            .scanner
            .try_borrow_mut()
            .map_err(|_| "scanner is busy")?;
        scanner.batch_with_whitelist(
            count,
            |i, edit| writer(i, edit),
            |i, view| reader(i, view),
            Some(&self.pins),
        )
    }
}
