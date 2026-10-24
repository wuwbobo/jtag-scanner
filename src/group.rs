use std::{cell::RefCell, rc::Rc};

use bitvec::{order::Lsb0, vec::BitVec};

use crate::{
    BsrEdit, BsrView, ChainView, PinId, PinScanner, PinState, TapCmd, TapInfo, jtag::JtagInterface,
};

/// Groups pins and assign given names to each pin, restricting
/// the usage of [PinScanner] within the group.
pub struct PinGroup<T: JtagInterface> {
    scanner: Rc<RefCell<PinScanner<T>>>,
    pins: Vec<PinGroupItem>,
    mask: AccessMask,
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
    pub fn build(
        scanner: &Rc<RefCell<PinScanner<T>>>,
        pins: impl IntoIterator<Item = PinGroupItem>,
    ) -> Result<Self, String> {
        let sc = scanner.borrow();
        let mut vec_pins = Vec::new();
        let mut mask = AccessMask::new(sc.tap_infos());
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
            mask.add_pin(pin_id, *writable);
            vec_pins.push(item);
        }
        Ok(Self {
            scanner: scanner.clone(),
            pins: vec_pins,
            mask,
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
        let bsr_view = BsrView::new(&scanner, Some(&self.mask));
        bsr_view.get(pin_id)
    }

    /// Similar to [PinScanner::set_pin_state], disabling access to pins
    /// outside the group, or pins without write access.
    pub fn set_pin_state(&mut self, pin_id: &PinId, new_state: Option<bool>) -> Result<(), String> {
        let mut scanner = self
            .scanner
            .try_borrow_mut()
            .map_err(|_| "scanner is busy")?;
        let mut bsr_edit = BsrEdit::new(&mut scanner, Some(&self.mask));
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
        scanner.batch_with_mask(
            count,
            |i, edit| writer(i, edit),
            |i, view| reader(i, view),
            Some(&self.mask),
        )
    }
}

pub(crate) struct AccessMask(Vec<BitVec<u8, Lsb0>>);

impl AccessMask {
    fn new(tap_infos: &[TapInfo]) -> Self {
        let mut perm_mask = vec![BitVec::new(); tap_infos.len()];
        for (i_tap, mask) in perm_mask.iter_mut().enumerate() {
            mask.resize(tap_infos[i_tap].dr_len(TapCmd::Extest), false);
        }
        Self(perm_mask)
    }

    fn add_pin(&mut self, pin_id: &PinId, writable: bool) {
        let mask = &mut self.0[pin_id.tap_index()];
        if let Some(cell_i) = pin_id.cell_i() {
            *mask.get_mut(cell_i).unwrap() = true;
        }
        if writable {
            if let Some(cell_o) = pin_id.cell_o() {
                *mask.get_mut(cell_o).unwrap() = true;
            }
        }
    }

    pub fn check_access(&self, pin_id: &PinId, for_write: bool) -> Result<(), String> {
        let mask = &self.0[pin_id.tap_index()];
        let valid = if !for_write {
            pin_id.cell_i().is_some_and(|i| mask[i])
        } else {
            pin_id.cell_o().is_some_and(|o| mask[o])
        };
        if valid {
            Ok(())
        } else {
            Err("operation not permitted by the pin group".into())
        }
    }
}
