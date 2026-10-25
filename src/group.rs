use std::{cell::RefCell, rc::Rc};

use bitvec::{order::Lsb0, vec::BitVec};

use crate::{
    BsrEdit, BsrView, ChainView, PinId, PinScanner, PinState, TapCmd, TapInfo, jtag::JtagInterface,
};

/// Groups pins and assign given names to each pin, restricting the
/// usage of [PinScanner] within the group. Implements [PinGroup].
pub struct PinGrouper<T: JtagInterface> {
    scanner: Rc<RefCell<PinScanner<T>>>,
    pins: Vec<PinGroupItem>,
    mask: AccessMask,
}

/// Items required during construction of [PinGroup].
/// - `String`: the given name for the pin, like `MISO`, `SCK`, or `D0`.
/// - `bool`: whether write access is allowed for this pin.
#[derive(Debug)]
pub struct PinGroupItem {
    name: String,
    pin_id: PinId,
    writable: bool,
}

impl PinGroupItem {
    /// Creates an item.
    pub fn new(name: &str, pin_id: PinId, writable: bool) -> Self {
        Self {
            name: name.to_string().trim().to_uppercase(),
            pin_id,
            writable,
        }
    }

    /// The given name.
    pub fn name(&self) -> &str {
        &self.name
    }
    /// The ID for the actual pin in the current chain.
    pub fn pin_id(&self) -> &PinId {
        &self.pin_id
    }
    /// True if write access should be enabled in the group.
    pub fn writable(&self) -> bool {
        self.writable
    }

    /// Returns true if `self` and `other` cannot exist in the same group.
    pub fn conflicts_with(&self, other: &PinGroupItem) -> bool {
        self.name == other.name || self.pin_id == other.pin_id
    }
}

/// Trait for IO adpters to operate on the JTAG boundary scanner.
pub trait PinGroup {
    /// Iterates through all [PinId]s in the group, with their given names.
    fn pin_ids(&self) -> impl Iterator<Item = &PinGroupItem>;

    /// Gets a pin identifier by its name given in the group.
    fn get_pin_id(&self, pin_name: &str) -> Result<PinId, String>;

    /// Similar to [PinScanner::get_pin_state], disabling access to pins
    /// outside the group.
    fn get_pin_state(&self, pin_id: &PinId) -> Result<PinState, String>;

    /// Similar to [PinScanner::set_pin_state], disabling access to pins
    /// outside the group, or pins without write access.
    fn set_pin_state(&mut self, pin_id: &PinId, new_state: Option<bool>) -> Result<(), String>;

    /// See [PinScanner::update].
    fn update(&mut self) -> Result<(), String>;

    /// See [PinScanner::write_update].
    fn write_update(&mut self) -> Result<(), String>;

    /// Similar to [PinScanner::batch], disabling access to pins outside
    /// the group; disables writes to pins without write access.
    fn batch(
        &mut self,
        count: usize,
        writer: impl FnMut(usize, &mut BsrEdit) -> Result<(), String>,
        reader: impl FnMut(usize, &BsrView) -> Result<(), String>,
    ) -> Result<(), String>;
}

impl<T: JtagInterface> PinGrouper<T> {
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
        for item in pins.into_iter() {
            sc.verify_pin_id(&item.pin_id, item.writable)?;
            if vec_pins
                .iter()
                .any(|existing| item.conflicts_with(existing))
            {
                return Err("duplicated pins in the supposed group".into());
            }
            mask.add_pin(&item.pin_id, item.writable);
            vec_pins.push(item);
        }
        Ok(Self {
            scanner: scanner.clone(),
            pins: vec_pins,
            mask,
        })
    }
}

impl<T: JtagInterface> PinGroup for PinGrouper<T> {
    fn pin_ids(&self) -> impl Iterator<Item = &PinGroupItem> {
        self.pins.iter()
    }

    fn get_pin_id(&self, pin_name: &str) -> Result<PinId, String> {
        let pin_name = pin_name.trim().to_uppercase();
        self.pin_ids()
            .find(|item| item.name == pin_name)
            .ok_or_else(|| "given name '{pin_name}' not found in group".into())
            .map(|item| item.pin_id().clone())
    }

    fn get_pin_state(&self, pin_id: &PinId) -> Result<PinState, String> {
        let scanner = self.scanner.try_borrow().map_err(|_| "scanner is busy")?;
        let bsr_view = BsrView::new(&scanner, Some(&self.mask));
        bsr_view.get(pin_id)
    }

    fn set_pin_state(&mut self, pin_id: &PinId, new_state: Option<bool>) -> Result<(), String> {
        let mut scanner = self
            .scanner
            .try_borrow_mut()
            .map_err(|_| "scanner is busy")?;
        let mut bsr_edit = BsrEdit::new(&mut scanner, Some(&self.mask));
        bsr_edit.set(pin_id, new_state)
    }

    fn update(&mut self) -> Result<(), String> {
        let mut scanner = self
            .scanner
            .try_borrow_mut()
            .map_err(|_| "scanner is busy")?;
        scanner.update()
    }

    fn write_update(&mut self) -> Result<(), String> {
        let mut scanner = self
            .scanner
            .try_borrow_mut()
            .map_err(|_| "scanner is busy")?;
        scanner.write_update()
    }

    fn batch(
        &mut self,
        count: usize,
        mut writer: impl FnMut(usize, &mut BsrEdit) -> Result<(), String>,
        mut reader: impl FnMut(usize, &BsrView) -> Result<(), String>,
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
