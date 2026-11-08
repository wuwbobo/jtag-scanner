use std::{
    cell::RefCell,
    rc::{Rc, Weak},
};

use bitvec::{order::Lsb0, vec::BitVec};

use crate::{
    BsrEdit, BsrView, ChainView, Error,
    ErrorKind::{self, GroupLimit, GroupingConflict},
    PinId, PinIdIter, PinScanner, PinState, TapCmd, TapInfo, TapState, UpdateMode,
};

/// Freezes the [states](crate::TapState) of TAPs and keeps a list of
/// pin usages of existing [PinGroup]s that depends upon the session.
///
/// Created by [PinScanner::enter_session]. Multi-thread operation
/// is unsupported.
///
/// In a session, pins in an existing [PinGroup] cannot be written
/// outside the group.
pub struct PinScanSession {
    inner: Rc<RefCell<PinScanSessionInner>>,
    tap_infos: Vec<TapInfo>,
    tap_cmds: Vec<TapCmd>,
}

struct PinScanSessionInner {
    scanner: PinScanner,
    usages: Vec<PinUsage>,
}

#[derive(Debug)]
struct PinUsage {
    pin_id: PinId,
    writable: bool,
    count: usize,
}

impl PinScanSession {
    pub(crate) fn new(scanner: PinScanner) -> Self {
        let tap_infos = Vec::from(scanner.tap_infos());
        let tap_cmds = Vec::from(scanner.tap_cmds());
        Self {
            inner: Rc::new(RefCell::new(PinScanSessionInner {
                scanner,
                usages: Vec::new(),
            })),
            tap_infos,
            tap_cmds,
        }
    }

    /// Count of TAPs on the chain.
    pub fn tap_count(&self) -> usize {
        self.tap_infos.len()
    }

    /// Information for TAPs on the chain.
    pub fn tap_infos(&self) -> &[TapInfo] {
        &self.tap_infos
    }

    /// [TapInfo] of a TAP; the index is of [Self::tap_infos].
    pub fn get_tap_info(&self, i: usize) -> Result<TapInfo, Error> {
        ChainView::get_tap_info(self, i)
    }

    /// Iterates through all [PinId]s for a TAP in the chain.
    /// `i_tap` can be determined by finding in [Self::tap_infos].
    pub fn pin_ids_for_tap(&self, i_tap: usize) -> Result<PinIdIter, Error> {
        ChainView::pin_ids_for_tap(self, i_tap)
    }

    /// Gets a pin identifier by its port name in the BSDL file.
    /// `i_tap` can be determined by finding in [Self::tap_infos].
    ///
    /// For a pin in a bus port, `pin_name` should be "PORT_NAME(i)".
    pub fn get_pin_id(&self, i_tap: usize, pin_name: &str) -> Result<PinId, Error> {
        ChainView::get_pin_id(self, i_tap, pin_name)
    }

    /// Current state of a TAP; the index is of [Self::tap_infos].
    pub fn get_tap_state(&self, i: usize) -> Result<TapState, Error> {
        ChainView::get_tap_state(self, i)
    }

    /// Gets the pin state at the timepoint before the last [Self::update] call
    /// applied newly set pin states.
    pub fn get_pin_state(&self, pin_id: &PinId) -> Result<PinState, Error> {
        self.inner.try_borrow()?.scanner.get_pin_state(pin_id)
    }

    /// Sets the new pin state that will be applied on the next [Self::update] call.
    /// Only for setting output-capable pins not used in any group under EXTEST mode.
    ///
    /// See [PinScanner::set_pin_state] for the meaning of `new_state`.
    pub fn set_pin_state(&mut self, pin_id: &PinId, new_state: Option<bool>) -> Result<(), Error> {
        let mut inner = self.inner.try_borrow_mut()?;
        if inner.usages.iter().any(|u| &u.pin_id == pin_id) {
            return Error::result(ErrorKind::GroupingConflict, "pin used in a group");
        }
        inner.scanner.set_pin_state(pin_id, new_state)
    }

    /// Applies [Self::set_pin_state] calls since last [Self::update] and refreshes
    /// the capture buffer for later [Self::get_pin_state] calls, according to the
    /// [UpdateMode].
    pub fn update(&mut self, mode: UpdateMode) -> Result<(), Error> {
        self.inner.try_borrow_mut()?.scanner.update(mode)
    }

    /// Creates a pin group to be a handler of a protocol adapter.
    ///
    /// Writable pin items prevents new groups from using these pins;
    /// non-writable pin items prevents new groups from using these
    /// pins for writing.
    pub fn group(
        &mut self,
        pins: impl IntoIterator<Item = PinGroupItem>,
    ) -> Result<PinGroup, Error> {
        let mut inner = self.inner.try_borrow_mut()?;
        let mut vec_pins = Vec::new();
        let mut mask = AccessMask::new(inner.scanner.tap_infos());
        for item in pins.into_iter() {
            inner.scanner.check_pin_id(&item.pin_id, item.writable)?;
            if vec_pins
                .iter()
                .any(|existing| item.conflicts_with(existing))
            {
                return Error::result(
                    GroupingConflict,
                    "duplicated pins found in the supposed group",
                );
            }
            mask.add_pin(&item.pin_id, item.writable);
            inner.occupy_for_item(&item)?;
            vec_pins.push(item);
        }
        Ok(PinGroup {
            session: Rc::downgrade(&self.inner),
            pins: vec_pins,
            mask,
        })
    }

    /// Exits the session, invalidating all derived [PinGroup]s.
    pub fn exit(self) -> PinScanner {
        Rc::into_inner(self.inner).unwrap().into_inner().scanner
    }
}

impl PinScanSessionInner {
    fn occupy_for_item(&mut self, group_item: &PinGroupItem) -> Result<(), Error> {
        if let Some(usage) = self
            .usages
            .iter_mut()
            .find(|u| u.pin_id == group_item.pin_id)
        {
            if usage.writable || group_item.writable {
                return Error::result(GroupingConflict, "pin used in another group");
            }
            usage.count += 1;
            Ok(())
        } else {
            self.usages.push(PinUsage {
                pin_id: group_item.pin_id.clone(),
                writable: group_item.writable(),
                count: 1,
            });
            Ok(())
        }
    }

    fn release_for_item(&mut self, group_item: &PinGroupItem) {
        let usage = self
            .usages
            .extract_if(.., |u| u.pin_id == group_item.pin_id)
            .next();
        if let Some(mut usage) = usage {
            if usage.writable {
                assert_eq!(usage.count, 1);
            } else if usage.count > 1 {
                usage.count -= 1;
                self.usages.push(usage);
            }
        }
    }
}

impl ChainView for PinScanSession {
    fn tap_infos(&self) -> &[TapInfo] {
        &self.tap_infos
    }
    fn tap_cmds(&self) -> &[TapCmd] {
        &self.tap_cmds
    }
    fn access_mask(&self) -> Option<&AccessMask> {
        None
    }
}

/// Groups pins and assign given names to each pin, restricting the usage of
/// [PinScanner] within the group. Created by [PinScanSession::group].
pub struct PinGroup {
    session: Weak<RefCell<PinScanSessionInner>>,
    pins: Vec<PinGroupItem>,
    mask: AccessMask,
}

/// Items required during construction of [PinGroup].
#[derive(Debug)]
pub struct PinGroupItem {
    name: String,
    pin_id: PinId,
    writable: bool,
}

impl PinGroupItem {
    /// Creates a supposed group item.
    /// - `name`: the given name for the pin, like `MISO`, `SCK`, or `D0`.
    /// - `writable`: whether write access is allowed for this pin.
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

impl PinGroup {
    /// Iterates through all [PinGroupItem]s in the group.
    pub fn pin_ids(&self) -> impl Iterator<Item = &PinGroupItem> {
        self.pins.iter()
    }

    /// Checks if the pin is writable in this group.
    pub fn pin_writable(&self, pin_id: &PinId) -> Result<bool, Error> {
        self.pin_ids()
            .find_map(|item| (&item.pin_id == pin_id).then_some(item.writable))
            .ok_or(Error::new(GroupLimit, "this pin is not in the group"))
    }

    /// Gets a pin identifier by its name given in the group.
    pub fn get_pin_id(&self, pin_name: &str) -> Result<PinId, Error> {
        let pin_name = pin_name.trim().to_uppercase();
        self.pin_ids()
            .find(|item| item.name == pin_name)
            .ok_or_else(|| "given name '{pin_name}' not found in group".into())
            .map(|item| item.pin_id().clone())
    }

    /// Similar to [PinScanner::get_pin_state], disabling access to pins
    /// outside the group.
    pub fn get_pin_state(&self, pin_id: &PinId) -> Result<PinState, Error> {
        let session = self.get_session()?;
        let session = session.try_borrow()?;
        let bsr_view = BsrView::new(&session.scanner, Some(&self.mask));
        bsr_view.get(pin_id)
    }

    /// Similar to [PinScanner::set_pin_state], disabling access to pins
    /// outside the group, or pins without write access.
    pub fn set_pin_state(&mut self, pin_id: &PinId, new_state: Option<bool>) -> Result<(), Error> {
        let session = self.get_session()?;
        let mut session = session.try_borrow_mut()?;
        let mut bsr_edit = BsrEdit::new(&mut session.scanner, Some(&self.mask));
        bsr_edit.set(pin_id, new_state)
    }

    /// See [PinScanner::update].
    pub fn update(&mut self, mode: UpdateMode) -> Result<(), Error> {
        let session = self.get_session()?;
        let mut session = session.try_borrow_mut()?;
        session.scanner.update(mode)
    }

    /// Similar to [PinScanner::batch], disabling access to pins outside
    /// the group; disables writes to pins without write access.
    pub fn batch(
        &mut self,
        count: usize,
        mode: UpdateMode,
        mut writer: impl FnMut(usize, &mut BsrEdit) -> Result<(), Error>,
        mut reader: impl FnMut(usize, &BsrView) -> Result<(), Error>,
    ) -> Result<(), Error> {
        let session = self.get_session()?;
        let mut session = session.try_borrow_mut()?;
        session.scanner.batch_with_mask(
            count,
            mode,
            |i, edit| writer(i, edit),
            |i, view| reader(i, view),
            Some(&self.mask),
        )
    }

    fn get_session(&self) -> Result<Rc<RefCell<PinScanSessionInner>>, Error> {
        self.session.upgrade().ok_or(Error::new(
            ErrorKind::SessionEnded,
            "pin group is invalidated",
        ))
    }
}

impl Drop for PinGroup {
    fn drop(&mut self) {
        if let Ok(session) = self.get_session()
            && let Ok(mut session) = session.try_borrow_mut()
        {
            for item in self.pin_ids() {
                session.release_for_item(item);
            }
        }
    }
}

/// This is just an optimized alternative of iterating through pins
/// in the group to check for permission. It should keep consistent
/// with the pins list.
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

    pub fn check_access(&self, pin_id: &PinId, for_write: bool) -> Result<(), Error> {
        let mask = &self.0[pin_id.tap_index()];
        let valid = if !for_write {
            pin_id.cell_i().is_some_and(|i| mask[i])
        } else {
            pin_id.cell_o().is_some_and(|o| mask[o])
        };
        if valid {
            Ok(())
        } else {
            Error::result(
                ErrorKind::GroupLimit,
                "operation not permitted by the pin group",
            )
        }
    }
}
