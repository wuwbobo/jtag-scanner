use bitvec::{order::Lsb0, vec::BitVec};
use std::sync::Arc;

use crate::Error;

/// Needed information for each TAP on the chain.
#[derive(Debug, Clone)]
pub enum TapInfo {
    /// Parsed BSDL information.
    WithBsdl(Arc<bsdl::Entity>),
    /// IDCODE (`u32`) and IR length (`usize`) for a TAP without known BSDL.
    AlwaysBypass(u32, usize),
}

impl TapInfo {
    /// The TAP's IR length.
    pub fn ir_len(&self) -> usize {
        match self {
            Self::WithBsdl(entity) => entity.instruction_length(),
            Self::AlwaysBypass(_, ir_len) => *ir_len,
        }
    }

    /// The BSDL entity name, or "ALWAYS_BYPASS" for `AlwaysBypass`.
    pub fn name(&self) -> &str {
        match self {
            Self::WithBsdl(entity) => entity.name().trim(),
            Self::AlwaysBypass(_, _) => "ALWAYS_BYPASS",
        }
    }

    /// DR length when IR is written by `cmd`.
    pub(crate) fn dr_len(&self, cmd: TapCmd) -> usize {
        let Self::WithBsdl(entity) = self else {
            return 1;
        };
        match cmd {
            TapCmd::Bypass => 1,
            TapCmd::Idcode => 32,
            TapCmd::Sample | TapCmd::Preload | TapCmd::Extest => entity.boundary_length(),
        }
    }

    pub(crate) fn matches_idcode(&self, code: u32) -> bool {
        match self {
            Self::WithBsdl(entity) => jtag_idcode::IDCode::new(code) == entity.idcode_register()[0],
            Self::AlwaysBypass(idcode, _) => code == *idcode,
        }
    }

    /// Converts `cmd` to the IR bit sequence for the `cmd` found in BSDL,
    /// and push them into `buf_ir`. Pushes all ones for `AlwaysBypass`.
    pub(crate) fn push_opcode(&self, cmd: TapCmd, buf_ir: &mut BitVec<u8, Lsb0>) {
        let opcode = if let Self::WithBsdl(bsdl) = self {
            match cmd {
                TapCmd::Bypass => &u64::MAX,
                TapCmd::Idcode => bsdl.opcodes().get("IDCODE").unwrap(),
                TapCmd::Sample => bsdl.opcodes().get("SAMPLE").unwrap(),
                TapCmd::Preload => bsdl
                    .opcodes()
                    .get("PRELOAD")
                    .unwrap_or_else(|| bsdl.opcodes().get("SAMPLE").unwrap()),
                TapCmd::Extest => bsdl.opcodes().get("EXTEST").unwrap(),
            }
        } else {
            &u64::MAX
        };
        for i in 0..self.ir_len() {
            let bit = (opcode >> i) & 1;
            buf_ir.push(bit > 0);
        }
    }

    /// The safe BSR values to scan in prior to EXTEST.
    pub(crate) fn safe_bsr_value(&self) -> BitVec<u8, Lsb0> {
        if let Self::WithBsdl(bsdl) = self {
            let mut buf = BitVec::repeat(true, bsdl.boundary_length());
            buf.force_align();
            bsdl.get_safe(buf.as_raw_mut_slice()).unwrap();
            buf
        } else {
            BitVec::repeat(true, 1)
        }
    }

    /// Used for binding the [TapId] with the [crate::TapScanner].
    pub(crate) fn sessioned_tap_id(&self) -> usize {
        let Self::WithBsdl(entity) = self else {
            return 0;
        };
        Arc::as_ptr(entity) as _
    }
}

/// Supported TAP states for normal operations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TapState {
    Bypass,
    Sample,
    Extest,
}

/// TAP states for operations, including transition commands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum TapCmd {
    Bypass,
    Idcode,
    Sample,
    Preload,
    Extest,
}

impl From<TapState> for TapCmd {
    fn from(value: TapState) -> Self {
        match value {
            TapState::Bypass => Self::Bypass,
            TapState::Sample => Self::Sample,
            TapState::Extest => Self::Extest,
        }
    }
}

/// Unique pin identifier associated with a [PinScanner](crate::PinScanner)
/// instance. This is not intended to be persistent or serializable.
#[derive(Debug, Clone)]
pub struct PinId {
    i_tap: usize,
    bsdl: Arc<bsdl::Entity>,
    i_port: usize,
    cell_i: Option<usize>,
    cell_o: Option<usize>,
    cell_oe: Option<usize>,
    cell_oe_disabler: bool,
}

impl PinId {
    /// TAP index in the chain, starting from 0 (closest to TDI).
    pub fn tap_index(&self) -> usize {
        self.i_tap
    }
    /// Gets all port information in BSDL.
    pub fn port_info(&self) -> &bsdl::Port {
        &self.bsdl.ports()[self.i_port]
    }
    /// Port name in BSDL.
    pub fn name(&self) -> &str {
        &self.port_info().name
    }
    /// Pin number in BSDL.
    pub fn pin(&self) -> &str {
        &self.port_info().pin
    }
    /// True if it has an input cell.
    pub fn is_input(&self) -> bool {
        self.cell_i.is_some()
    }
    /// True if it has an output cell.
    pub fn is_output(&self) -> bool {
        self.cell_o.is_some()
    }
    /// True if it has an output enable cell.
    pub fn is_tristates(&self) -> bool {
        self.cell_oe.is_some()
    }

    pub(crate) fn cell_i(&self) -> Option<usize> {
        self.cell_i
    }
    pub(crate) fn cell_o(&self) -> Option<usize> {
        self.cell_o
    }
    pub(crate) fn cell_oe(&self) -> Option<usize> {
        self.cell_oe
    }
    /// The value to be updated in the OE cell in order to disable
    /// the corresponding output cell (usually turning it to high-Z).
    pub(crate) fn cell_oe_disabler(&self) -> bool {
        self.cell_oe_disabler
    }

    /// Used for binding the [TapId] with the [crate::TapScanner].
    pub(crate) fn sessioned_tap_id(&self) -> usize {
        Arc::as_ptr(&self.bsdl) as _
    }

    /// Finds a port definition named by `pin_name` and builds a `PinId`.
    pub(crate) fn from_name(
        i_tap: usize,
        bsdl: Arc<bsdl::Entity>,
        pin_name: &str,
    ) -> Result<PinId, Error> {
        let pin_name = pin_name.trim().to_uppercase();
        if let Some(i_port) = bsdl
            .ports()
            .iter()
            .position(|port| port.name.trim().to_uppercase() == pin_name)
        {
            Self::from_port_index(i_tap, bsdl, i_port)
        } else {
            Err(format!("port name '{}' not found in BSDL", pin_name).into())
        }
    }

    pub(crate) fn from_port_index(
        i_tap: usize,
        bsdl: Arc<bsdl::Entity>,
        i_port: usize,
    ) -> Result<PinId, Error> {
        let port = &bsdl.ports()[i_port];
        use bsdl::PortDirection::*;
        let cell_i = if matches!(port.dir, In | Inout) {
            port.cell_i
        } else {
            None
        };
        let cell_o = if matches!(port.dir, Out | Inout | Buffer) {
            port.cell_o
        } else {
            None
        };
        let cell_oe = if matches!(port.dir, Out | Inout) {
            port.cell_o.and_then(|c| bsdl.boundary_register()[c].ccell)
        } else {
            None
        };
        let cell_oe_disabler = if cell_oe.is_some() {
            bsdl.boundary_register()[port.cell_o.unwrap()].disval == bsdl::LogicVal::One
        } else {
            true
        };
        if cell_i.is_none() && cell_o.is_none() {
            return Err("input cell/output cell not found in port definition".into());
        }
        Ok(PinId {
            i_tap,
            bsdl,
            i_port,
            cell_i,
            cell_o,
            cell_oe,
            cell_oe_disabler,
        })
    }
}

impl PartialEq for PinId {
    fn eq(&self, other: &Self) -> bool {
        self.sessioned_tap_id() == other.sessioned_tap_id() && self.i_port == other.i_port
    }
}

/// Iterates through all [PinId]s for a TAP in the chain handled by
/// [PinScanner](crate::PinScanner).
#[derive(Debug)]
pub struct PinIdIter {
    i_tap: usize,
    bsdl: Arc<bsdl::Entity>,
    i_port: usize, // next `i_port` to be checked
}

impl PinIdIter {
    pub(crate) fn new(i_tap: usize, bsdl: Arc<bsdl::Entity>) -> Self {
        Self {
            i_tap,
            bsdl,
            i_port: 0,
        }
    }
}

impl Iterator for PinIdIter {
    type Item = PinId;
    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.i_port >= self.bsdl.ports().len() {
                return None;
            }
            let id = PinId::from_port_index(self.i_tap, self.bsdl.clone(), self.i_port);
            self.i_port += 1;
            if let Ok(id) = id {
                return Some(id);
            }
        }
    }
}

/// Lastly captured states of scan cells of a pin.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PinState {
    pub(crate) input: Option<bool>,
    pub(crate) output: Option<bool>,
    pub(crate) output_enable: Option<bool>,
}

impl PinState {
    /// This is the actual input state.
    pub fn input(&self) -> Option<bool> {
        self.input
    }
    /// The intended output value from the internal logic.
    pub fn output(&self) -> Option<bool> {
        self.output
    }
    /// The intended output enable value from the internal logic.
    pub fn output_enable(&self) -> Option<bool> {
        self.output_enable
    }
}
