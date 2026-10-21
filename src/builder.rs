use std::{cell::OnceCell, path::Path, sync::Arc};

use bitvec::vec::BitVec;

use crate::{
    jtag::{
        JtagInterface, JtagState,
        Register::{Data, Instruction},
    },
    scanner::PinScanner,
    types::TapInfo,
};

/// The only way to build [PinScanner]; detects IDCODEs of devices on the chain,
/// and scan for BSDL files.
pub struct PinScannerBuilder<T: JtagInterface> {
    sm: T,
    idcodes: OnceCell<Vec<u32>>,
    tap_infos: Vec<TapInfo>,
    extest_keep_present: bool,
}

impl<T: JtagInterface> PinScannerBuilder<T> {
    /// All configuration for the `sm` needs to be done before wrapping
    /// it with the builder.
    pub fn new(mut sm: T) -> Self {
        let _ = sm
            .mode_reset()
            .inspect_err(|e| eprintln!("sm.mode_reset() failed: {e}"));
        Self {
            sm,
            idcodes: OnceCell::new(),
            tap_infos: Vec::new(),
            extest_keep_present: false,
        }
    }

    /// Scans for BSDL files under `path` nonrecursively and tries to build [TapInfo]s
    /// for all TAPs on the chain. Overrides infos added by [Self::add_tap_info].
    ///
    /// Supports forced bypass of some TAPs, which needs empty bypass files with naming
    /// `<id_code_hex>_<ir_len>.bypass`.
    pub fn scan_tap_info(&mut self, path: &Path) -> Result<(), String> {
        if !path.exists() || !path.is_dir() {
            return Err("Invalid path".into());
        }
        let mut bsdls = Vec::new();
        let mut bypassed = Vec::new();
        for entry in path.read_dir().map_err(|e| e.to_string())? {
            if let Ok(entry) = entry
                && let Ok(file_type) = entry.file_type()
                && file_type.is_file()
                && let Some(ext) = entry.path().extension()
            {
                match ext.to_string_lossy().to_lowercase().as_str() {
                    "bsd" | "bsdl" => match bsdl::Entity::parse(&entry.path(), None) {
                        Ok(bsdl) => bsdls.push((bsdl, entry.path())),
                        Err(e) => println!("Error parsing {}: {e}", entry.file_name().display()),
                    },
                    "bypass" => {
                        let path = entry.path();
                        let stem = path.file_stem().unwrap_or_default().to_string_lossy();
                        let mut stem_iter = stem.split('_');
                        let (Some(idcode), Some(ir_len)) = (stem_iter.next(), stem_iter.next())
                        else {
                            println!("Error parsing file name {}", entry.file_name().display());
                            continue;
                        };
                        let (Ok(idcode), Ok(ir_len)) =
                            (u32::from_str_radix(idcode, 16), ir_len.parse())
                        else {
                            println!("Error parsing file name {}", entry.file_name().display());
                            continue;
                        };
                        bypassed.push((idcode, ir_len));
                    }
                    _ => (),
                }
            }
        }

        let mut tap_infos = Vec::new();
        for idcode in self.get_id_codes()? {
            if let Some((_, path)) = bsdls
                .iter()
                .find(|(i, _)| i.idcode_register()[0] == jtag_idcode::IDCode::new(idcode))
            {
                // NOTE: this makes sure that every `TapInfo::WithBsdl` in the chain
                // has an `Arc` of a different address, used for identifying `PinId`.
                // At the time of writing this code, `bsdl::Entity` is not clonable.
                tap_infos.push(TapInfo::WithBsdl(Arc::new(
                    bsdl::Entity::parse(path, None).unwrap(),
                )));
            } else if let Some((idcode, ir_len)) = bypassed.iter().find(|(i, _)| *i == idcode) {
                tap_infos.push(TapInfo::AlwaysBypass(*idcode, *ir_len));
            } else {
                return Err(format!("BSDL or BYPASS file of {idcode:08x} not found"));
            };
        }

        self.tap_infos = tap_infos;
        Ok(())
    }

    /// Adds `TapInfo` manually for the next TAP.
    /// Call this for `builder.get_id_codes().len()` times.
    pub fn add_tap_info(&mut self, info: TapInfo) -> Result<(), String> {
        let idcodes = self.get_id_codes()?;
        if self.tap_infos.len() >= idcodes.len() {
            return Err("already having enough `TapInfo`".into());
        }
        if !info.matches_idcode(idcodes[self.tap_infos.len()]) {
            return Err("IDCODE mismatch".into());
        }
        self.tap_infos.push(info);
        Ok(())
    }

    /// `true` indicates that output and output enable values from
    /// the internal logic be kept when switching to EXTEST mode;
    ///
    /// `false` indicates that safe values defined by BSDL should be
    /// loaded when switching to EXTEST mode.
    ///
    /// `false` by default.
    pub fn extest_keep_present(&mut self, new_val: bool) {
        self.extest_keep_present = new_val;
    }

    /// Builds the [PinScanner] if there are enough [TapInfo]s.
    pub fn build(mut self) -> Result<PinScanner<T>, String> {
        if self.tap_infos.len() != self.get_id_codes()?.len() {
            return Err("not having enough `TapInfo`".into());
        }

        // verify IR length
        let ir_len = self.sm.measure_ir_len()?;
        let info_ir_len: usize = self.tap_infos.iter().map(|info| info.ir_len()).sum();
        if info_ir_len != ir_len {
            return Err(format!(
                "IR length mismatch, current: {}, from given info: {}",
                ir_len, info_ir_len,
            ));
        }

        PinScanner::build(self.sm, self.tap_infos, ir_len, self.extest_keep_present)
    }

    /// Gets IDCODEs of all TAPs on the chain, from TDI to TDO.
    pub fn get_id_codes(&mut self) -> Result<Vec<u32>, String> {
        if let Some(idcodes) = self.idcodes.get() {
            return Ok(idcodes.clone());
        }

        let dev_cnt = self.count_devices()?;

        self.sm.mode_reset()?; // test logic reset
        let dr_len = self.sm.measure_dr_len()?;
        if dr_len != dev_cnt * 32 {
            return Err(format!(
                "Found device without IDCODE register, dr_len: {dr_len}"
            ));
        }

        self.sm.mode_reset()?; // test logic reset

        self.sm.switch_mode(JtagState::CaptureDR)?;
        // reads IDCODEs starting from the last device on the chain
        let mut raw_idcodes = BitVec::repeat(false, 32 * dev_cnt);
        self.sm.read_bits(Data, &mut raw_idcodes)?;
        self.sm.mode_reset()?; // test logic reset

        let mut idcodes = Vec::new();
        for raw_idcode in raw_idcodes.as_raw_slice().chunks(4) {
            let idcode = (raw_idcode[0] as u32)
                | ((raw_idcode[1] as u32) << 8)
                | ((raw_idcode[2] as u32) << 16)
                | ((raw_idcode[3] as u32) << 24);
            idcodes.push(idcode);
        }
        idcodes.reverse();

        let _ = self.idcodes.set(idcodes.clone());
        Ok(idcodes)
    }

    fn count_devices(&mut self) -> Result<usize, String> {
        self.sm.mode_reset()?; // test logic reset

        // make sure all TAPs are in bypass mode
        self.sm.write_repeated(Instruction, 1, 1024)?;
        self.sm.switch_mode(JtagState::UpdateIR)?;

        self.sm.measure_dr_len()
    }
}
