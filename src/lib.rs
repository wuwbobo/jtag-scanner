//! JTAG boundary scanning module.
//!
//! **WARNING**: This tool may damage your hardware in `EXTEST` mode, especially when the target is not a development board: to reduce the risk, do not enable the output of any pin with unknown functionality for a long time (the duration should be as short as possible).
//!
//! ## Usage
//!
//! - Check for [compatible debug adapters](jtag_taps::cable), connect the adapter to the target, connect the adapter to the PC and power on the target;
//! - Construct a [`jtag_taps::cable::Cable`] instance and build the [`jtag_taps::statemachine::JtagSM`];
//! - Create a [`PinScannerBuilder`], passing the `JtagSM` instance to it;
//! - Check the result of [`PinScannerBuilder::get_id_codes`] to make sure the TAP chain is working;
//! - Call [`PinScannerBuilder::scan_tap_info`] with a directory path for scanning BSDL files;
//! - Call [`PinScannerBuilder::build`] to build the [`PinScanner`];
//! - Enumerate in [`PinScanner::tap_infos`] to get the index of the TAP that you want to control;
//! - Get [`PinId`] values via [`PinScanner::get_pin_id`] (providing the port name as the argument), or iterate through [`PinScanner::pin_ids_for_tap`].
//! - Set the TAP to `SAMPLE` or `EXTEST` mode by [`PinScanner::set_tap_state`].
//! - Call [`PinScanner::get_pin_state`] or [`PinScanner::set_pin_state`], and then [`PinScanner::update`] or [`PinScanner::write_update`].
//! - Check [`PinScanner::batch`] if higher transmission efficiency is required.

#![deny(unsafe_code)]

pub mod jtag;

mod builder;
mod scanner;
mod types;

pub use {builder::*, scanner::*, types::*};
