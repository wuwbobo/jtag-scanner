//! JTAG boundary scanning module.
//!
//! **WARNING**: This tool may damage your hardware in `EXTEST` mode, especially when the target
//! is not a development board: to reduce the risk, do not enable the output of any pin with
//! unknown functionality for a long time (the duration should be as short as possible).
//!
//! ## Usage
//!
//! ### Building the `PinScanner`
//! - Check for [compatible debug adapters](jtag_taps::cable), connect the adapter to the target,
//!   connect the adapter to the PC and power on the target;
//! - Construct a [Cable](jtag_taps::cable::Cable) instance and build the
//!   [JtagSM](jtag_taps::statemachine::JtagSM);
//! - Create a [`PinScannerBuilder`] with the `JtagSM` instance, and (optionally) check the result of
//!   [get_id_codes](PinScannerBuilder::get_id_codes) to make sure the TAP chain is working;
//! - With the builder, call [scan_tap_info](PinScannerBuilder::scan_tap_info) with a directory path
//!   for scanning BSDL files, then call [build](PinScannerBuilder::build) to build the [`PinScanner`].
//!
//! ### Using the `PinScanner`
//! - Enumerate in [tap_infos](PinScanner::tap_infos) to get the index of the TAP that you want to control.
//! - Get [`PinId`] values via [get_pin_id](PinScanner::get_pin_id), providing the port name as the argument;
//!   or iterate through [pin_ids_for_tap](PinScanner::pin_ids_for_tap).
//! - Set the TAP to `SAMPLE` or `EXTEST` mode by [set_tap_state](PinScanner::set_tap_state).
//! - Call [get_pin_state](PinScanner::get_pin_state) or [set_pin_state](PinScanner::set_pin_state), and then
//!   [update](PinScanner::update) or [write_update](PinScanner::write_update).
//! - Check [`PinScanner::batch`] if higher transmission efficiency is required.
//!
//! ### Creating bitbang IO interface adapters
//! Some adapters (e.g. SPI, I2C, NOR and NAND flash) may be added in the future. These adapters should
//! depend on [`PinGroup`] instead of using [`PinScanner`] directly.

#![deny(unsafe_code)]

pub mod jtag;

mod builder;
mod group;
mod scanner;
mod types;

pub use {builder::*, group::*, scanner::*, types::*};
