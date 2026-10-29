//! IO interface adapters. All of them are under the `protocol` feature gate.

pub mod pin;

#[cfg(feature = "protocol-spi")]
mod spi;
#[cfg(feature = "protocol-spi")]
pub use spi::*;
