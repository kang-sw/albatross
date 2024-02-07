#![allow(clippy::bool_comparison)]

pub mod bitindex;
pub mod bsp;
pub mod primitive;

pub use primitive::ControlIntensity;

// Reexport necessary items.
pub use slotmap::new_key_type as define_key;
pub use slotmap::Key;
