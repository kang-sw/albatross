#![allow(clippy::bool_comparison)]

pub mod bitindex;
pub mod bsp;
pub mod collision;
pub mod macros;
pub mod packed;
pub mod primitive;

// Reexport necessary items.
pub use slotmap::new_key_type as define_key;
pub use slotmap::Key;

#[cfg(feature = "fixed")]
pub extern crate fixed;

pub extern crate derive_more;
pub extern crate num;
