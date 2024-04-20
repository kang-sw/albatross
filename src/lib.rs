#![allow(clippy::bool_comparison)]

pub mod bits;
pub mod bsp;
pub mod collision;
pub mod macros;
pub mod primitive;
pub mod var_grid;

// Reexport necessary items.
pub use slotmap::new_key_type as define_key;
pub use slotmap::Key;

#[cfg(feature = "fixed")]
pub extern crate fixed;

pub extern crate derive_more;
pub extern crate num;
pub extern crate static_assertions;
