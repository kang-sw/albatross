#![allow(clippy::bool_comparison)]

pub mod bsp;
pub mod primitive;

pub use primitive::ControlIntensity;

#[cold]
#[inline(always)]
fn cold_path() {}
