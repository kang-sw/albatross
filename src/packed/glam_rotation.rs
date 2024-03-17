use super::*;

/// A normalized spherical coordinate to represent a directional vector.
// TODO: implement vector conversion
#[derive(Default, Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "reflect", derive(Reflect))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct PackedDirection<B, const PAN: usize>(B);

/// Packed scaled axis. length is normalized in range `0..PI`
// TODO: implement quaternion conversion
#[derive(Default, Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "reflect", derive(Reflect))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct PackedScaledAxis(PackedIVec3<u32, 10, 12, 10>);
