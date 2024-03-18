/* ============================================================================================== */
/*                                       CUSTOM VECTOR TYPE                                       */
/* ============================================================================================== */

use num::cast::AsPrimitive;

/// Defines custom vector type, with basic vector arithmetics & conversions.
#[macro_export]
macro_rules! define_custom_vector {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident {
            $(
                $(#[$elem_meta:meta])*
                $elem:ident: $elem_ty:ty,
            )*
        }

        $($rest:tt)*
    ) => {
        $(#[$meta])*
        #[derive(Default, Debug, Clone, Copy, PartialEq)]
        #[derive(
            $crate::derive_more::Add,
            $crate::derive_more::Sub,
            $crate::derive_more::AddAssign,
            $crate::derive_more::SubAssign,
            $crate::derive_more::MulAssign,
            $crate::derive_more::DivAssign
        )]
        $vis struct $name {
            $(
                $(#[$elem_meta])*
                pub $elem: $elem_ty,
            )*
        }

        #[allow(unused)]
        const _: () = {
            use $crate::num::traits::*;
            use $crate::define_custom_vector as _m;

            impl $name {
                #[inline]
                pub const fn new($($elem: $elem_ty),*) -> Self {
                    Self {
                        $($elem),*
                    }
                }

                #[inline]
                pub const fn to_tuple(self) -> ($($elem_ty,)*) {
                    ($(
                        self.$elem,
                    )*)
                }

                #[inline]
                pub fn to_array<T>(self) -> [
                    T; _m!(@count $($elem )*)
                ] where
                T: Num + Copy + 'static,
                $($elem_ty: AsPrimitive<T>,)*
                {
                    [
                        $(
                            AsPrimitive::as_(self.$elem),
                        )*
                    ]
                }
            }

            impl<T> From<[T;_m!(@count $($elem )*)]> for $name
            where
                $(T: AsPrimitive<$elem_ty>,)*
            {
                #[inline]
                fn from([$( $elem ),*]: [T; _m!(@count $($elem )*)]) -> Self {
                    Self {
                        $($elem: $elem.as_(),)*
                    }
                }
            }

            impl From<( $($elem_ty, )* )> for $name {
                #[inline]
                fn from(($($elem,)*): ( $($elem_ty, )* )) -> Self {
                    Self {
                        $($elem,)*
                    }
                }
            }

            impl<__R> std::ops::Mul<__R> for $name
                where __R: Copy $(+AsPrimitive<$elem_ty>)*
            {
                type Output = Self;

                #[inline]
                fn mul(self, rhs: __R) -> Self::Output
                {
                    Self {
                        $($elem: self.$elem * AsPrimitive::<$elem_ty>::as_(rhs),)*
                    }
                }
            }

            impl<__R> std::ops::Div<__R> for $name
                where __R: Copy $(+AsPrimitive<$elem_ty>)*
            {
                type Output = Self;

                #[inline]
                fn div(self, rhs: __R) -> Self::Output
                {
                    Self {
                        $($elem: self.$elem / AsPrimitive::<$elem_ty>::as_(rhs),)*
                    }
                }
            }
        };

        $crate::define_custom_vector!( $($rest)* );
    };

    (@count $_0:ident) => { 1 };
    (@count $_0:ident $_1:ident) => { 2 };
    (@count $_0:ident $_1:ident $_2:ident) => { 3 };
    (@count $_0:ident $_1:ident $_2:ident $_3:ident) => { 4 };
    (@count $_0:ident $_1:ident $_2:ident $_3:ident $($tail:ident)*) => { 4 + _m!(@count $($tail)*) };

    () => {};
}

#[test]
fn test_custom_vector() {
    define_custom_vector!(
        struct MyVec {
            x: i32,
            y: f32,
            z: f64,
            w: i32,
        }

        struct OtherVec {
            _0: i32,
            _1: f32,
            _2: f64,
            _3: i32,
            _4: i32,
            _5: i32,
        }
    );

    let v = MyVec::new(1, 2.0, 3.0, 4);
    assert_eq!(v.to_tuple(), (1, 2.0, 3.0, 4));
    assert_eq!(v.to_array::<i32>(), [1, 2, 3, 4]);
    assert_eq!(MyVec::from([1, 2, 3, 4]), v);
    assert_eq!(v + v, MyVec::new(2, 4.0, 6.0, 8));
    assert_eq!(v - v, MyVec::new(0, 0.0, 0.0, 0));
    assert_eq!(v * 2, MyVec::new(2, 4.0, 6.0, 8));
    assert_eq!(v / 2, MyVec::new(0, 1.0, 1.5, 2));
}

/* ============================================================================================== */
/*                                    CUSTOM PACKED VECTOR TYPE                                   */
/* ============================================================================================== */

pub struct BitAccessProxy<B, T, const S: usize, const E: usize> {
    _marker: std::marker::PhantomData<(B, T)>,
}

impl<B, T, const S: usize, const E: usize> BitAccessProxy<B, T, S, E>
where
    B: 'static + num::PrimInt + num::Unsigned,
    T: 'static + Copy,
{
    #[inline]
    #[doc(hidden)]
    pub fn __get(&self) -> B {
        // SAEFTY: `self` is a valid reference to `B`
        let base = unsafe { &*(self as *const Self as *const B) };
        (*base >> S) & Self::mask()
    }

    #[inline]
    pub fn get(&self) -> T
    where
        B: AsPrimitive<T>,
        T: IntegerExt,
    {
        if T::SIGNED {
            let sign_bit_pos = (E - S) as u32;
            let value = self.__get();
            let leading_zeros = value.leading_zeros();
            let msb_pos = B::max_value().count_ones() - leading_zeros;

            if msb_pos != sign_bit_pos {
                value
            } else {
                value | (B::max_value() << sign_bit_pos as usize)
            }
        } else {
            self.__get()
        }
        .as_()
    }

    #[inline]
    pub fn normal(&self) -> f32
    where
        B: AsPrimitive<T>,
        T: IntegerExt,
    {
        self.get().to_normal(E - S)
    }

    #[inline]
    pub fn set_normal(&mut self, value: f32)
    where
        B: AsPrimitive<T>,
        T: IntegerExt + AsPrimitive<B>,
    {
        debug_assert!((0.0..=1.0).contains(&value));
        self.set(T::from_normal(value, E - S));
    }

    #[inline]
    pub fn to<D>(&self) -> D
    where
        B: AsPrimitive<T>,
        T: IntegerExt + AsPrimitive<D>,
        D: 'static + Copy,
    {
        self.get().as_()
    }

    #[inline]
    pub fn set(&mut self, value: T)
    where
        T: AsPrimitive<B>,
    {
        let base = unsafe { &mut *(self as *mut Self as *mut B) };
        let value: B = value.as_() & Self::mask();
        *base = (*base & !(Self::mask() << S)) | (value << S);
    }

    #[inline]
    pub fn set_clamped(&mut self, value: T)
    where
        T: IntegerExt + AsPrimitive<B>,
    {
        self.set(value.clamped(E - S));
    }

    #[inline]
    pub fn mutate(&mut self, f: impl FnOnce(T) -> T)
    where
        T: AsPrimitive<B> + IntegerExt,
        B: AsPrimitive<T>,
    {
        self.set(f(self.get()));
    }

    #[inline]
    fn mask() -> B {
        (B::one() << (E - S)) - B::one()
    }
}

macro_rules! define_opr {
    (
        $trait_name:ident,
        $constraint:tt,
        $($body:tt)*
    ) => {
        impl<B, T, const S: usize, const E: usize> std::ops::$trait_name<T>
            for BitAccessProxy<B, T, S, E>
        where
            B: 'static + num::PrimInt + num::Unsigned + AsPrimitive<T>,
            T: 'static + Copy + IntegerExt + AsPrimitive<B> + std::ops::$constraint<Output = T>,
        {
            $($body)*
        }
    };
}

define_opr!(
    AddAssign,
    Add,
    fn add_assign(&mut self, rhs: T) {
        self.mutate(|x| x + rhs);
    }
);

define_opr!(
    SubAssign,
    Sub,
    fn sub_assign(&mut self, rhs: T) {
        self.mutate(|x| x - rhs);
    }
);

define_opr!(
    MulAssign,
    Mul,
    fn mul_assign(&mut self, rhs: T) {
        self.mutate(|x| x * rhs);
    }
);

define_opr!(
    DivAssign,
    Div,
    fn div_assign(&mut self, rhs: T) {
        self.mutate(|x| x / rhs);
    }
);

/* ------------------------------------- Bit Proxy With Type ------------------------------------ */

pub struct BitAccessProxyAs<B, T, As, const S: usize, const E: usize> {
    _marker: std::marker::PhantomData<(B, T, As)>,
}

impl<B, T, As, const S: usize, const E: usize> std::ops::Deref
    for BitAccessProxyAs<B, T, As, S, E>
{
    type Target = BitAccessProxy<B, T, S, E>;

    fn deref(&self) -> &Self::Target {
        // SAFETY: `self` is a valid reference to `BitAccessProxy`
        unsafe { &*(self as *const Self as *const BitAccessProxy<B, T, S, E>) }
    }
}

impl<B, T, As, const S: usize, const E: usize> std::ops::DerefMut
    for BitAccessProxyAs<B, T, As, S, E>
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: `self` is a valid reference to `BitAccessProxy`
        unsafe { &mut *(self as *mut Self as *mut BitAccessProxy<B, T, S, E>) }
    }
}

// TODO: set(As), get() -> As

/* --------------------------------------- Signed Integers -------------------------------------- */

#[doc(hidden)]
pub trait IntegerExt {
    const SIGNED: bool = false;

    fn to_normal(self, bits: usize) -> f32;
    fn from_normal(normal: f32, bits: usize) -> Self;
    fn clamped(self, bits: usize) -> Self;
}

macro_rules! signed {

    (true, $($t:ty)*) => {
        $(
            impl IntegerExt for $t {
                signed!(base, true);

                fn to_normal(self, bits: usize) -> f32 {
                    2.0 * if self >= 0 {
                        self as f32 / ((1 << bits) - 1) as f32
                    } else {
                        self as f32 / (1 << bits) as f32
                    }
                }

                fn from_normal(mut normal: f32, bits: usize) -> Self {
                    normal *= 0.5;

                    if normal >= 0.0 {
                        (normal * ((1 << bits) - 1) as f32)
                    } else {
                        (normal * (1 << bits) as f32)
                    }.round() as Self
                }

                fn clamped(self, bits: usize) -> Self {
                    self.clamp(-(1 << (bits - 1)), (1 << (bits - 1)) - 1)
                }
            }
        )*
    };

    (false, $($t:ty)*) => {
        $(
            impl IntegerExt for $t {
                signed!(base, false);

                fn to_normal(self, bits: usize) -> f32 {
                    self as f32 / ((1 << bits) - 1) as f32
                }

                fn from_normal(normal: f32, bits: usize) -> Self {
                    (normal * ((1 << bits) - 1) as f32).round() as Self
                }

                fn clamped(self, bits: usize) -> Self {
                    self.clamp(0, (1 << bits) - 1)
                }
            }
        )*
    };

    (base, $value:literal) => {
        const SIGNED: bool = $value;
    };
}

signed!(true, i8 i16 i32 i64 isize i128);
signed!(false, u8 u16 u32 u64 usize u128);

/* ----------------------------------------- Macro Body ----------------------------------------- */

#[macro_export]
macro_rules! define_packed_vector {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident<$base:ty> {
            $(
                $(#[$elem_meta:meta])*
                // element identifier + representation type
                $elem_vis:vis $elem:ident: $elem_ty:ident
                    // start bit position .. to end bit position
                    @ $elem_start:literal$(..$elem_end:literal)?
                    // optional alternative representation type, From + Into required
                    $(as $elem_as_ty:path)?,
            )*
        }

        $($rest:tt)*
    ) => {
        $(#[$meta])*
        #[derive(Default, Clone, Copy, PartialEq, Eq)]
        $vis struct $name($base);

        #[allow(unused)]
        const _: () = {
            use $crate::define_packed_vector as _m;
            use $crate::num;
            use $crate::num::traits::*;
            use $crate::macros::BitAccessProxy;
            use $crate::static_assertions::*;

            $(
                $(
                    const_assert!(<$base>::BITS >= $elem_end);
                    const_assert!($elem_end > $elem_start);
                )?
                const_assert!(<$base>::BITS > $elem_start);
            )*

            #[repr(C)]
            $vis struct ProxyType {
                $(
                    $elem_vis $elem: _m!(@proxy $base, $elem_ty, $elem_start $($elem_end)?),
                )*

                // Unused body, to make `taking out` the proxy type safe.
                _body: $base,
            }

            impl std::ops::Deref for $name {
                type Target = ProxyType;

                #[inline]
                fn deref(&self) -> &Self::Target {
                    unsafe { &*(self as *const $name as *const ProxyType) }
                }
            }

            impl std::ops::DerefMut for $name {
                #[inline]
                fn deref_mut(&mut self) -> &mut Self::Target {
                    unsafe { &mut *(self as *mut $name as *mut ProxyType) }
                }
            }

            impl std::fmt::Debug for $name
            where $( $elem_ty: std::fmt::Debug, )*
            {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    f.debug_struct(stringify!($name))
                    $(
                        .field(stringify!($elem), &self.$elem())
                    )*
                    .finish()
                }
            }

            impl $name {
                pub const ZERO: Self = Self(0);

                #[inline]
                pub fn to_tuple(self) -> ($($elem_ty,)*) {
                    (
                        $(
                            self.$elem(),
                        )*
                    )
                }

                #[inline]
                pub fn to_array<T>(self) -> [T; _m!(@count $($elem )*)]
                where
                    T: Num + Copy + 'static,
                    $($elem_ty: AsPrimitive<T>,)*
                {
                    [
                        $(
                            AsPrimitive::as_(self.$elem()),
                        )*
                    ]
                }

                #[inline]
                pub fn from_tuple(
                    ($( $elem, )* ): ($($elem_ty,)*)
                ) -> Self {
                    let mut base = Self::default();
                    $(
                        base.$elem.set($elem);
                    )*
                    base
                }

                #[inline]
                pub fn from_array<T>([
                    $($elem,)*
                ]: [T; _m!(@count $($elem )*)]) -> Self
                where
                    T: $(AsPrimitive<$elem_ty> +)* Copy,
                    $($elem_ty: AsPrimitive<T>,)*
                {
                    Self::from_tuple(($($elem.as_(),)*))
                }

                $(
                    #[inline]
                    _m!(@getter $elem_ty, $elem);
                )*
            }
        };

        $crate::define_packed_vector!($($rest)*);
    };

    /* ------------------------------------- Proxy Retrieval ------------------------------------ */

    (
        @proxy $base:ty, bool, $elem_start:literal
    ) => {
        BitAccessProxy<$base, bool, $elem_start, {$elem_start + 1}>
    };

    (
        @proxy $base:ty, $elem_ty:ty, $elem_start:literal $elem_end:literal
    ) => {
        BitAccessProxy<$base, $elem_ty, $elem_start, $elem_end>
    };

    /* ---------------------------------- Getter Function Body ---------------------------------- */

    (@getter bool, $elem:ident) => {
        pub fn $elem(&self) -> bool {
            self.$elem.__get() != 0
        }
    };

    (@getter $elem_ty:ty, $elem:ident) => {
        pub fn $elem(&self) -> $elem_ty {
            self.$elem.get()
        }
    };

    /* ·························································································· */

    (@count $_0:ident) => { 1 };
    (@count $_0:ident $_1:ident) => { 2 };
    (@count $_0:ident $_1:ident $_2:ident) => { 3 };
    (@count $_0:ident $_1:ident $_2:ident $_3:ident) => { 4 };
    (@count $_0:ident $_1:ident $_2:ident $_3:ident $($tail:ident)*) => { 4 + _m!(@count $($tail)*) };

    () => {

    }
}

#[test]
fn test_custom_bit_vector() {
    define_packed_vector!(
        struct MyVec<u32> {
            x: i32@0..3,
            y: i32@3..5,
            z: u32@5..7,
            w: u32@7..10,
            b: bool@0,
        }
    );

    let mut g = MyVec::ZERO;
    assert_eq!(&g.x as *const _ as usize, &g as *const _ as usize);
    assert_eq!(&g.y as *const _ as usize, &g as *const _ as usize);
    assert_eq!(&g.b as *const _ as usize, &g as *const _ as usize);

    g.b.set(true);
    assert_eq!(g.0, 1);
    assert!(g.b());

    assert_eq!(g.x(), 1);
    assert_eq!(g.y(), 0);

    g.x.set(2);
    assert!(!g.b());
    assert_eq!(g.0, 2);

    g.x.set(-1);
    assert_eq!(g.x(), -1);
    assert_eq!(g.0, 0b111);

    g.y.set(1);
    assert_eq!(g.x(), -1);
    assert_eq!(g.y(), 1);
    assert_eq!(g.0, 0b1111);

    g.z.set(1);
    assert_eq!(g.z(), 1);
    g.z += 1;
    assert_eq!(g.z(), 2);
    g.z += 1;
    assert_eq!(g.z(), 3);
    g.z += 1;
    assert_eq!(g.z(), 0);

    g.z.set(2);
    g.z /= 2;
    assert_eq!(g.z(), 1);
    g.z *= 3;
    assert_eq!(g.z(), 3);

    g.z.mutate(|x| x / 3);
    assert_eq!(g.z(), 1);

    dbg!(g);

    // TODO: migrate test cases
}

// TODO: migrate test cases for packed vector
#[cfg(any())]
#[cfg(test)]
mod tests {
    use super::PackedIVec3;

    #[test]
    fn test_packed_ivec3_unsigned() {
        let g = PackedIVec3::<u8, 2, 2, 4>::new(1, 2, 3);
        assert_eq!(g.to_arr(), [1, 2, 3]);

        let k = PackedIVec3::<u16, 4, 4, 8>::new(2, 3, 4);
        assert_eq!(k.to_arr(), [2, 3, 4]);

        let k = PackedIVec3::<u16, 4, 4, 8>::new(15, 15, 15);
        assert_eq!(k.to_arr(), [15, 15, 15]);

        let k = PackedIVec3::<u16, 0, 4, 12>::new(0, 15, 4095);
        assert_eq!(k.to_arr(), [0, 15, 4095]);

        // Test case for u32 with different bit widths
        let m = PackedIVec3::<u32, 10, 10, 12>::new(512, 512, 1024);
        assert_eq!(m.to_arr(), [512, 512, 1024]);

        // Test case with maximum values for given bit widths
        let n = PackedIVec3::<u32, 5, 5, 6>::new(31, 31, 63);
        assert_eq!(n.to_arr(), [31, 31, 63]);

        // Test case with minimum values (assuming signed integers are not allowed)
        let o = PackedIVec3::<u8, 3, 3, 2>::new(0, 0, 0);
        assert_eq!(o.to_arr(), [0, 0, 0]);
    }

    #[test]
    fn test_packed_ivec3_signed() {
        // Test case with mixed positive and negative values
        let a = PackedIVec3::<i16, 5, 6, 5>::new(-16, 31, -1);
        assert_eq!(a.to_arr(), [-16, 31, -1]);

        // Test case with all negative values
        let b = PackedIVec3::<i8, 3, 3, 2>::new(-4, -3, -2);
        assert_eq!(b.to_arr(), [-4, -3, -2]);

        // Test case to verify handling of minimum and maximum values for signed integers
        let c = PackedIVec3::<i32, 16, 1, 15>::new(i16::MIN as i32, 0, (i16::MAX >> 1) as i32);
        assert_eq!(c.to_arr(), [i16::MIN as i32, 0, (i16::MAX >> 1) as i32]);

        // Test case to check behavior with values just beyond positive and negative limits
        let d = PackedIVec3::<i16, 4, 4, 8>::new(-9, 8, 128); // Beyond the range of 4-bit signed (-8 to 7) and 8-bit signed (-128 to 127)
        assert_eq!(d.to_arr(), [-8, 7, 127]); // Assuming clipping or wrapping to the closest valid value

        // Test case with zero values to verify no sign bit issues
        let e = PackedIVec3::<i32, 10, 10, 12>::new(0, 0, 0);
        assert_eq!(e.to_arr(), [0, 0, 0]);
    }

    #[test]
    fn test_normals_conversion() {
        // Round-Trip Conversion Test for Unsigned Type
        let original_unsigned = PackedIVec3::<u16, 5, 5, 6>::new(31, 0, 63);
        let normalized_vec = original_unsigned.to_normals();
        let converted_back = PackedIVec3::from_normals(normalized_vec);
        assert_eq!(original_unsigned, converted_back);

        // Round-Trip Conversion Test for Signed Type
        let original_signed = PackedIVec3::<i16, 5, 5, 6>::new(-16, 0, 31);
        let normalized_vec_signed = original_signed.to_normals();
        let converted_back_signed = PackedIVec3::from_normals(normalized_vec_signed);
        assert_eq!(original_signed, converted_back_signed);
    }

    #[test]
    fn test_normalization_accuracy() {
        // Unsigned normalization
        let max_value_unsigned = PackedIVec3::<u16, 5, 5, 6>::new(31, 31, 63);
        let normalized_unsigned = max_value_unsigned.to_normals();
        assert!((normalized_unsigned[0] - 1.0).abs() < 1.0 / 31.0);
        assert!((normalized_unsigned[1] - 1.0).abs() < 1.0 / 31.0);
        assert!((normalized_unsigned[2] - 1.0).abs() < 1.0 / 63.0);

        // Signed normalization
        let max_value_signed = PackedIVec3::<i16, 5, 5, 6>::new(-16, 0, 31);
        let normalized_signed = max_value_signed.to_normals();
        assert!((normalized_signed[0] + 1.0).abs() < 1.0 / 16.0); // Mapping -16 to -1.0
        assert!(normalized_signed[1].abs() < 1.0 / 16.0); // Mapping 0 to 0.0
        assert!((normalized_signed[2] - 1.0).abs() < 1.0 / 31.0); // Mapping 31 to 1.0
    }

    #[test]
    fn test_denormalization_accuracy() {
        // Test denormalization from normals back to packed integers
        let normal_vec = [1.0, 0.5, -1.0];
        let packed = PackedIVec3::<i16, 5, 5, 6>::from_normals(normal_vec);
        // Assuming B::from_normal correctly maps the floating-point range back to integers
        assert_eq!(packed.to_arr(), [15, 8, -32]); // Example expected values, adjust based on actual mapping logic
    }
}
