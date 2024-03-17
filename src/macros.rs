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
        #[derive(Debug, Clone, Copy, PartialEq)]
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
{
    #[inline]
    #[doc(hidden)]
    pub fn __get(&self) -> B
    where
        T: 'static + Copy,
    {
        // SAEFTY: `self` is a valid reference to `B`
        let base = unsafe { &*(self as *const Self as *const B) };
        (*base >> S) & Self::mask()
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

    fn mask() -> B {
        B::one() << ((E - S) - 1)
    }
}

/* --------------------------------------- Signed Integers -------------------------------------- */

#[doc(hidden)]
pub trait SignedInteger {
    const SIGNED: bool = false;

    fn leading_zeros(self) -> u32;
    fn bit_flip(self) -> Self;
    fn clamp_bits(self, bits: usize) -> Self;
    fn to_normal(self, bits: usize) -> f32;
    fn from_normal(normal: f32, bits: usize) -> Self;
    fn to_usize(self) -> usize;
}

macro_rules! signed {

    (true, $($t:ty)*) => {
        $(
            impl SignedInteger for $t {
                signed!(base, true);

                fn clamp_bits(self, bits: usize) -> Self {
                    self.clamp(
                        -(1 << (bits - 1)),
                        (1 << (bits - 1)) - 1
                    )
                }

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
                    }.round () as Self
                }

            }
        )*
    };

    (false, $($t:ty)*) => {
        $(
            impl SignedInteger for $t {
                signed!(base, false);

                fn clamp_bits(self, bits: usize) -> Self {
                    self.min((1 << bits) - 1)
                }

                fn to_normal(self, bits: usize) -> f32 {
                    self as f32 / ((1 << bits) - 1) as f32
                }

                fn from_normal(normal: f32, bits: usize) -> Self {
                    (normal * ((1 << bits) - 1) as f32).round() as Self
                }
            }
        )*
    };

    (base, $value:literal) => {
        const SIGNED: bool = $value;

        fn leading_zeros(self) -> u32 {
            self.leading_zeros()
        }

        fn bit_flip(self) -> Self {
            !self
        }

        fn to_usize(self) -> usize {
            self as usize
        }
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
                $elem_vis:vis $elem:ident: $elem_ty:ident($elem_start:literal $(..$elem_end:literal)?),
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

            pub struct ProxyType {
                $(
                    $elem_vis $elem: _m!(@proxy $base, $elem_ty, $elem_start $($elem_end)?),
                )*
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

            impl $name {
                #[inline]
                pub fn new(
                    $(
                        $elem: $elem_ty,
                    )*
                ) -> Self {
                    let mut base = Self::default();
                    $(
                        base.$elem.set($elem);
                    )*
                    base
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
            self.$elem.__get() as _
        }
    };

    /* ·························································································· */

    () => {

    }
}

#[test]
fn test_custom_bit_vector() {
    define_packed_vector!(
        struct MyVec<u32> {
            x: i32(2..4),
            y: i32(1..3),
            b: bool(0),
        }
    );

    let mut g = MyVec(0);
    assert_eq!(&g.x as *const _ as usize, &g as *const _ as usize);
    assert_eq!(&g.y as *const _ as usize, &g as *const _ as usize);
    assert_eq!(&g.b as *const _ as usize, &g as *const _ as usize);

    g.b.set(true);
    assert_eq!(g.0, 1);
    assert!(g.b());
}
