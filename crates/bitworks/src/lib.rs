/* ============================================================================================== */
/*                                            BIT INDEX                                           */
/* ============================================================================================== */
use std::ops::{Bound, RangeBounds};

const WORD_BITS: usize = 64;

/// Quickly store set of fixed number of indexes.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct FixedIntSet<const N_U64: usize> {
    bits: [u64; N_U64],
}

impl<const N: usize> Default for FixedIntSet<N> {
    fn default() -> Self {
        Self { bits: [0; N] }
    }
}

impl<const N: usize> FixedIntSet<N> {
    pub const fn capacity() -> usize {
        N * WORD_BITS
    }

    pub fn len(&self) -> usize {
        self.bits
            .into_iter()
            .fold(0, |acc, x| acc + x.count_ones() as usize)
    }

    pub const fn all() -> Self {
        Self {
            bits: [u64::MAX; N],
        }
    }

    pub const fn all_until(mut until: usize) -> Self {
        Self {
            bits: {
                let mut bits = [0; N];
                let mut cursor = 0;

                while cursor < N {
                    let fill = if until < WORD_BITS { until } else { 64 };
                    until = until.saturating_sub(WORD_BITS);

                    bits[cursor] = (1 << fill) - 1;
                    cursor += 1;
                }

                bits
            },
        }
    }

    pub const fn empty() -> Self {
        Self { bits: [0; N] }
    }

    pub fn is_empty(&self) -> bool {
        for i in 0..N {
            if self.bits[i] != 0 {
                return false;
            }
        }
        true
    }

    pub fn set(&mut self, index: usize) {
        let (i, j) = (index / WORD_BITS, index % 64);
        self.bits[i] |= 1 << j;
    }

    pub fn assign_range(&mut self, range: impl RangeBounds<usize>, value: bool) {
        let start = match range.start_bound() {
            Bound::Included(v) => *v,
            Bound::Excluded(v) => *v + 1,
            Bound::Unbounded => 0,
        };

        let end = match range.end_bound() {
            Bound::Included(v) => *v + 1,
            Bound::Excluded(v) => *v,
            Bound::Unbounded => WORD_BITS * N,
        };

        self._assign_n(start, end, value)
    }

    fn _assign_n(&mut self, mut start: usize, end: usize, value: bool) {
        debug_assert!(start <= end);

        let modulo = start % WORD_BITS;
        let unaligned_start = start - modulo;
        start = unaligned_start + WORD_BITS;

        // Apply unaligned access
        todo!();

        fn make_word(start: usize, end: usize) -> u64 {
            ((1 << (end - start)) - 1) << start
        }
    }

    pub fn unset(&mut self, index: usize) {
        let (i, j) = (index / WORD_BITS, index % 64);
        self.bits[i] &= !(1 << j);
    }

    pub fn get(&self, index: usize) -> bool {
        let (i, j) = (index / WORD_BITS, index % 64);
        (self.bits[i] & (1 << j)) != 0
    }

    pub fn clear(&mut self) {
        for i in 0..N {
            self.bits[i] = 0;
        }
    }

    pub fn is_disjoint(&self, other: &Self) -> bool {
        for i in 0..N {
            if self.bits[i] & other.bits[i] != 0 {
                return false;
            }
        }
        true
    }

    pub fn is_subset(&self, other: &Self) -> bool {
        for i in 0..N {
            if self.bits[i] & !other.bits[i] != 0 {
                return false;
            }
        }
        true
    }

    pub fn is_superset(&self, other: &Self) -> bool {
        for i in 0..N {
            if !self.bits[i] & other.bits[i] != 0 {
                return false;
            }
        }
        true
    }

    pub fn iter(&self) -> FixedIntSetIter<N> {
        FixedIntSetIter {
            bits: &self.bits,
            cache: self.bits[0],
            index: 0,
        }
    }
}

impl<const N: usize> std::iter::FromIterator<usize> for FixedIntSet<N> {
    fn from_iter<I: IntoIterator<Item = usize>>(iter: I) -> Self {
        let mut result = Self::default();
        for i in iter {
            result.set(i);
        }
        result
    }
}

impl<const N: usize> std::ops::Index<usize> for FixedIntSet<N> {
    type Output = bool;

    fn index(&self, index: usize) -> &Self::Output {
        if self.get(index) {
            &true
        } else {
            &false
        }
    }
}

impl<const N: usize> std::ops::BitAnd for FixedIntSet<N> {
    type Output = Self;

    fn bitand(self, rhs: Self) -> Self::Output {
        let mut result = Self::default();
        for i in 0..N {
            result.bits[i] = self.bits[i] & rhs.bits[i];
        }
        result
    }
}

impl<const N: usize> std::ops::BitOr for FixedIntSet<N> {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        let mut result = Self::default();
        for i in 0..N {
            result.bits[i] = self.bits[i] | rhs.bits[i];
        }
        result
    }
}

impl<const N: usize> std::ops::BitXor for FixedIntSet<N> {
    type Output = Self;

    fn bitxor(self, rhs: Self) -> Self::Output {
        let mut result = Self::default();
        for i in 0..N {
            result.bits[i] = self.bits[i] ^ rhs.bits[i];
        }
        result
    }
}

impl<const N: usize> std::ops::Not for FixedIntSet<N> {
    type Output = Self;

    fn not(self) -> Self::Output {
        let mut result = Self::default();
        for i in 0..N {
            result.bits[i] = !self.bits[i];
        }
        result
    }
}

impl<const N: usize> std::ops::BitAndAssign for FixedIntSet<N> {
    fn bitand_assign(&mut self, rhs: Self) {
        for i in 0..N {
            self.bits[i] &= rhs.bits[i];
        }
    }
}

impl<const N: usize> std::ops::BitOrAssign for FixedIntSet<N> {
    fn bitor_assign(&mut self, rhs: Self) {
        for i in 0..N {
            self.bits[i] |= rhs.bits[i];
        }
    }
}

impl<const N: usize> std::ops::BitXorAssign for FixedIntSet<N> {
    fn bitxor_assign(&mut self, rhs: Self) {
        for i in 0..N {
            self.bits[i] ^= rhs.bits[i];
        }
    }
}

impl<const N: usize> std::fmt::Debug for FixedIntSet<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_list().entries(self.iter()).finish()
    }
}

pub struct FixedIntSetIter<'a, const N: usize> {
    bits: &'a [u64; N],
    cache: u64,
    index: usize,
}

impl<'a, const N: usize> Iterator for FixedIntSetIter<'a, N> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        while self.cache == 0 {
            self.index += 1;

            if self.index >= N {
                return None;
            }

            self.cache = self.bits[self.index];
        }

        let trailing_zeros = self.cache.trailing_zeros() as usize;
        self.cache &= self.cache - 1;

        Some(self.index * WORD_BITS + trailing_zeros)
    }
}

#[cfg(test)]
#[test]
fn __test_bits() {
    use tap::Tap;

    let mut bits = FixedIntSet::<3>::default();

    let trues = [0, 4, 5, 9, 11, 2, 52, 161, 32, 144, 33, 64, 98, 45];
    let sorted_trues = { trues }.tap_mut(|x| x.sort());

    assert!(trues.iter().all(|&i| !bits.get(i)));
    assert!(bits.iter().count() == 0);

    for &i in trues.iter() {
        bits.set(i);
    }

    assert!(trues.iter().all(|&i| bits.get(i)));
    assert!(bits.iter().count() == trues.len());
    assert!(bits.iter().zip(sorted_trues).all(|(a, b)| a == b));

    let mut other_bits = bits;
    other_bits.clear();

    for &i in trues.iter() {
        bits.unset(i);
    }

    assert!(other_bits == bits);
    assert!(bits == Default::default());
}

/* ============================================================================================== */
/*                                         SCOPED INTEGER                                         */
/* ============================================================================================== */

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub struct ScopedInt<const MIN: i32, const MAX: i32>(i32);

impl<const MIN: i32, const MAX: i32> ScopedInt<MIN, MAX> {
    pub const MIN: i32 = MIN;
    pub const MAX: i32 = MAX;

    pub const fn new(value: i32) -> Self {
        assert!(value >= MIN && value <= MAX);
        Self(value)
    }

    pub fn raw(&self) -> i32 {
        self.0
    }
}

impl<const MIN: i32, const MAX: i32> std::ops::Deref for ScopedInt<MIN, MAX> {
    type Target = i32;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const MIN: i32, const MAX: i32> From<i32> for ScopedInt<MIN, MAX> {
    fn from(value: i32) -> Self {
        Self::new(value)
    }
}

impl<const MIN: i32, const MAX: i32> From<ScopedInt<MIN, MAX>> for i32 {
    fn from(value: ScopedInt<MIN, MAX>) -> Self {
        value.0
    }
}

impl<const MIN: i32, const MAX: i32> From<u32> for ScopedInt<MIN, MAX> {
    fn from(value: u32) -> Self {
        Self::new(value as _)
    }
}

impl<const MIN: i32, const MAX: i32> From<ScopedInt<MIN, MAX>> for u32 {
    fn from(value: ScopedInt<MIN, MAX>) -> Self {
        value.0 as _
    }
}

impl<const MIN: i32, const MAX: i32> From<u64> for ScopedInt<MIN, MAX> {
    fn from(value: u64) -> Self {
        Self::new(value as _)
    }
}

impl<const MIN: i32, const MAX: i32> From<ScopedInt<MIN, MAX>> for u64 {
    fn from(value: ScopedInt<MIN, MAX>) -> Self {
        value.0 as _
    }
}
