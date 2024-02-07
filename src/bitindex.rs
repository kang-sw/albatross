/// Quickly store set of fixed number of indexes.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct BitIndexSet<const N_U64: usize> {
    bits: [u64; N_U64],
}

impl<const N: usize> Default for BitIndexSet<N> {
    fn default() -> Self {
        Self { bits: [0; N] }
    }
}

impl<const N: usize> BitIndexSet<N> {
    pub const fn capacity() -> usize {
        N * 64
    }

    pub fn len(&self) -> usize {
        self.iter().count()
    }

    pub const fn all() -> Self {
        Self {
            bits: [u64::MAX; N],
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
        let (i, j) = (index / 64, index % 64);
        self.bits[i] |= 1 << j;
    }

    pub fn unset(&mut self, index: usize) {
        let (i, j) = (index / 64, index % 64);
        self.bits[i] &= !(1 << j);
    }

    pub fn get(&self, index: usize) -> bool {
        let (i, j) = (index / 64, index % 64);
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

    pub fn iter(&self) -> BitIndexSetIter<N> {
        BitIndexSetIter {
            bits: &self.bits,
            cache: self.bits[0],
            index: 0,
        }
    }
}

impl<const N: usize> std::iter::FromIterator<usize> for BitIndexSet<N> {
    fn from_iter<I: IntoIterator<Item = usize>>(iter: I) -> Self {
        let mut result = Self::default();
        for i in iter {
            result.set(i);
        }
        result
    }
}

impl<const N: usize> std::ops::Index<usize> for BitIndexSet<N> {
    type Output = bool;

    fn index(&self, index: usize) -> &Self::Output {
        if self.get(index) {
            &true
        } else {
            &false
        }
    }
}

impl<const N: usize> std::ops::BitAnd for BitIndexSet<N> {
    type Output = Self;

    fn bitand(self, rhs: Self) -> Self::Output {
        let mut result = Self::default();
        for i in 0..N {
            result.bits[i] = self.bits[i] & rhs.bits[i];
        }
        result
    }
}

impl<const N: usize> std::ops::BitOr for BitIndexSet<N> {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        let mut result = Self::default();
        for i in 0..N {
            result.bits[i] = self.bits[i] | rhs.bits[i];
        }
        result
    }
}

impl<const N: usize> std::ops::BitXor for BitIndexSet<N> {
    type Output = Self;

    fn bitxor(self, rhs: Self) -> Self::Output {
        let mut result = Self::default();
        for i in 0..N {
            result.bits[i] = self.bits[i] ^ rhs.bits[i];
        }
        result
    }
}

impl<const N: usize> std::ops::Not for BitIndexSet<N> {
    type Output = Self;

    fn not(self) -> Self::Output {
        let mut result = Self::default();
        for i in 0..N {
            result.bits[i] = !self.bits[i];
        }
        result
    }
}

impl<const N: usize> std::ops::BitAndAssign for BitIndexSet<N> {
    fn bitand_assign(&mut self, rhs: Self) {
        for i in 0..N {
            self.bits[i] &= rhs.bits[i];
        }
    }
}

impl<const N: usize> std::ops::BitOrAssign for BitIndexSet<N> {
    fn bitor_assign(&mut self, rhs: Self) {
        for i in 0..N {
            self.bits[i] |= rhs.bits[i];
        }
    }
}

impl<const N: usize> std::ops::BitXorAssign for BitIndexSet<N> {
    fn bitxor_assign(&mut self, rhs: Self) {
        for i in 0..N {
            self.bits[i] ^= rhs.bits[i];
        }
    }
}

impl<const N: usize> std::fmt::Debug for BitIndexSet<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_list().entries(self.iter()).finish()
    }
}

pub struct BitIndexSetIter<'a, const N: usize> {
    bits: &'a [u64; N],
    cache: u64,
    index: usize,
}

impl<'a, const N: usize> Iterator for BitIndexSetIter<'a, N> {
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

        Some(self.index * 64 + trailing_zeros)
    }
}

#[cfg(test)]
#[test]
fn __test_bits() {
    use tap::Tap;

    let mut bits = BitIndexSet::<3>::default();

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
