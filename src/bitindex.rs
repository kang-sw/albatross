/// Quickly store set of fixed number of indexes.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct BitIndexer<const N_U64: usize> {
    bits: [u64; N_U64],
}

impl<const N: usize> Default for BitIndexer<N> {
    fn default() -> Self {
        Self { bits: [0; N] }
    }
}

impl<const N: usize> BitIndexer<N> {
    pub const fn len() -> usize {
        N * 64
    }

    pub fn set(&mut self, index: usize, value: bool) {
        let (i, j) = (index / 64, index % 64);
        if value {
            self.bits[i] |= 1 << j;
        } else {
            self.bits[i] &= !(1 << j);
        }
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

    pub fn iter(&self) -> BitIndexerIter<N> {
        BitIndexerIter {
            bits: &self.bits,
            cache: self.bits[0],
            index: 0,
        }
    }
}

impl<const N: usize> std::ops::BitAnd for BitIndexer<N> {
    type Output = Self;

    fn bitand(self, rhs: Self) -> Self::Output {
        let mut result = Self::default();
        for i in 0..N {
            result.bits[i] = self.bits[i] & rhs.bits[i];
        }
        result
    }
}

impl<const N: usize> std::ops::BitOr for BitIndexer<N> {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        let mut result = Self::default();
        for i in 0..N {
            result.bits[i] = self.bits[i] | rhs.bits[i];
        }
        result
    }
}

impl<const N: usize> std::ops::BitXor for BitIndexer<N> {
    type Output = Self;

    fn bitxor(self, rhs: Self) -> Self::Output {
        let mut result = Self::default();
        for i in 0..N {
            result.bits[i] = self.bits[i] ^ rhs.bits[i];
        }
        result
    }
}

impl<const N: usize> std::ops::Not for BitIndexer<N> {
    type Output = Self;

    fn not(self) -> Self::Output {
        let mut result = Self::default();
        for i in 0..N {
            result.bits[i] = !self.bits[i];
        }
        result
    }
}

impl<const N: usize> std::ops::BitAndAssign for BitIndexer<N> {
    fn bitand_assign(&mut self, rhs: Self) {
        for i in 0..N {
            self.bits[i] &= rhs.bits[i];
        }
    }
}

impl<const N: usize> std::ops::BitOrAssign for BitIndexer<N> {
    fn bitor_assign(&mut self, rhs: Self) {
        for i in 0..N {
            self.bits[i] |= rhs.bits[i];
        }
    }
}

impl<const N: usize> std::ops::BitXorAssign for BitIndexer<N> {
    fn bitxor_assign(&mut self, rhs: Self) {
        for i in 0..N {
            self.bits[i] ^= rhs.bits[i];
        }
    }
}

impl<const N: usize> std::fmt::Debug for BitIndexer<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_list().entries(self.iter()).finish()
    }
}

pub struct BitIndexerIter<'a, const N: usize> {
    bits: &'a [u64; N],
    cache: u64,
    index: usize,
}

impl<'a, const N: usize> Iterator for BitIndexerIter<'a, N> {
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

    let mut bits = BitIndexer::<3>::default();

    let trues = [0, 4, 5, 9, 11, 2, 52, 161, 32, 144, 33, 64, 98, 45];
    let sorted_trues = { trues }.tap_mut(|x| x.sort());

    assert!(trues.iter().all(|&i| !bits.get(i)));
    assert!(bits.iter().count() == 0);

    for &i in trues.iter() {
        bits.set(i, true);
    }

    assert!(trues.iter().all(|&i| bits.get(i)));
    assert!(bits.iter().count() == trues.len());
    assert!(bits.iter().zip(sorted_trues).all(|(a, b)| a == b));

    let mut other_bits = bits;
    other_bits.clear();

    for &i in trues.iter() {
        bits.set(i, false);
    }

    assert!(other_bits == bits);
    assert!(bits == Default::default());
}