use std::{hash::Hash, marker::PhantomData};

use ahash::HashMap;
use bitvec::boxed::BitBox;
use nd::prelude::ArrayViewMut;
use nd::Dimension as _;

pub trait GridIndex: Clone + Copy + Hash + PartialEq + Eq + std::fmt::Debug {
    type Dim: nd::Dimension;

    fn extent() -> Self::Dim;

    fn as_linear_buffer_index(self) -> usize;
    fn as_grid_index(self) -> Self::Dim;

    /// Larger value will be used to determine when we should escape out of sparse state. Smaller
    /// value will be used to determine when we should transition into sparse state from dense
    /// state.
    ///
    /// For example, for value `[0.05, 0.10]`:
    ///
    /// - In [`VarGrid3::Sparse`], when non-background block count ratio exceeds 10%, transition
    ///   into [`VarGrid3::PaletteDense`]
    /// - In [`VarGrid3::PaletteDense`], when the most common block count exceeds 95%(`1.0 -
    ///   0.05`), transition into [`VarGrid3::Sparse`].
    fn sparse_ratio() -> [f32; 2] {
        [0.05, 0.10]
    }
}

#[derive(Clone, Debug)]
pub enum VarGrid<I, T> {
    /// Monostate container.
    Monostate(T),

    /// Sparse data container.  
    Sparse(SparseData<I, T>),

    /// Dense palette buffer; Reduces buffer usage for most cases & quick optimization support
    PaletteDense(PaletteDenseData<I, T>),

    /// This state is never reached unless explicitly initialized
    Dense(DenseData<I, T>),
}

impl<I, T> Default for VarGrid<I, T>
where
    T: Default,
{
    fn default() -> Self {
        Self::Monostate(T::default())
    }
}

impl<I, T> VarGrid<I, T>
where
    I: GridIndex,
    T: Default + Clone + Copy,
{
    pub fn optimize(&mut self) {
        // TODO: see GridIndex::sparse_ratio()
    }

    pub fn get(&self, pos: I) -> T {
        match self {
            VarGrid::Monostate(value) => *value,
            VarGrid::Sparse(sparse) => {
                if let Some(value) = sparse.get(&pos) {
                    *value
                } else {
                    sparse.bg
                }
            }
            VarGrid::Dense(dense) => dense.data()[pos.as_linear_buffer_index()],
            VarGrid::PaletteDense(_) => todo!(),
        }
    }

    pub fn set(&mut self, pos: I, value: T) {
        todo!()
    }

    pub fn make_dense(&mut self) -> &mut DenseData<I, T> {
        if let Self::Dense(data) = self {
            data
        } else {
            self.init_dense_buffer()
        }
    }

    #[cold]
    fn init_dense_buffer(&mut self) -> &mut DenseData<I, T> {
        let dense = match self {
            VarGrid::Monostate(value) => DenseData(
                vec![*value; I::extent().size()].into_boxed_slice(),
                Default::default(),
            ),
            VarGrid::PaletteDense(_) | VarGrid::Sparse(_) => {
                todo!("implement this on demand; currently this method is editor-mode only.")
            }
            VarGrid::Dense(_) => unreachable!(),
        };

        *self = VarGrid::Dense(dense);

        let VarGrid::Dense(dense) = self else {
            unreachable!()
        };

        dense
    }

    /// Destination buffer extent MUST be equal with `I::EXTENT.prod()`
    pub fn dump_data_with_src_pred(
        &self,
        mut dst: ArrayViewMut<T, I::Dim>,
        should_copy: impl Fn(&T) -> bool,
    ) {
        assert!(dst.raw_dim() == I::extent());

        match self {
            VarGrid::Monostate(value) => {
                if should_copy(value) {
                    dst.fill(*value);
                }
            }
            VarGrid::Sparse(sparse) => {
                if should_copy(&sparse.bg) {
                    dst.fill(sparse.bg);
                }

                for (&coord, &cell) in sparse.iter() {
                    dst[coord.as_grid_index()] = cell;
                }
            }
            VarGrid::Dense(dense) => {
                nd::Zip::from(&mut dst)
                    .and(&dense.view())
                    .for_each(|dst, src| {
                        if should_copy(src) {
                            *dst = *src
                        }
                    });
            }
            VarGrid::PaletteDense(_) => todo!(),
        }
    }

    pub fn dump_data(&self, dst: ArrayViewMut<T, I::Dim>) {
        self.dump_data_with_src_pred(dst, |_| true)
    }
}

/* ······························································································ */

#[derive(Clone, Debug)]
pub struct SparseData<I, T> {
    data: HashMap<I, T>,
    bg: T,
}

impl<I, T> SparseData<I, T>
where
    I: GridIndex,
    T: Default + Clone,
{
    pub fn background(&self) -> &T {
        &self.bg
    }
}

impl<I, T> std::ops::Deref for SparseData<I, T>
where
    I: GridIndex,
    T: Default + Clone,
{
    type Target = HashMap<I, T>;

    fn deref(&self) -> &Self::Target {
        &self.data
    }
}

/* ······························································································ */

#[derive(Clone, Debug)]
pub struct PaletteDenseData<I, T> {
    palette: Vec<T>,
    data: BitBox,
    lookup: HashMap<T, LookupArg>,
    _p: PhantomData<I>,
}

#[derive(Clone, Debug)]
struct LookupArg {
    index: u32,
    count: u32,
}

/* ······························································································ */

#[derive(Clone, Debug)]
#[repr(transparent)]
pub struct DenseData<I, T>(Box<[T]>, PhantomData<I>);

impl<I, T> DenseData<I, T>
where
    I: GridIndex,
    T: Clone,
{
    pub fn data(&self) -> &[T] {
        &self.0
    }

    pub fn data_mut(&mut self) -> &mut [T] {
        &mut self.0
    }

    pub fn view(&self) -> nd::ArrayView<T, I::Dim> {
        nd::ArrayView::from_shape(I::extent(), self.data()).unwrap()
    }

    pub fn view_mut(&mut self) -> nd::ArrayViewMut<T, I::Dim> {
        nd::ArrayViewMut::from_shape(I::extent(), self.data_mut()).unwrap()
    }
}

impl<I, T> std::ops::Deref for DenseData<I, T>
where
    I: GridIndex,
    T: Clone,
{
    type Target = [T];

    fn deref(&self) -> &Self::Target {
        self.data()
    }
}

impl<I, T> std::ops::DerefMut for DenseData<I, T>
where
    I: GridIndex,
    T: Clone,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.data_mut()
    }
}
