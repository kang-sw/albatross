mod optimize;
mod trace;

use std::{fmt::Debug, mem::replace};

use crate::primitive::{AabbRect, AxisIndex, DirectionSegment, Number, Vector};
use enum_as_inner::EnumAsInner;
use slab::Slab;
use slotmap::{Key, SlotMap};
use tap::Tap;

pub use optimize::*;

/// Maximum count of axis that can be used in BSP tree.
pub const NUM_MAX_AXIS: usize = 64;

/* ---------------------------------------------------------------------------------------------- */
/*                                               BSP                                              */
/* ---------------------------------------------------------------------------------------------- */

/// A trait which represents actual data type of BSP.
pub trait Element {
    type Vector: Vector;
    type NodeKey: Key;
    type ElemKey: Key;

    /// Any data type that implements [`Default`] and [`Clone`] can be put into leaf node.
    type LeafData: Default + Clone;

    /// Mark this element moved.
    fn relocated(&mut self, _owner: Self::NodeKey) {}

    /// Returns the radius of this element. This does not affect to BSP tree itself,
    /// however, referred by tracing methods when evaluating collision.
    ///
    /// It does not provide 100% perfect result when the shape is much larger than minimum
    /// split size or at some corner cases. Therefore it is recommended to set some margin
    /// when evaluating collision rectangle.
    fn extent(&self) -> TraceShape<Self::Vector> {
        TraceShape::Sphere(Number::ZERO)
    }
}

/// A BSP tree implementation. Hard limit of tree depth is 65,535.
///
/// Each leaf node can contain 2^31-1 elements. (MSB is reserved)
#[derive(Clone)]
pub struct Tree<T: Element> {
    nodes: SlotMap<T::NodeKey, TreeNode<T>>,
    elems: SlotMap<T::ElemKey, TreeElement<T>>,
    leaf_bodies: Slab<LeafNodeBody<T>>,
    root: T::NodeKey,
}

#[derive(Debug, Clone, EnumAsInner)]
enum TreeNode<T: Element> {
    Split(TreeNodeSplit<T>),
    Leaf(TreeNodeLeaf<T>),
}

/// Splits the region into two parts based on the specified axis and value(hyperplane).
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct TreeNodeSplit<T: Element> {
    pub axis: u8,
    pub value: <T::Vector as Vector>::Num,

    pub minus: T::NodeKey,
    pub plus: T::NodeKey,

    /// The value is computed as `plus.count - minus.count`, which represents the level of
    /// imbalance in the split. This ensures that the split node is not evaluated as
    /// unbalanced even if it was initially empty or had a bias.
    balance_bias: i32,
}

#[derive(Debug, Clone)]
struct TreeNodeLeaf<T: Element> {
    head: T::ElemKey,
    tail: T::ElemKey, // To quickly merge two leaves
    len: u32,
    body_id: u32,
}

/// Separate body of leaf node to reduce memory usage, as split node count is mostly 1:1
/// with leaf node count, and the leaf node size easily exceeds split node size in order
/// of magnitude.
#[derive(Debug, Clone)]
pub struct LeafNodeBody<T: Element> {
    pub bound: AabbRect<T::Vector>,
    pub data: T::LeafData,
}

/* -------------------------------------- Trace Data Types -------------------------------------- */

/// A shape descriptor for an element.
#[derive(Debug, Clone, Copy)]
pub enum TraceShape<V: Vector> {
    /// Value is extent of each axis for AABB.
    Aabb(V),

    /// Value is radius for sphere.
    Sphere(V::Num),

    /// Capsule extent; Line and radius
    Capsule {
        dir: DirectionSegment<V>,
        radius: V::Num,
    },
}

/* ----------------------------------------- Misc Impls ----------------------------------------- */

impl<T: Element> TreeNodeSplit<T> {
    #[inline(always)]
    pub fn axis(&self) -> usize {
        self.axis as usize
    }
}

/* ----------------------------------------- Trait Impls ---------------------------------------- */

impl<T: Element> Default for Tree<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Element> Debug for Tree<T>
where
    T::NodeKey: Debug,
    T::ElemKey: Debug,
    T: Debug,
    T::Vector: Debug,
    T::LeafData: Debug,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Tree")
            .field("nodes", &self.nodes)
            .field("elems", &self.elems)
            .field("leaf_bodies", &self.leaf_bodies)
            .field("root", &self.root)
            .finish()
    }
}

/* --------------------------------------- Public Tree API -------------------------------------- */

impl<T: Element> Tree<T> {
    pub fn with_capacity(node_capacity: usize, elem_capacity: usize) -> Self {
        Self {
            nodes: SlotMap::with_capacity_and_key(node_capacity),
            leaf_bodies: Slab::with_capacity((node_capacity / 2 + 1).min(node_capacity)),
            elems: SlotMap::with_capacity_and_key(elem_capacity),
            root: T::NodeKey::null(),
        }
        .tap_mut(|x| x.init_root_node())
    }

    fn init_root_node(&mut self) {
        let (root_node, ..) = self.create_leaf_node();
        self.root = root_node;
    }

    pub fn new() -> Self {
        Self::with_capacity(0, 0)
    }

    pub fn len_all_elems(&self) -> usize {
        self.elems.len()
    }

    /// It must succeeds, since every region is covered by root.
    pub fn query(&self, pos: &T::Vector) -> T::NodeKey {
        self.query_with_hierarchy(self.root(), pos, |_, _| {})
    }

    /// # Panics
    ///
    /// If `index` is invalid.
    pub fn query_with_hierarchy(
        &self,
        mut index: T::NodeKey,
        pos: &T::Vector,
        mut visit_trail: impl FnMut(&TreeNodeSplit<T>, bool), // is plus?
    ) -> T::NodeKey {
        assert!(self.nodes.contains_key(index));

        loop {
            match unsafe { self.nodes.get_unchecked(index) } {
                TreeNode::Split(split) => {
                    if pos[split.axis()] < split.value {
                        index = split.minus;
                        visit_trail(split, false);
                    } else {
                        index = split.plus;
                        visit_trail(split, true);
                    }
                }
                TreeNode::Leaf(..) => return index,
            }
        }
    }

    /// Queries nodes with simple rectangular nodes
    pub fn query_region(&self, rect: &AabbRect<T::Vector>, on_query_hit: impl FnMut(T::NodeKey)) {
        self.query_region_with_cutter(rect, on_query_hit, |region, axis, value| {
            // Split the region into two parts based on the specified axis and value
            let minus = { *region }.tap_mut(|x| x.apply_split_minus(axis, value));
            let plus = { *region }.tap_mut(|x| x.apply_split_plus(axis, value));

            [minus, plus]
        })
    }

    // TODO: `trace_line`

    // TODO: `trace_line_thick`

    // TODO: `trace_sphere`

    pub fn root(&self) -> T::NodeKey {
        self.root
    }

    pub fn leaf_data(&self, id: T::NodeKey) -> &T::LeafData {
        self.get_leaf_node(id).map(|x| &x.1.data).unwrap()
    }

    pub fn leaf_data_mut(&mut self, id: T::NodeKey) -> &mut T::LeafData {
        self.get_leaf_node_mut(id).map(|x| &mut x.1.data).unwrap()
    }

    pub fn leaf_len(&self, id: T::NodeKey) -> u32 {
        self.nodes[id].as_leaf().unwrap().len
    }

    pub fn is_leaf(&self, id: T::NodeKey) -> Option<bool> {
        self.nodes.get(id).map(|node| node.is_leaf())
    }

    pub fn leaf_bound(&self, id: T::NodeKey) -> &AabbRect<T::Vector> {
        &self.get_leaf_node(id).unwrap().1.bound
    }

    pub fn split_info(&self, id: T::NodeKey) -> Option<&TreeNodeSplit<T>> {
        self.nodes.get(id).and_then(|node| node.as_split())
    }

    pub fn clear(&mut self) {
        self.nodes.clear();
        self.elems.clear();
        self.leaf_bodies.clear();

        self.init_root_node();
    }

    /// # Panics
    ///
    /// `id` is invalid.
    pub fn leaf_iter(&self, id: T::NodeKey) -> impl Iterator<Item = (T::ElemKey, &TreeElement<T>)> {
        struct TreeIter<'a, T: Element> {
            tree: &'a Tree<T>,
            elem: T::ElemKey,
        }

        impl<'a, T: Element> Iterator for TreeIter<'a, T> {
            type Item = (T::ElemKey, &'a TreeElement<T>);

            fn next(&mut self) -> Option<Self::Item> {
                // SAFETY: as long as it exist, the index is always valid.
                // - We're utilizing unchecked version here to skip version validation.

                if self.elem.is_null() {
                    return None;
                }

                unsafe {
                    let elem = self.tree.elems.get_unchecked(self.elem);

                    let elem_id = self.elem;
                    self.elem = elem.next;

                    Some((elem_id, elem))
                }
            }
        }

        TreeIter {
            elem: self.nodes[id].as_leaf().unwrap().head,
            tree: self,
        }
    }

    // TODO: Find way to implement `iter_leaf_mut`

    /// # Try not to relocate element
    ///
    /// If one iterating tree node indexes that was hit by query, relocation can move the
    /// element into the node that not visited yet, which makes the iteration on same
    /// element occur more than once.
    pub fn leaf_for_each_mut(
        &mut self,
        id: T::NodeKey,
        mut visit: impl FnMut(&mut TreeElementEdit<T>),
    ) {
        let mut elem_id = self.nodes[id].as_leaf_mut().unwrap().head;

        while elem_id.is_null() == false {
            let elem = &mut self.elems[elem_id];
            elem_id = elem.next;

            let mut edit = TreeElementEdit::new(self, elem_id);
            visit(&mut edit);
        }
    }

    pub fn visit_leaves(&self, mut insert: impl FnMut(T::NodeKey)) {
        fn recurse<T: Element>(
            tree: &Tree<T>,
            node: T::NodeKey,
            insert: &mut impl FnMut(T::NodeKey),
        ) {
            match unsafe { tree.nodes.get_unchecked(node) } {
                TreeNode::Split(split) => {
                    recurse(tree, split.minus, insert);
                    recurse(tree, split.plus, insert);
                }
                TreeNode::Leaf(_) => {
                    insert(node);
                }
            }
        }

        recurse(self, self.root, &mut insert);
    }

    pub fn visit_leaves_with_depth(&self, at: T::NodeKey, mut insert: impl FnMut(u16, T::NodeKey)) {
        fn recurse<T: Element>(
            tree: &Tree<T>,
            node: T::NodeKey,
            depth: u16,
            insert: &mut impl FnMut(u16, T::NodeKey),
        ) {
            match unsafe { tree.nodes.get_unchecked(node) } {
                TreeNode::Split(split) => {
                    recurse(tree, split.minus, depth + 1, insert);
                    recurse(tree, split.plus, depth + 1, insert);
                }
                TreeNode::Leaf(_) => {
                    insert(depth, node);
                }
            }
        }

        recurse(self, at, 0, &mut insert);
    }

    pub fn insert(&mut self, pos: T::Vector, entity: T) -> T::ElemKey {
        // NOTE: To not lookup `elem_pool` again after insertion, we cache position
        // firstly at here.
        let elem_index = self.elems.insert(TreeElement {
            data: entity,
            pos,
            owner: T::NodeKey::null(),
            prev: T::ElemKey::null(),
            next: T::ElemKey::null(),
        });

        // Perform insertion; query -> insert.
        let leaf = self.query(&pos);

        // SAFETY: leaf and elem_index both are valid.
        unsafe { self.push_link_back(leaf, elem_index) };

        elem_index
    }

    pub fn remove(&mut self, id: T::ElemKey) -> Option<T> {
        if self.elems.contains_key(id) {
            // SAFETY: We've just checked whether the key exists.
            unsafe { self.unlink(id) };

            Some(self.elems.remove(id).unwrap().data)
        } else {
            None
        }
    }

    pub fn get(&self, id: T::ElemKey) -> Option<&TreeElement<T>> {
        self.elems.get(id)
    }

    pub fn get_mut(&mut self, id: T::ElemKey) -> Option<TreeElementEdit<T>> {
        if self.elems.contains_key(id) {
            Some(TreeElementEdit::new(self, id))
        } else {
            None
        }
    }

    pub fn get_mut_unchecked(&mut self, id: T::ElemKey) -> TreeElementEdit<T> {
        TreeElementEdit::new(self, id)
    }

    pub fn get_mut_n<const N: usize>(
        &mut self,
        keys: [T::ElemKey; N],
    ) -> Option<[&mut TreeElement<T>; N]> {
        self.elems.get_disjoint_mut(keys)
    }

    pub fn contains_element(&self, id: T::ElemKey) -> bool {
        self.elems.contains_key(id)
    }

    /// # Safety
    ///
    /// This should only be used if `contains_element(key)` is true for every given
    /// key and no two keys are equal. Otherwise it is potentially unsafe.
    pub unsafe fn get_mut_n_unchecked<const N: usize>(
        &mut self,
        keys: [T::ElemKey; N],
    ) -> [&mut TreeElement<T>; N] {
        #[cfg(debug_assertions)]
        {
            self.elems.get_disjoint_mut(keys).unwrap()
        }

        #[cfg(not(debug_assertions))]
        {
            self.elems.get_disjoint_unchecked_mut(keys)
        }
    }
}

impl<T: Element> std::ops::Index<T::ElemKey> for Tree<T> {
    type Output = TreeElement<T>;

    fn index(&self, index: T::ElemKey) -> &Self::Output {
        &self.elems[index]
    }
}

impl<T: Element> std::ops::IndexMut<T::ElemKey> for Tree<T> {
    fn index_mut(&mut self, index: T::ElemKey) -> &mut Self::Output {
        &mut self.elems[index]
    }
}

/* ---------------------------------------- Verification ---------------------------------------- */

impl<T: Element> Tree<T> {
    /// An API to check internal state. Solely debug purpose; thus do not use this!
    #[doc(hidden)]
    pub fn __debug_verify_tree_state(&self) -> Result<usize, String> {
        let mut errors = String::new();
        let mut leaf_bounds = Vec::new();

        let mut len_total = 0;

        self.visit_leaves(|node| {
            let leaf = self.nodes[node].as_leaf().unwrap();
            let body = &self.leaf_bodies[leaf.body_id as usize];
            leaf_bounds.push((node, body.bound));

            let count = self.leaf_iter(node).count();
            len_total += count;

            if count != leaf.len as usize {
                errors.push_str(&format!(
                    "Leaf node {:?}.len = {}, mismatches actual count {}\n",
                    node, leaf.len, count
                ));
            }

            let mut prev = Key::null();

            let mut number = 0;
            let mut head_id = leaf.head;
            while let Some(elem) = self.get(head_id) {
                if elem.owner != node {
                    errors.push_str(&format!("Leaf {:?}: elem {number} is not owned \n", node));
                }

                if elem.prev != prev {
                    errors.push_str(&format!(
                        "Leaf {:?}: elem {number} has wrong prev, desired: {:?}, actual: {:?} \n",
                        node, prev, elem.prev
                    ));
                }

                number += 1;
                prev = head_id;
                head_id = elem.next;
            }
        });

        for ((n1, c1), (n2, c2)) in leaf_bounds.iter().zip(leaf_bounds.iter().skip(1)) {
            if c1.intersects(c2) {
                errors.push_str(&format!("Leaf bounds {:?} and {:?} intersects\n", n1, n2));
            }
        }

        // Verify memory leak
        let mut leaf_body_count = self.leaf_bodies.len();
        let mut leaf_count = self.nodes.iter().filter(|x| x.1.is_leaf()).count();
        let mut leaf_mask = vec![false; self.leaf_bodies.capacity()];

        if leaf_body_count != leaf_count {
            errors.push_str(&format!(
                "Leaf body count {} mismatches leaf count {}\n",
                leaf_body_count, leaf_count
            ));
        }

        self.visit_leaves(|leaf_id| {
            leaf_count -= 1;
            let (leaf, _) = self.get_leaf_node(leaf_id).unwrap();

            let occupied = replace(&mut leaf_mask[leaf.body_id as usize], true);

            if occupied {
                errors.push_str(&format!(
                    "{:?} Leaf body {:?} is already occupied\n",
                    leaf_id, leaf.body_id
                ));
            } else {
                leaf_body_count -= 1;
            }
        });

        if leaf_count != leaf_body_count {
            errors.push_str(&format!(
                "leaf_count {leaf_count} != leaf_body_count {leaf_body_count}\n"
            ));
        }

        if leaf_count != 0 {
            errors.push_str(&format!("leaf node is leaked (N: {leaf_count})\n"));
        }

        if errors.is_empty() {
            Ok(len_total)
        } else {
            Err(errors)
        }
    }
}
/* ---------------------------------------- Internal APIs --------------------------------------- */

impl<T: Element> Tree<T> {
    /// # Safety
    ///
    /// `leaf` and `elem_index` both are valid element resides in `self`
    unsafe fn push_link_back(&mut self, node: T::NodeKey, elem_index: T::ElemKey) {
        let elem = self.elems.get_unchecked_mut(elem_index);

        debug_assert!(elem.owner.is_null());
        elem.owner = node;

        // Check if all links are disconnected.
        debug_assert!(elem.prev.is_null());
        debug_assert!(elem.next.is_null());

        let leaf = self.nodes.get_unchecked_mut(node).as_leaf_mut().unwrap();

        // Count is managed manually.
        leaf.len += 1;

        if leaf.tail.is_null() {
            debug_assert!(leaf.head.is_null());

            leaf.head = elem_index;
            leaf.tail = elem_index;
        } else {
            debug_assert!(leaf.head.is_null() == false);

            let tail = leaf.tail;
            leaf.tail = elem_index;

            let [elem, prev_tail] = self.elems.get_disjoint_unchecked_mut([elem_index, tail]);

            prev_tail.next = elem_index;
            elem.prev = tail;
        }
    }

    /// This is marked unsafe to reduce overhead during referring multiple same references
    /// over and over to avoid borrow checker complaints.
    ///
    /// > The logical safety MUST be validated during debug phase.
    ///
    /// # Safety
    ///
    /// `leaf` and `elem_index` both are valid element resides in `self`
    unsafe fn unlink(&mut self, elem_index: T::ElemKey) {
        let elem = self.elems.get_unchecked_mut(elem_index);
        let leaf_id = elem.owner;

        debug_assert!(leaf_id.is_null() == false);
        elem.owner = T::NodeKey::null();

        let elem_prev = elem.prev;
        let elem_next = elem.next;

        // Explicitly unlink nodes.
        elem.prev = T::ElemKey::null();
        elem.next = T::ElemKey::null();

        let leaf = self.nodes.get_unchecked_mut(leaf_id).as_leaf_mut().unwrap();

        // Count is managed manually.
        leaf.len -= 1;

        if !elem_prev.is_null() {
            let prev = self.elems.get_unchecked_mut(elem_prev);
            prev.next = elem_next;
        } else {
            debug_assert!(leaf.head == elem_index);
            leaf.head = elem_next;
        }

        if !elem_next.is_null() {
            let next = self.elems.get_unchecked_mut(elem_next);
            next.prev = elem_prev;
        } else {
            debug_assert!(leaf.tail == elem_index);
            leaf.tail = elem_prev;
        }
    }

    fn create_leaf_node(&mut self) -> (T::NodeKey, &mut TreeNodeLeaf<T>, &mut LeafNodeBody<T>) {
        let body_id = self.leaf_bodies.insert(LeafNodeBody {
            bound: AabbRect::maximum(),
            data: T::LeafData::default(),
        });

        let leaf = self.nodes.insert(TreeNode::Leaf(TreeNodeLeaf {
            head: T::ElemKey::null(),
            tail: T::ElemKey::null(),
            len: 0,
            body_id: body_id as _,
        }));

        (
            leaf,
            self.nodes.get_mut(leaf).unwrap().as_leaf_mut().unwrap(),
            self.leaf_bodies.get_mut(body_id).unwrap(),
        )
    }

    fn cvt_split_to_leaf(
        &mut self,
        node: T::NodeKey,
    ) -> (&mut TreeNodeLeaf<T>, &mut LeafNodeBody<T>) {
        self.nodes[node] = TreeNode::Leaf(TreeNodeLeaf {
            head: T::ElemKey::null(),
            tail: T::ElemKey::null(),
            len: 0,
            body_id: self.leaf_bodies.insert(LeafNodeBody {
                bound: AabbRect::maximum(),
                data: T::LeafData::default(),
            }) as _,
        });

        self.get_leaf_node_mut(node).unwrap()
    }

    fn cvt_leaf_to_split(&mut self, node: T::NodeKey, data: TreeNodeSplit<T>) {
        let body = replace(&mut self.nodes[node], TreeNode::Split(data));
        self.leaf_bodies
            .remove(body.as_leaf().unwrap().body_id as _);
    }

    fn get_leaf_node(&self, key: T::NodeKey) -> Option<(&TreeNodeLeaf<T>, &LeafNodeBody<T>)> {
        self.nodes.get(key).and_then(|node| {
            node.as_leaf().map(|x| unsafe {
                let id = x.body_id as _;
                (x, self.leaf_bodies.get_unchecked(id))
            })
        })
    }

    fn get_leaf_node_mut(
        &mut self,
        key: T::NodeKey,
    ) -> Option<(&mut TreeNodeLeaf<T>, &mut LeafNodeBody<T>)> {
        self.nodes.get_mut(key).and_then(|node| {
            node.as_leaf_mut().map(|x| unsafe {
                let id = x.body_id as _;
                (x, self.leaf_bodies.get_unchecked_mut(id))
            })
        })
    }
}

/* -------------------------------------------- Query ------------------------------------------- */

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CutDirection {
    Plus,
    Minus,
}

impl<T: Element> Tree<T> {
    /// This method performs a spatial query within a Binary Space Partitioning (BSP)
    /// region, efficiently organizing and querying geometric data. It requires a custom
    /// `cut` function provided by the caller to divide the given region into two separate
    /// rectangles based on a specific axis and value. This division is crucial for
    /// partitioning the space into manageable sections that can be easily queried and
    /// optimized for different geometric shapes.
    ///
    /// The `cut` function is designed to take a region (represented as
    /// `AabbRect<T::Vector>`) and split it into two new rectangles
    /// (`[AabbRect<T::Vector>; 2]`), according to the specified axis (`AxisIndex`) and
    /// value (`<T::Vector as Vector>::Num`). This is straightforward for rectangular
    /// regions, resulting in two smaller rectangles. However, the flexibility of the
    /// `cut` function allows for optimizing queries involving more complex shapes by
    /// implementing shape-specific logic. Examples include:
    ///
    /// - Diagonal Lines: After splitting, it may allow the non-split axis of the AABB
    ///   rectangle to be adjusted for a more accurate representation of the divided
    ///   space.
    /// - Ellipses/Spheres: The function can compute the intersection of the shape with a
    ///   plane defined by the cut, refining the query to the actual impacted area.
    ///
    /// The method also requires an `on_query_hit` callback function, which is invoked to
    /// report the indices of leaf nodes that fall within the queried region. This allows
    /// the caller to collect or process these indices according to their specific needs,
    /// such as appending them to a list or array for further handling. This callback is
    /// essential for identifying which parts of the partitioned space are relevant to the
    /// query, enabling applications to respond dynamically to spatial queries.
    ///
    /// Through the combination of a custom `cut` function and the `on_query_hit`
    /// callback, this method offers a powerful and flexible approach to spatial querying
    /// in BSP trees. It supports efficient data organization and querying for a wide
    /// range of geometric shapes and is adaptable to various application-specific
    /// requirements.
    ///
    /// Parameters:
    /// - `region`: The region within the BSP tree to be queried, represented by an
    ///   axis-aligned bounding box (`AabbRect<T::Vector>`).
    /// - `on_query_hit`: A callback function that is called with the index of each leaf
    ///   node that is determined to be within the query region. This allows the caller to
    ///   track which parts of the space are affected by the query.
    /// - `cut`: A function that defines how to split the provided region along a
    ///   specified axis and value, resulting in two subdivided regions
    ///   (`AabbRect<T::Vector>` objects). This function enables the method to adapt to
    ///   the spatial characteristics of various geometric shapes.
    pub fn query_region_with_cutter(
        &self,
        region: &AabbRect<T::Vector>,
        on_query_hit: impl FnMut(T::NodeKey),
        cut_minus_plus: impl Fn(
            &AabbRect<T::Vector>,
            AxisIndex,
            <T::Vector as Vector>::Num,
        ) -> [AabbRect<T::Vector>; 2],
    ) {
        // Prepare for recursive method call
        let mut param_pack = (on_query_hit, cut_minus_plus);

        // Enter recursive routine
        self.impl_query_region_with_cutter(self.root, region, &mut param_pack);
    }

    fn impl_query_region_with_cutter(
        &self,
        node_index: T::NodeKey,
        region: &AabbRect<T::Vector>,
        methods: &mut (
            impl FnMut(T::NodeKey),
            impl Fn(
                &AabbRect<T::Vector>,
                AxisIndex,
                <T::Vector as Vector>::Num,
            ) -> [AabbRect<T::Vector>; 2],
        ),
    ) {
        let (on_query_hit, cut_m_p) = methods;
        let split = match &self.nodes[node_index] {
            TreeNode::Split(sp) => sp,
            TreeNode::Leaf(_) => {
                on_query_hit(node_index);
                return;
            }
        };

        let (over_min, under_max) = {
            let axis_min = region.min()[split.axis()];
            let axis_max = region.max()[split.axis()];

            (axis_min <= split.value, split.value < axis_max)
        };

        match (over_min, under_max) {
            (true, true) => {
                // Only apply cutting if the split crosses the hyperplane.
                let [minus, plus] = cut_m_p(region, split.axis(), split.value);

                self.impl_query_region_with_cutter(split.minus, &minus, methods);
                self.impl_query_region_with_cutter(split.plus, &plus, methods);
            }
            (false, true) => {
                self.impl_query_region_with_cutter(split.plus, region, methods);
            }
            (true, false) => {
                self.impl_query_region_with_cutter(split.minus, region, methods);
            }
            (false, false) => {
                unreachable!()
            }
        }
    }
}

/* ---------------------------------------- Update Logic ---------------------------------------- */

impl<T: Element> Tree<T> {
    /// This method updates the internal status of nodes within a tree structure and
    /// serves as a form of garbage collection (GC) for managing the elements within the
    /// tree. It accepts a visitor function that is invoked for each element in the tree.
    /// The return value of this visitor function should be an `ElementUpdateResponse`,
    /// indicating the action to be taken for each element (e.g., whether an element
    /// should be relocated, left untouched, or removed).
    ///
    /// The `ElementUpdateResponse` enum is used to signal the outcome of updating an
    /// element:
    /// - `Relocated`: Indicates that the element has been moved within the tree. This is
    ///   the default response for operations that don't explicitly return a value.
    /// - `Untouched`: Suggests that the element remains in its current position without
    ///   changes.
    /// - `Removed`: Specifies that the element should be removed from the tree, typically
    ///   used to indicate that it is no longer relevant or needed.
    ///
    /// Conversions from `()`, `bool`, and `Option<bool>` to `ElementUpdateResponse` are
    /// provided to simplify the usage of common patterns in update logic. For instance,
    /// returning `true` from the updater function implies the element has been relocated,
    /// `false` implies it remains untouched, and `None` (when using `Option<bool>`)
    /// signals that the element should be removed.
    ///
    /// It is highly recommended to invoke the `Tree::optimize` method following an update
    /// operation. While the tree structure remains valid post-update(as long as you
    /// properly implement updator function), the efficiency of querying or iterating over
    /// the tree may be compromised due to potential structural inefficiencies. Calling
    /// `optimize` helps to restructure the tree for optimal performance.
    ///
    /// Parameters:
    /// - `updator`: A mutable closure that is applied to each `TreeElement<T>` within the
    ///   tree. The closure's return type, `R`, must be convertible into an
    ///   `ElementUpdateResponse`, dictating the action to be taken for the element it is
    ///   applied to.
    pub fn update<R>(&mut self, mut updator: impl FnMut(&mut TreeElementEdit<T>)) {
        // 1. Iterate every nodes

        // 2. Split node evaluates `merge score`
        //   - Balance => If overly unbalanced, collapse tree and re-evaluate split.
        //   - Total count => If drops under threshold, collapse tree.

        // 3. Leaf node evaluates `split score`
        //   - Simply depends on count of element.

        let root = self.root;

        let mut context = (&mut *self, &mut updator, T::ElemKey::null());

        // Phase 1: Recursive element update
        update_inner_recurse(&mut context, root);

        // Phase 2: Consume all relocations
        context.0.impl_consume_relocation(context.2);
    }

    /// Update given disjoint leaves. It is useful when evaluating region query result,
    /// while preventing duplicated evaluation(update) of entity which is moved into a
    /// node that wasn't evaluated yet.
    pub fn update_disjoint_leaves<I: Iterator<Item = T::NodeKey> + Clone>(
        &mut self,
        nodes: impl IntoIterator<IntoIter = I> + Clone,
        updator: impl FnMut(&mut TreeElementEdit<T>),
    ) {
        let nodes = nodes.into_iter();

        let all_disjoint = 'finish: {
            for node_id in nodes.clone() {
                let Some(node) = self.nodes.get_mut(node_id).and_then(|x| x.as_leaf_mut()) else {
                    break 'finish false;
                };

                if node.len & 0x8000_0000 != 0 {
                    break 'finish false;
                }

                node.len |= 0x8000_0000;
            }

            true
        };

        if all_disjoint {
            for node_id in nodes.clone() {
                // SAFETY: All nodes are proven valid.
                unsafe {
                    self.nodes
                        .get_unchecked_mut(node_id)
                        .as_leaf_mut()
                        .unwrap()
                        .len &= 0x7FFF_FFFF;
                }
            }

            // SAFETY: We've just checked if it's safe to do so.
            unsafe { self.update_disjoint_leaves_unchecked(nodes, updator) }
        } else {
            for node_id in nodes {
                let Some(node) = self.nodes.get_mut(node_id).and_then(|x| x.as_leaf_mut()) else {
                    break;
                };

                node.len &= 0x7FFF_FFFF;
            }
        }
    }

    /// Update given disjoint leaves. It is useful when evaluating region query result,
    /// while preventing duplicated evaluation(update) of entity which is moved into a
    /// node that wasn't evaluated yet.
    ///
    /// # Safety
    ///
    /// This should only be used if `contains_element(key)` is true for every given
    /// key and no two keys are equal. Otherwise it is potentially unsafe.    
    pub unsafe fn update_disjoint_leaves_unchecked(
        &mut self,
        nodes: impl IntoIterator<Item = T::NodeKey>,
        mut updator: impl FnMut(&mut TreeElementEdit<T>),
    ) {
        let mut ctx = (&mut *self, &mut updator, Key::null());

        for node_id in nodes {
            update_inner_recurse(&mut ctx, node_id);
        }

        ctx.0.impl_consume_relocation(ctx.2);
    }

    fn impl_consume_relocation(&mut self, mut relocate: T::ElemKey) {
        while !relocate.is_null() {
            let elem = &mut self.elems[relocate];
            let elem_index = relocate;
            relocate = elem.next;

            // unlink the element manually.
            elem.next = T::ElemKey::null();

            let pos = elem.pos;
            let leaf = self.query(&pos);

            // SAFETY: We know leaf and elem index both are valid.
            unsafe {
                self.push_link_back(leaf, elem_index);
                self.elems.get_unchecked_mut(elem_index).relocated(leaf);
            };
        }
    }
}

fn update_inner_recurse<T: Element, F: FnMut(&mut TreeElementEdit<T>)>(
    ctx: &mut (&mut Tree<T>, &mut F, T::ElemKey),
    node: T::NodeKey,
) {
    let (tree, update_element, relocate_head) = ctx;

    match tree.nodes[node] {
        TreeNode::Split(TreeNodeSplit { minus, plus, .. }) => {
            update_inner_recurse(ctx, minus);
            update_inner_recurse(ctx, plus);
        }
        TreeNode::Leaf(TreeNodeLeaf {
            head: leaf_head,
            body_id,
            ..
        }) => {
            let leaf_bound = tree.leaf_bodies[body_id as usize].bound;
            let mut elem_index = leaf_head;

            while !elem_index.is_null() {
                let elem = &mut tree.elems[elem_index];
                elem_index = elem.next;

                debug_assert!(elem.owner == node);

                let mut edit = TreeElementEdit::new(tree, elem_index);
                update_element(&mut edit);

                if !edit.moved {
                    // In this case, it's just okay to let the default drop guard to deal
                    // with the result(untouched / removed).
                    continue;
                }

                // Since in this context we have to deal with the deferred
                // relocation to prevent update logic run duplicates on single
                // elemnt, here we have to manually deal with `moved` response.
                edit.moved = false; // Prevent default move drop guard to run.
                drop(edit);

                // SAFETY: `elem_index` is proven exist.
                let elem = unsafe { tree.elems.get_unchecked_mut(elem_index) };

                // Check if element position still resides in current leaf
                // node's boundary.
                if leaf_bound.contains(&elem.pos) {
                    // It's just okay with no-op.
                    continue;
                }

                // If it was moved, put it to pending-relocate list

                // SAFETY: We're already aware that node and element are valid!
                let elem = unsafe {
                    tree.unlink(elem_index);
                    tree.elems.get_unchecked_mut(elem_index)
                };

                // It's okay with just monodirectional link.
                elem.next = *relocate_head;
                *relocate_head = elem_index;
            }
        }
    }
}

/* ---------------------------------------- Node Visitor ---------------------------------------- */

impl<T: Element> Tree<T> {
    pub fn visit_split_nodes(&self, epoch: T::NodeKey, visit: &impl Fn(&TreeNodeSplit<T>)) {
        match &self.nodes[epoch] {
            TreeNode::Split(split) => {
                visit(split);
                self.visit_split_nodes(split.minus, visit);
                self.visit_split_nodes(split.plus, visit);
            }
            TreeNode::Leaf(_) => {}
        }
    }
}

/* ----------------------------------------- Entity Type ---------------------------------------- */

#[derive(Debug, Clone)]
pub struct TreeElement<T: Element> {
    data: T,

    pos: T::Vector,

    // Retaining owner
    //
    // Advs:
    //   1. Better remove ergonomics
    //   2. Owner-swtiching(relocation) detection (*IMPORTANT*!)
    //
    // Cons:
    //   1. Adds O(n) complexity during optimization. It makes O(1) to O(n) thus pretty
    //      significant, however, it is simple linear substitution unlike complicated
    //      computation of splitting ops which is much more frequent, therefore its added
    //      overhead regarded negligible.
    owner: T::NodeKey,
    prev: T::ElemKey,
    next: T::ElemKey,
}

impl<T: Element> TreeElement<T> {
    pub fn pos(&self) -> &T::Vector {
        &self.pos
    }

    pub fn owner(&self) -> T::NodeKey {
        self.owner
    }
}

impl<E: Element> std::ops::Deref for TreeElement<E> {
    type Target = E;

    fn deref(&self) -> &Self::Target {
        &self.data
    }
}

impl<E: Element> std::ops::DerefMut for TreeElement<E> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.data
    }
}

/* ---------------------------------------- Element Proxy --------------------------------------- */

pub struct TreeElementEdit<'a, T: Element> {
    tree: &'a mut Tree<T>,
    elem: T::ElemKey,
    moved: bool,
    removed: bool,
}

impl<'a, T: Element> TreeElementEdit<'a, T> {
    fn new(this: &'a mut Tree<T>, elem: T::ElemKey) -> Self {
        debug_assert!(this.elems.contains_key(elem));

        Self {
            tree: this,
            elem,
            moved: false,
            removed: false,
        }
    }

    fn _get_elem_mut(&mut self) -> &mut TreeElement<T> {
        // SAFETY: On creation, element validity is verified
        unsafe { self.tree.elems.get_unchecked_mut(self.elem) }
    }

    fn _get_elem(&self) -> &TreeElement<T> {
        // SAFETY: On creation, element validity is verified
        unsafe { self.tree.elems.get_unchecked(self.elem) }
    }

    pub fn index(&self) -> T::ElemKey {
        self.elem
    }

    pub fn pos(&self) -> &T::Vector {
        &self._get_elem().pos
    }

    pub fn owner(&self) -> T::NodeKey {
        self._get_elem().owner
    }

    pub fn set_pos(&mut self, pos: T::Vector) {
        debug_assert!(!self.removed, "Trying to modify removed element");

        self._get_elem_mut().pos = pos;
        self.moved = true;
    }

    pub fn remove(&mut self) {
        debug_assert!(!self.removed, "Trying to modify removed element");

        self.removed = true;
    }
}

impl<T: Element> Drop for TreeElementEdit<'_, T> {
    fn drop(&mut self) {
        if self.removed {
            // SAFETY: `self.elem` is valid.
            unsafe { self.tree.unlink(self.elem) };
            self.tree.elems.remove(self.elem);
        } else if self.moved {
            // SAFETY: `self.elem` is valid element resides in valid node.
            unsafe {
                let elem = self.tree.elems.get_unchecked_mut(self.elem);
                let pos = elem.pos;

                let owner_id = elem.owner;

                let (_, owner_body) = self.tree.get_leaf_node_mut(owner_id).unwrap();
                if owner_body.bound.contains(&pos) {
                    return;
                }

                // SAFETY: `leaf` and `elem` both are valid.
                self.tree.unlink(self.elem);

                let leaf_id = self.tree.query(&pos);

                debug_assert!(owner_id != leaf_id);

                self.tree.push_link_back(leaf_id, self.elem);
                self.tree.get_mut_unchecked(self.elem).relocated(leaf_id);
            };
        }
    }
}

impl<'a, E: Element> std::ops::Deref for TreeElementEdit<'a, E> {
    type Target = E;

    fn deref(&self) -> &Self::Target {
        &self._get_elem().data
    }
}

impl<'a, E: Element> std::ops::DerefMut for TreeElementEdit<'a, E> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self._get_elem_mut().data
    }
}

/* ------------------------------------------ Id Types ------------------------------------------ */

slotmap::new_key_type! {
    /// Index of tree node
    pub struct NodeKey;

    /// Index of tree element
    pub struct ElemKey;
}

/* ---------------------------------------------------------------------------------------------- */
/*                                              TESTS                                             */
/* ---------------------------------------------------------------------------------------------- */

#[cfg(test)]
mod __test;
