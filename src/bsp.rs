mod optimize;

use crate::primitive::{AabbRect, AxisIndex, Vector};
use enum_as_inner::EnumAsInner;
use slotmap::{Key, SlotMap};
use tap::Tap;

pub use optimize::*;

/* ---------------------------------------------------------------------------------------------- */
/*                                               BSP                                              */
/* ---------------------------------------------------------------------------------------------- */

/// A trait which represents actual data type of BSP.
pub trait ElementData {
    type Vector: Vector;

    /// Mark this element moved. The element MUST NOT move its position within this method
    /// call!
    fn relocated(&mut self, _owner: TreeNodeIndex) {}
}

/// A BSP tree implementation. Hard limit of tree depth is 65,535.
pub struct Tree<T: ElementData> {
    nodes: SlotMap<TreeNodeIndex, TreeNode<T>>,
    elems: SlotMap<ElementIndex, TreeElement<T>>,
    root: TreeNodeIndex,
}

#[derive(EnumAsInner)]
enum TreeNode<T: ElementData> {
    Split(TreeNodeSplit<T>),
    Leaf(TreeNodeLeaf<T>),
}

struct TreeNodeSplit<T: ElementData> {
    axis: AxisIndex,
    value: <T::Vector as Vector>::Num,
    minus: TreeNodeIndex,
    plus: TreeNodeIndex,
}

struct TreeNodeLeaf<T: ElementData> {
    /// For faster evaluation when the node moves
    bound: AabbRect<T::Vector>,
    head: ElementIndex,
    tail: ElementIndex, // To quickly merge two leaves
    len: u32,
}

/* --------------------------------------- Public Tree API -------------------------------------- */

impl<T: ElementData> Default for Tree<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: ElementData> Tree<T> {
    pub fn with_capacity(capacity: usize) -> Self {
        let mut node_pool = SlotMap::with_capacity_and_key(capacity);
        let root_node = node_pool.insert(TreeNode::Leaf(TreeNodeLeaf {
            bound: AabbRect::maximum(),
            head: ElementIndex::null(),
            tail: ElementIndex::null(),
            len: 0,
        }));

        Self {
            nodes: node_pool,
            root: root_node,
            elems: SlotMap::with_capacity_and_key(capacity),
        }
    }

    pub fn new() -> Self {
        Self::with_capacity(0)
    }

    /// It must succeeds, since every region is covered by root.
    pub fn query(&self, pos: &T::Vector) -> TreeNodeIndex {
        let mut index = self.root; // Starts from root

        loop {
            match self.nodes[index] {
                TreeNode::Split(TreeNodeSplit {
                    axis,
                    value,
                    minus,
                    plus,
                }) => {
                    if pos[axis] < value {
                        index = minus;
                    } else {
                        index = plus;
                    }
                }
                TreeNode::Leaf(..) => return index,
            }
        }
    }

    /// Queries nodes with simple rectangular nodes
    pub fn query_region(
        &self,
        rect: &AabbRect<T::Vector>,
        on_query_hit: impl FnMut(&Self, TreeNodeIndex),
    ) {
        self.query_region_with_cutter(rect, on_query_hit, |region, axis, value| {
            // Split the region into two parts based on the specified axis and value
            let minus = { *region }.tap_mut(|x| x.split_minus(axis, value));
            let plus = { *region }.tap_mut(|x| x.split_plus(axis, value));

            [minus, plus]
        })
    }

    pub fn root(&self) -> TreeNodeIndex {
        self.root
    }

    pub fn iter_leaf(&self, id: TreeNodeIndex) -> impl Iterator<Item = &TreeElement<T>> {
        Some(todo!()).into_iter()
    }

    /// # Try not to relocate element
    ///
    /// If one iterating tree node indexes that was hit by query, relocation can move the
    /// element into the node that not visited yet, which makes the iteration on same
    /// element occur more than once.
    pub fn iter_leaf_mut(
        &mut self,
        id: TreeNodeIndex,
    ) -> impl Iterator<Item = &mut TreeElement<T>> {
        Some(todo!()).into_iter()
    }

    pub fn insert(&mut self, pos: T::Vector, entity: T) -> ElementIndex {
        // NOTE: To not lookup `elem_pool` again after insertion, we cache position
        // firstly at here.
        let elem_index = self.elems.insert(TreeElement {
            data: entity,
            pos,
            moved: false,
            owner: TreeNodeIndex::null(),
            prev: ElementIndex::null(),
            next: ElementIndex::null(),
        });

        // Perform insertion; query -> insert.
        let leaf = self.query(&pos);

        // SAFETY: leaf and elem_index both are valid.
        unsafe { self.push_link_back(leaf, elem_index) };

        elem_index
    }

    pub fn remove(&mut self, id: ElementIndex) -> Option<T> {
        if self.elems.contains_key(id) {
            // SAFETY: We've just checked whether the key exists.
            unsafe { self.unlink(id) };

            Some(self.elems.remove(id).unwrap().data)
        } else {
            None
        }
    }

    pub fn get(&self, id: ElementIndex) -> Option<&TreeElement<T>> {
        self.elems.get(id)
    }

    pub fn get_mut(&mut self, id: ElementIndex) -> Option<&mut TreeElement<T>> {
        self.elems.get_mut(id)
    }
}

/* ---------------------------------------- Internal APIs --------------------------------------- */

impl<T: ElementData> Tree<T> {
    /// # Safety
    ///
    /// `leaf` and `elem_index` both are valid element resides in `self`
    unsafe fn push_link_back(&mut self, node: TreeNodeIndex, elem_index: ElementIndex) {
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
            elem.prev = elem_index;
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
    unsafe fn unlink(&mut self, elem_index: ElementIndex) {
        let elem = self.elems.get_unchecked_mut(elem_index);
        let leaf = elem.owner;

        debug_assert!(leaf.is_null() == false);
        elem.owner = TreeNodeIndex::null();

        let elem_prev = elem.prev;
        let elem_next = elem.next;

        // Explicitly unlink nodes.
        elem.prev = ElementIndex::null();
        elem.next = ElementIndex::null();

        let leaf = self.nodes.get_unchecked_mut(leaf).as_leaf_mut().unwrap();

        // Count is managed manually.
        leaf.len -= 1;

        if !elem_prev.is_null() {
            let prev = elem_prev;
            let [elem, prev] = self.elems.get_disjoint_unchecked_mut([elem_index, prev]);

            prev.next = elem.next;
        } else {
            debug_assert!(leaf.head == elem_index);
            leaf.head = elem_next;
        }

        if !elem_next.is_null() {
            let next = elem_next;
            let [elem, next] = self.elems.get_disjoint_unchecked_mut([elem_index, next]);

            next.prev = elem.prev;
        } else {
            debug_assert!(leaf.tail == elem_index);
            leaf.tail = elem_prev;
        }
    }
}

/* -------------------------------------------- Query ------------------------------------------- */

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CutDirection {
    Plus,
    Minus,
}

impl<T: ElementData> Tree<T> {
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
        on_query_hit: impl FnMut(&Self, TreeNodeIndex),
        cut_minus_plus: impl Fn(
            &AabbRect<T::Vector>,
            AxisIndex,
            <T::Vector as Vector>::Num,
        ) -> [AabbRect<T::Vector>; 2],
    ) {
        // Prepare for recursive method call
        let mut param_pack = (on_query_hit, cut_minus_plus);

        // Enter recursive routine
        recurse(self, self.root, region, &mut param_pack);

        fn recurse<T: ElementData>(
            tree: &Tree<T>,
            node_index: TreeNodeIndex,
            region: &AabbRect<T::Vector>,
            methods: &mut (
                impl FnMut(&Tree<T>, TreeNodeIndex),
                impl Fn(
                    &AabbRect<T::Vector>,
                    AxisIndex,
                    <T::Vector as Vector>::Num,
                ) -> [AabbRect<T::Vector>; 2],
            ),
        ) {
            let (on_query_hit, cut_m_p) = methods;
            let split = match &tree.nodes[node_index] {
                TreeNode::Split(sp) => sp,
                TreeNode::Leaf(leaf) => {
                    on_query_hit(tree, node_index);
                    return;
                }
            };

            let (over_min, under_max) = {
                let axis_min = region.min()[split.axis];
                let axis_max = region.max()[split.axis];

                (axis_min <= split.value, split.value < axis_max)
            };

            match (over_min, under_max) {
                (true, true) => {
                    // Only apply cutting if the split crosses the hyperplane.
                    let [minus, plus] = cut_m_p(region, split.axis, split.value);

                    recurse(tree, split.minus, &minus, methods);
                    recurse(tree, split.plus, &plus, methods);
                }
                (false, true) => {
                    recurse(tree, split.plus, region, methods);
                }
                (true, false) => {
                    recurse(tree, split.minus, region, methods);
                }
                (false, false) => {
                    unreachable!()
                }
            }
        }
    }
}

/* ---------------------------------------- Update Logic ---------------------------------------- */

/// Response to update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ElementUpdateResponse {
    Relocated,
    Untouched,
    Removed,
}

impl From<()> for ElementUpdateResponse {
    fn from(_: ()) -> Self {
        Self::Relocated
    }
}

impl From<bool> for ElementUpdateResponse {
    fn from(b: bool) -> Self {
        if b {
            Self::Relocated
        } else {
            Self::Untouched
        }
    }
}

impl From<Option<bool>> for ElementUpdateResponse {
    fn from(opt: Option<bool>) -> Self {
        match opt {
            Some(b) => b.into(),
            None => Self::Removed,
        }
    }
}

impl<T: ElementData> Tree<T> {
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
    pub fn update<R>(&mut self, mut updator: impl FnMut(&mut TreeElement<T>) -> R)
    where
        R: Into<ElementUpdateResponse>,
    {
        // 1. Iterate every nodes

        // 2. Split node evaluates `merge score`
        //   - Balance => If overly unbalanced, collapse tree and re-evaluate split.
        //   - Total count => If drops under threshold, collapse tree.

        // 3. Leaf node evaluates `split score`
        //   - Simply depends on count of element.

        let root = self.root;

        let mut context = UpdateContext {
            tree: self,
            updator: &mut updator,
            relocate_head: ElementIndex::null(),
        };

        // Phase 1: Recursive element update
        recurse_update(&mut context, root);

        // Phase 2: Insert all floating elements.
        let mut relocate = context.relocate_head;

        while !relocate.is_null() {
            let elem = &mut self.elems[relocate];
            let elem_index = relocate;
            relocate = elem.next;

            // unlink the element manually.
            elem.next = ElementIndex::null();

            let pos = elem.pos;
            let leaf = self.query(&pos);

            // SAFETY: We know leaf and elem index both are valid.
            unsafe {
                self.push_link_back(leaf, elem_index);
                self.elems.get_unchecked_mut(elem_index).relocated(leaf);
            };
        }

        /* ------------------------------------ Inline Items ------------------------------------ */

        struct UpdateContext<
            'a,
            T: ElementData,
            F: FnMut(&mut TreeElement<T>) -> R,
            R: Into<ElementUpdateResponse>,
        > {
            tree: &'a mut Tree<T>,
            updator: &'a mut F,
            relocate_head: ElementIndex,
        }

        fn recurse_update<
            T: ElementData,
            F: FnMut(&mut TreeElement<T>) -> R,
            R: Into<ElementUpdateResponse>,
        >(
            ctx: &mut UpdateContext<T, F, R>,
            node: TreeNodeIndex,
        ) {
            let UpdateContext {
                tree,
                updator: update_element,
                relocate_head,
                ..
            } = ctx;

            match tree.nodes[node] {
                TreeNode::Split(TreeNodeSplit { minus, plus, .. }) => {
                    recurse_update(ctx, minus);
                    recurse_update(ctx, plus);
                }
                TreeNode::Leaf(TreeNodeLeaf {
                    bound: leaf_bound,
                    head: leaf_head,
                    ..
                }) => {
                    let mut elem_index = leaf_head;

                    while !elem_index.is_null() {
                        let elem = &mut tree.elems[elem_index];
                        elem_index = elem.next;

                        debug_assert!(elem.owner == node);

                        match update_element(elem).into() {
                            ElementUpdateResponse::Relocated => {
                                // Check if element position still resides in current leaf
                                // node's boundary.
                                if leaf_bound.contains(&elem.pos) {
                                    // It's just okay with no-op.
                                    continue;
                                }

                                // If it was moved, put it to pending-relocate list

                                // SAFETY: We're already aware that node and element are valid!
                                unsafe { tree.unlink(elem_index) };

                                let elem = &mut tree.elems[elem_index];

                                // It's okay with just monodirectional link.
                                elem.next = *relocate_head;
                                *relocate_head = elem_index;
                            }
                            ElementUpdateResponse::Removed => {
                                // SAFETY: We're already aware that node and element are valid!
                                unsafe { tree.unlink(elem_index) };
                                tree.elems.remove(elem_index);
                            }
                            ElementUpdateResponse::Untouched => {
                                // The updator function MUST be properly implemented.
                                debug_assert!(leaf_bound.contains(&elem.pos));
                            }
                        }
                    }
                }
            }
        }
    }
}

/* ----------------------------------------- Entity Type ---------------------------------------- */

pub struct TreeElement<T: ElementData> {
    data: T,

    pos: T::Vector,
    moved: bool,

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
    owner: TreeNodeIndex,
    prev: ElementIndex,
    next: ElementIndex,
}

impl<E: ElementData> TreeElement<E> {
    pub fn pos(&self) -> &E::Vector {
        &self.pos
    }

    pub fn set_pos(&mut self, pos: E::Vector) {
        self.pos = pos;
        self.moved = true;
    }

    pub fn owner(&self) -> TreeNodeIndex {
        self.owner
    }
}

impl<E: ElementData> std::ops::Deref for TreeElement<E> {
    type Target = E;

    fn deref(&self) -> &Self::Target {
        &self.data
    }
}

impl<E: ElementData> std::ops::DerefMut for TreeElement<E> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.data
    }
}

/* ------------------------------------------ Id Types ------------------------------------------ */

slotmap::new_key_type! {
    /// Index of tree node
    pub struct TreeNodeIndex;

    /// Index of tree element
    pub struct ElementIndex;
}

/* ---------------------------------------------------------------------------------------------- */
/*                                              TESTS                                             */
/* ---------------------------------------------------------------------------------------------- */

#[cfg(test)]
mod __test;
