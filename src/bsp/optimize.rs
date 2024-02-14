use enum_as_inner::EnumAsInner;
use slotmap::Key;
use tap::Pipe;
use tap::Tap as _;

use super::Element;
use super::Tree;
use super::TreeNode;
use super::TreeNodeLeaf;
use super::TreeNodeSplit;

use crate::bitindex::BitIndexSet;
use crate::primitive::AabbRect;
use crate::primitive::NumberCommon;
use crate::primitive::Vector;
use crate::primitive::VectorExt;

#[non_exhaustive]
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SplitStrategy {
    /// Selects the central point of a dataset as the split point by calculating the
    /// average position of all points. This method aims to divide the dataset around a
    /// central value, providing a straightforward and intuitive splitting strategy.
    Average,

    /// Enhances the average split point calculation by incorporating a weighted average,
    /// where the weights are based on the distance from the initial average point. Points
    /// further away have a greater influence on the final split point. This adjustment
    /// aims to achieve a more spatially balanced partitioning by considering the
    /// distribution of points across the space, potentially leading to clusters that are
    /// more evenly spread out.
    SpatialMedian,

    /// Modifies the average split point calculation by using weights inversely
    /// proportional to the distance from the average point. Closer points have a greater
    /// impact on determining the final split point. This method is designed to highlight
    /// denser clusters, making the central point of the split lean towards areas with a
    /// higher concentration of points, thereby accommodating the clustering tendency
    /// within the dataset.
    ClusterMedian,
}

/// System parameter of BSP
#[non_exhaustive]
#[derive(Debug, Clone)]
pub struct OptimizeParameter {
    /// Defines the minimum number of elements a leaf node must contain before it is
    /// considered for splitting. This threshold helps manage the granularity of spatial
    /// subdivisions within the BSP tree, ensuring nodes are split only when necessary
    /// to maintain an efficient structure.
    ///
    /// This parameter is specifically evaluated for leaf nodes to determine when they
    /// have become sufficiently populated to warrant division into smaller nodes,
    /// thereby optimizing spatial queries by maintaining a balanced distribution of
    /// elements across the tree.
    pub split_threshold: u32,

    /// Specifies the maximum number of elements a node can contain before it is
    /// considered for collapsing back into its parent node. This threshold is crucial
    /// for preventing overly sparse distributions of elements within the tree, ensuring
    /// that nodes are collapsed to consolidate space and improve query performance.
    ///
    /// This parameter is evaluated for nodes that have previously been split (split
    /// nodes) and determines when they should be merged to maintain an optimal tree
    /// structure, especially after deletions or significant movements of elements.
    pub collapse_threshold: u32,

    /// Sets the maximum depth that can be collapsed in a single update cycle, taking
    /// into account the node's balance and the number of child nodes. This parameter
    /// plays a significant role in the tree's rebalancing efforts, ensuring that
    /// collapses are performed judiciously to maintain an efficient and balanced tree
    /// structure.
    ///
    /// By limiting the collapse depth, this prevents excessive collapsing that could
    /// destabilize the tree structure, ensuring that rebalancing is both effective and
    /// conservative.
    pub max_collapse_height: u16,

    /// Within normalized range from 0 to 1, specifies the intensity of the balancing
    /// process. This parameter influences the degree to which the tree is balanced during
    /// optimization, affecting the distribution of elements across the tree and the
    /// efficiency of spatial queries.
    ///
    /// 0 => Disable balancing 1 => Most aggressive balancing
    ///
    /// For example, if the value is 0.3, then the tree will tolerate 30% of unbalance
    /// before it starts to balance. This is default behavior for `moderate` setup.
    pub balancing: f32,

    /// Specifies the algorithm used to calculate the split value when a node is divided
    /// into two child nodes. This parameter influences the method used to determine the
    /// optimal split point, affecting the spatial distribution of elements within the    
    /// tree and the efficiency of spatial queries.
    pub split_strategy: SplitStrategy,

    /// Specifies minimum length of leaf node when splitted.
    pub minimum_length: f64,
    /// ^^ TODO: Tidy these doc comments
    ///

    /// Specifies the size of the snap grid used to align split points and node bounds
    pub snap_size: f64,

    /// If split fails due to the axis that has largest stdvar is too short, how many axis
    /// can be fallback-ed to find suboptimal axis, which has next largest stdvar?
    pub suboptimal_split_count: u8,

    /// For axes specified in this set, the final variance on choosing axis to split is
    /// amplified by the axis length ratio to make the space split square as possible.
    pub square_split_axes: BitIndexSet<1>,

    /// Specifies the height at which the tree should start balancing. This is useful for
    /// trees that are expected to be very large, as it can be beneficial to allow the
    /// tree to grow to a certain size before starting to balance it.
    pub balancing_start_height: u16,
}

impl OptimizeParameter {
    pub fn collapse_all() -> Self {
        Self::disable_all().with(|x| {
            x.split_threshold = u32::MAX;
            x.collapse_threshold = u32::MAX;
        })
    }

    pub fn with(mut self, visit: impl FnOnce(&mut Self)) -> Self {
        visit(&mut self);
        self
    }

    pub fn moderate(split_count: u32) -> Self {
        Self {
            split_threshold: split_count,
            collapse_threshold: split_count / 2,
            max_collapse_height: 8,
            balancing: 0.3,
            split_strategy: SplitStrategy::ClusterMedian,
            minimum_length: 0.,
            suboptimal_split_count: 1,
            square_split_axes: BitIndexSet::empty(),
            balancing_start_height: 4,
            snap_size: 0.,
        }
    }

    pub fn disable_all() -> Self {
        Self {
            split_threshold: u32::MAX,
            collapse_threshold: 0,
            max_collapse_height: u16::MAX,
            balancing: 0.,
            split_strategy: SplitStrategy::Average,
            minimum_length: 0.,
            suboptimal_split_count: 1,
            square_split_axes: BitIndexSet::empty(),
            balancing_start_height: 0,
            snap_size: 0.,
        }
    }
}

/// This is to manage outer context which relies on tree node indices.
#[derive(EnumAsInner)]
pub enum OptimizationEvent<K: Key> {
    Split { from: K, minus: K, plus: K },
    Merge { from: K, into: K },
}

impl<T: Element> Tree<T> {
    /// Optimizes the structure of the BSP tree according to specified parameters,
    /// enhancing its efficiency for spatial querying and management of geometric
    /// elements. This method evaluates the current state of the tree against the
    /// criteria defined in `params` and performs adjustments such as node splitting,
    /// collapsing, or rebalancing to ensure the tree remains optimally organized.
    ///
    /// The optimization process is guided by the `OptimizeParameter` struct, which
    /// includes thresholds and coefficients influencing when and how nodes should be
    /// split or collapsed, as well as depth and balance considerations. By applying
    /// these parameters, the method aims to achieve a balanced and efficient tree
    /// structure, reducing query times and improving overall performance.
    ///
    /// This function is critical for maintaining the BSP tree in response to dynamic
    /// changes within the spatial environment, such as movements or modifications of
    /// contained elements. Regular invocation can help mitigate the effects of
    /// incremental changes, ensuring the tree remains well-suited for efficient
    /// spatial operations.
    ///
    /// Parameters:
    /// - `params`: A reference to an `OptimizeParameter` instance containing the
    ///   configuration values used to guide the optimization process. This includes
    ///   thresholds for node splitting and collapsing, depth limits, and other
    ///   factors critical to maintaining an optimal tree structure.
    pub fn optimize(
        &mut self,
        params: &OptimizeParameter,
        mut on_update: impl FnMut(OptimizationEvent<T::NodeKey>),
    ) {
        /*
            PSEUDO IMPLMENTATION OF CODE

            recurse_phase_1_merge(root);
            recurse_phase_2_split(root);
            recurse_phase_3_validate(root); // Validate all owner pointers of null-pointing nodes

            fn recurse_phase_1_merge(node) {
                if node.is_split() {
                    report1 = recurse_phase_1_merge(node.minus)
                    report2 = recurse_phase_1_merge(node.plus)

                    if evalute_balance(report1, report2) {
                        node.set_to_leaf();

                        merge_recursively(node, node.minus);
                        merge_recursively(node, node.plus);

                        // Now this is `leaf` node, however, all nodes must be pointing
                        // empty owner, and we won't modify them; as they're still able to
                        // be merged by upper nodes!
                    } else {
                        return sum(report1, report2);
                    }
                }

                return leaf_report;
            }

            fn merge_recursively(root, node) {
                if node.is_leaf() {
                    if not node.head.is_null() && node.head.owner != null {
                        // Temporarily clear all node elements' owner
                        for elem in node.iter() {
                            elem.owner = null;
                        }
                    }

                    append(node, root);
                    dispose(node);
                } else {
                    merge_recursively(root, node.minus);
                    merge_recursively(root, node.plus);
                }
            }

            fn recurse_phase_2_split(node) {
                if node.is_split() {
                    recurse_phase_2_split(node.minus);
                    recurse_phase_2_split(node.plus);

                    return;
                }

                // At this point, node MUST be `leaf`
                if node.len > params.split_threshold {
                    assert node.head != null;

                    if node.head.owner != null {
                        // This was valid node previously ... clear all owner info first!
                        for elem in node.iter() {
                            elem.owner = null;
                        }
                    }

                    // Split the node, balanced as possible.

                    // TODO: median? average? clustering?
                    // - 평균값 중심 계산
                    // - 표준편차 계산 => 가장 값이 큰 축 선택
                    // - 평균값에서 가장 먼(편차 큰) 엘리먼트에 가중치 부여해서 축 중심 재계산
                    // - 해당 점 중심으로 스플릿
                }

                // This is consistent; recursive split always creates balanced tree.
                return leaf_report;
            }

            fn recurse_phase_3_validate(node) {
                if node.is_split() {
                    // recurse
                } else {
                    // If head is null, every element of this node has empty owner for
                    // sure.
                    if node.head && node.head.owner == null {
                        for elem in node.iter() {
                            elem.owner = node;
                        }
                    }
                }
            }

            NOTE: `elem.owner` 업데이트 시점 =>
        */

        let root = self.root;
        let context = &mut (self, &mut on_update, params);
        recurse_phase_1(context, root);
        recurse_phase_2(context, root);
        recurse_phase_3_validate(context.0, root);
    }
}

/* ---------------------------------------- Inner Methods --------------------------------------- */

pub(crate) struct P1Report {
    height: u16,
    count: u32,
}

/* ---------------------------------- Phase 1: Collapse --------------------------------- */

macro_rules! f {
    ($context:ident.tree) => {
        $context.0
    };
    ($context:ident.on_update) => {
        $context.1
    };
    ($context:ident.params) => {
        $context.2
    };
}

pub(crate) fn recurse_phase_1<T: Element>(
    context: &mut (
        &mut Tree<T>,
        &mut impl FnMut(OptimizationEvent<T::NodeKey>),
        &OptimizeParameter,
    ),
    node: T::NodeKey,
) -> P1Report {
    match f!(context.tree).nodes[node] {
        TreeNode::Split(TreeNodeSplit {
            minus,
            plus,
            initial_balance,
            ..
        }) => {
            let r_m = recurse_phase_1(context, minus);
            let r_p = recurse_phase_1(context, plus);
            let height = r_m.height.max(r_p.height);

            let params = f!(context.params);
            let collapse = 'collapse: {
                // Just don't try to do anything if lower depth exceeds allowed maximum
                if height > params.max_collapse_height {
                    break 'collapse false;
                }

                let cnt_m = r_m.count as usize;
                let cnt_p = r_p.count as usize;
                let total = cnt_m + cnt_p;

                if total == 0 {
                    break 'collapse true;
                }

                // If both children are leaf node, do not evaluate balance; as it's highly
                // likely just repeat merge/split over and over.
                let disable_balance = height < params.balancing_start_height;

                let unbalanced = 'balance: {
                    if disable_balance || params.balancing <= 0. {
                        break 'balance false;
                    }

                    let larger_count = cnt_m.max(cnt_p) as f32;
                    let balance = r_p.count as i32 - r_m.count as i32;

                    // Adjust the initial balance to avoid unnecessary merge/split
                    // operations that could be triggered by a false-positive from a leaf
                    // initially set to zero, which was created due to a split constraint.
                    let imbalance = (balance - initial_balance).abs();

                    if imbalance == 0 {
                        // Don't touch perfect balance.
                        break 'balance false;
                    }

                    let unbalance_rate = imbalance as f32 / larger_count;
                    let balance_rate = 1. - unbalance_rate;

                    balance_rate < params.balancing
                };

                unbalanced || total <= params.collapse_threshold as usize
            };

            if collapse {
                let (tree, on_update, ..) = context;

                // SAFETY: node is valid split reference.
                let count = unsafe { impl_node_collapse(tree, node, on_update) }.len;

                P1Report { height: 0, count }
            } else {
                P1Report {
                    height: height + 1,
                    count: r_m.count + r_p.count,
                }
            }
        }

        TreeNode::Leaf(ref leaf) => P1Report {
            height: 0,
            count: leaf.len,
        },
    }
}

/// # Safety
///
/// node is VALID leaf node.
unsafe fn impl_node_collapse<'a, T: Element>(
    tree: &'a mut Tree<T>,
    node: T::NodeKey,
    on_collapse: &mut impl FnMut(OptimizationEvent<T::NodeKey>),
) -> &'a mut TreeNodeLeaf<T> {
    let (minus, plus) = tree
        .nodes
        .get_unchecked(node)
        .as_split()
        .unwrap()
        .pipe(|x| (x.minus, x.plus));

    // Convert this node into leaf node.
    *unsafe { tree.nodes.get_unchecked_mut(node) } = TreeNode::Leaf(TreeNodeLeaf {
        // This is just temporary value.
        bound: AabbRect::maximum(),

        head: Key::null(),
        tail: Key::null(),
        len: 0,

        data: T::LeafData::default(),
    });

    // Subnodes' bound will be merged into this.
    let mut bounds: [<T as Element>::Vector; 2] = [
        <T::Vector as VectorExt>::maximum(),
        <T::Vector as VectorExt>::minimum(),
    ];

    impl_collapse_recursive(tree, on_collapse, &mut bounds, node, minus);
    impl_collapse_recursive(tree, on_collapse, &mut bounds, node, plus);

    // It is guaranteed the original range is restored.
    let leaf = tree.nodes.get_unchecked_mut(node).as_leaf_mut().unwrap();
    leaf.bound = AabbRect::new(bounds[0], bounds[1]);

    leaf
}

pub(crate) fn impl_collapse_recursive<T: Element>(
    tree: &mut Tree<T>,
    on_update: &mut impl FnMut(OptimizationEvent<T::NodeKey>),
    bounds: &mut [T::Vector; 2],
    root_id: T::NodeKey,
    node: T::NodeKey,
) {
    match tree.nodes.remove(node).unwrap() {
        TreeNode::Split(TreeNodeSplit { minus, plus, .. }) => {
            impl_collapse_recursive(tree, on_update, bounds, root_id, minus);
            impl_collapse_recursive(tree, on_update, bounds, root_id, plus);
        }
        TreeNode::Leaf(leaf) => {
            on_update(OptimizationEvent::Merge {
                from: node,
                into: root_id,
            });

            // Extend parent bound
            bounds[0] = bounds[0].min_values(leaf.bound.min());
            bounds[1] = bounds[1].max_values(leaf.bound.max());

            if let Some(head) = tree.elems.get_mut(leaf.head) {
                // Mark elements from this node as 'pending validation'
                head.owner = Key::null();
            } else {
                debug_assert!(leaf.len == 0);
                debug_assert!(leaf.tail.is_null());

                // It was just empty leaf.
                return;
            }

            // Append all elements to root node's tail.
            unsafe {
                let root = tree.nodes.get_unchecked_mut(root_id).as_leaf_mut().unwrap();
                let [leaf_head, leaf_tail] = tree
                    .elems
                    .get_disjoint_unchecked_mut([leaf.head, leaf.tail]);

                root.len += leaf.len;

                if root.tail.is_null() {
                    debug_assert!(root.head.is_null());
                    root.head = leaf.head;
                    root.tail = leaf.tail;
                } else {
                    debug_assert!(!root.head.is_null());

                    let prev_root_tail = root.tail;
                    leaf_head.prev = root.tail;
                    leaf_tail.next = Key::null();
                    root.tail = leaf.tail;

                    tree.elems.get_unchecked_mut(prev_root_tail).next = leaf.head;
                }
            }
        }
    }
}

/* ----------------------------------- Phase 2: Split ----------------------------------- */
pub(crate) fn recurse_phase_2<T: Element>(
    context: &mut (
        &mut Tree<T>,
        &mut impl FnMut(OptimizationEvent<T::NodeKey>),
        &OptimizeParameter,
    ),
    node: T::NodeKey,
) {
    match f!(context.tree).nodes[node] {
        TreeNode::Split(TreeNodeSplit { minus, plus, .. }) => {
            // This is dead simple; just visit children nodes.
            recurse_phase_2(context, minus);
            recurse_phase_2(context, plus);

            return; // Returns early.
        }
        TreeNode::Leaf(TreeNodeLeaf {
            bound, head, len, ..
        }) => {
            let (tree, on_update, params) = context;

            if (len as usize) < params.split_threshold.max(1) as usize {
                //            ^^ Makes zero always return.
                return;
            }

            // FIXME: Remove this magic number!
            const SQUARE_SPLIT_CAP: f64 = 64.;

            let mut bound_lengths = T::Vector::zero_f64();
            let bound_length_thr = (params.minimum_length * 2.).max(1e-4);
            for i in 0..T::Vector::D {
                bound_lengths[i] = bound.length(i).to_f64();
            }

            // If all bound lengths are too short, we can't split this node.
            if (0..T::Vector::D).all(|i| bound_lengths[i] < bound_length_thr) {
                return;
            }

            // # Calculate split point
            // 1. Average
            // 2. StdVar per axes
            // 3. a. Spatial Median
            // 3. b. Cluster Median

            let mut avg = tree
                .leaf_iter(node)
                .fold(T::Vector::zero_f64(), |mut val, (_, elem)| {
                    for i in 0..T::Vector::D {
                        val[i] += elem.pos[i].to_f64();
                    }
                    val
                });

            for i in 0..T::Vector::D {
                avg[i] /= len as f64;
            }

            let mut variant =
                tree.leaf_iter(node)
                    .fold(T::Vector::zero_f64(), |mut val, (_, elem)| {
                        for i in 0..T::Vector::D {
                            val[i] += (elem.pos[i].to_f64() - avg[i]).powi(2);
                        }
                        val
                    });

            'square_split: {
                if params.square_split_axes.is_empty() {
                    break 'square_split;
                }

                let max_value = <<T::Vector as Vector>::Num as NumberCommon>::MAXVALUE.to_f64();
                let mut shortest_border = max_value;

                for i in 0..T::Vector::D {
                    if params.square_split_axes[i] {
                        shortest_border = shortest_border.min(bound_lengths[i]);
                    }
                }

                if shortest_border >= max_value {
                    break 'square_split;
                }

                for i in 0..T::Vector::D {
                    if params.square_split_axes.get(i) {
                        variant[i] *= (bound_lengths[i] / shortest_border)
                            .min(SQUARE_SPLIT_CAP)
                            .powi(2);
                    }
                }
            }

            let mut axis = None;

            for _ in 0..(params.suboptimal_split_count + 1).min(T::Vector::D as u8) {
                let axis_max = (0..T::Vector::D)
                    .max_by(|&a, &b| variant[a].partial_cmp(&variant[b]).unwrap())
                    .unwrap();

                // NOTE: We re-evaluate bound lengths per selected axis once more, rather
                // than setting it earlier when we first evaluate bound lengths above.
                // This is to select the axis that has largest stdvar even it's too short.
                // If we don't, the `suboptimal_split_count` will be meaningless.
                if bound_lengths[axis_max].to_f64() > bound_length_thr {
                    axis = Some(axis_max);
                    break;
                }

                variant[axis_max] = f64::MIN;
            }

            // We couldn't find any suitable axis.
            let Some(axis) = axis else { return };

            let split_min = bound.min()[axis].to_f64() + params.minimum_length;
            let split_max = bound.max()[axis].to_f64() - params.minimum_length;

            let mut split_at = match params.split_strategy {
                SplitStrategy::Average => avg[axis],
                ref x @ (SplitStrategy::SpatialMedian | SplitStrategy::ClusterMedian) => {
                    let positive_weight = *x == SplitStrategy::SpatialMedian;
                    let (w, wp) =
                        tree.leaf_iter(node)
                            .fold((0., 0.), |(sum_weight, sum), (_, elem)| {
                                let distance_from_center =
                                    (elem.pos[axis].to_f64() - avg[axis]).abs().max(1e-6);
                                let weight = if positive_weight {
                                    distance_from_center
                                } else {
                                    1. / distance_from_center
                                };

                                let weighted_pos = elem.pos[axis].to_f64() * weight;

                                (sum_weight + weight, sum + weighted_pos)
                            });

                    wp / w
                }
            };

            // Apply snap size.
            if params.snap_size > 0. {
                split_at = (split_at / params.snap_size).round() * params.snap_size;
            }

            // Limit split_at within bound
            split_at = split_at.clamp(split_min, split_max);

            let split_at = <T::Vector as Vector>::Num::from_f64(split_at);
            let (minus_id, plus_id) = split_tree(tree, bound, axis, split_at, head, node);

            // Notify split
            on_update(OptimizationEvent::Split {
                from: node,
                minus: minus_id,
                plus: plus_id,
            });
        }
    }

    // Intentionally adopt early-return style to clean up stacks before recursion.

    let [minus, plus] = unsafe {
        match f!(context.tree).nodes.get_unchecked(node) {
            TreeNode::Split(TreeNodeSplit { minus, plus, .. }) => [*minus, *plus],
            _ => unreachable!(),
        }
    };

    recurse_phase_2(context, minus);
    recurse_phase_2(context, plus);
}

fn split_tree<T: Element>(
    tree: &mut Tree<T>,
    bound: AabbRect<<T as Element>::Vector>,
    axis: usize,
    split_at: <<T as Element>::Vector as Vector>::Num,
    head: <T as Element>::ElemKey,
    node: <T as Element>::NodeKey,
) -> (<T as Element>::NodeKey, <T as Element>::NodeKey) {
    // Create new splitted subnodes.
    let minus = { bound }.tap_mut(|x| x.split_minus(axis, split_at));
    let plus = { bound }.tap_mut(|x| x.split_plus(axis, split_at));

    let minus_id = tree.nodes.insert(TreeNode::Leaf(TreeNodeLeaf {
        bound: minus,
        head: Key::null(),
        tail: Key::null(),
        len: 0,
        data: T::LeafData::default(),
    }));

    let plus_id = tree.nodes.insert(TreeNode::Leaf(TreeNodeLeaf {
        bound: plus,
        head: Key::null(),
        tail: Key::null(),
        len: 0,
        data: T::LeafData::default(),
    }));

    let [minus, plus] = unsafe {
        tree.nodes
            .get_disjoint_unchecked_mut([minus_id, plus_id])
            .map(|x| x.as_leaf_mut().unwrap())
    };

    // Move all elements to subnodes.
    let mut next_head_id = head;

    while let Some(head) = tree.elems.get_mut(next_head_id) {
        let head_id = next_head_id;
        next_head_id = head.next;
        head.next = Key::null();

        let subnode = if head.pos[axis] < split_at {
            &mut *minus
        } else {
            &mut *plus
        };

        subnode.len += 1;

        if subnode.tail.is_null() {
            subnode.head = head_id;
            subnode.tail = head_id;
            head.prev = Key::null();
        } else {
            let prev_tail = subnode.tail;
            subnode.tail = head_id;
            head.prev = prev_tail;

            unsafe {
                tree.elems.get_unchecked_mut(prev_tail).next = head_id;
            }
        }
    }

    let [len_minus, len_plus] = [minus.len, plus.len];

    // Mark this node as `pending validation` by setting head node's owner as
    // null. Since split operation can be performed multiple times over same
    // element, changing the owner of every element everytime is a bit redundant.
    for subnode in [minus, plus] {
        if let Some(head) = tree.elems.get_mut(subnode.head) {
            head.owner = Key::null();
        }
    }

    // Change this node into split. SAFETY: node is VALID!
    unsafe {
        *tree.nodes.get_unchecked_mut(node) = TreeNode::Split(TreeNodeSplit {
            minus: minus_id,
            plus: plus_id,
            axis,
            value: split_at,
            initial_balance: len_plus as i32 - len_minus as i32,
        })
    };
    (minus_id, plus_id)
}

/* ------------------------------------- Phase 3: Validation ------------------------------------ */

pub(crate) fn recurse_phase_3_validate<T: Element>(tree: &mut Tree<T>, node_id: T::NodeKey) {
    // Simply visit all leaf nodes; then pick one.
    match tree.nodes[node_id] {
        TreeNode::Split(TreeNodeSplit { minus, plus, .. }) => {
            recurse_phase_3_validate(tree, minus);
            recurse_phase_3_validate(tree, plus);
        }
        TreeNode::Leaf(TreeNodeLeaf {
            head: mut head_id, ..
        }) => {
            // If head is null, every element of this node has empty owner for sure.
            let is_changed_node = tree
                .elems
                .get_mut(head_id)
                .is_some_and(|x| x.owner.is_null());

            if !is_changed_node {
                return;
            }

            while head_id.is_null() == false {
                unsafe {
                    let head = tree.elems.get_unchecked_mut(head_id);
                    head_id = head.next;

                    head.owner = node_id; // We don't care the previous owner value; it's just invalid.
                    head.data.relocated(node_id);
                }
            }
        }
    }
}

/* ---------------------------------------------------------------------------------------------- */
/*                                   MANUAL OPTIMIZATION METHODS                                  */
/* ---------------------------------------------------------------------------------------------- */

impl<T: Element> Tree<T> {
    /// # Panics
    ///
    /// Panics if `node` is not a valid leaf node.
    pub fn node_split_at(&mut self, node: T::NodeKey, axis: usize, at: <T::Vector as Vector>::Num) {
        let leaf_node = self.nodes.get(node).unwrap().as_leaf().unwrap();
        let bound = leaf_node.bound;

        split_tree(self, bound, axis, at, leaf_node.head, node);
        recurse_phase_3_validate(self, node);
    }

    /// # Panics
    ///
    /// Panics if `node` is not a valid split node
    pub fn node_collapse(
        &mut self,
        node: T::NodeKey,
        on_collapse: impl FnMut(OptimizationEvent<T::NodeKey>),
    ) {
        assert!(self.nodes[node].is_split());

        // SAFETY: We've just verified it's valid split node.
        unsafe { impl_node_collapse(self, node, &mut { on_collapse }) };
        recurse_phase_3_validate(self, node);
    }
}
