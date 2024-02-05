use enum_as_inner::EnumAsInner;
use slotmap::Key as _;
use tap::Pipe as _;
use tap::Tap as _;

use super::ElementData;
use super::ElementIndex;
use super::Tree;
use super::TreeNode;
use super::TreeNodeIndex;
use super::TreeNodeLeaf;
use super::TreeNodeSplit;

use crate::primitive::AabbRect;
use crate::primitive::AxisIndex;
use crate::primitive::Vector;
use crate::primitive::VectorExt;
use crate::ControlIntensity;

#[non_exhaustive]
#[derive(Clone, Debug)]
pub enum SplitAlgorithm {
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

    /// Indicates a depth level at which leaf nodes begin to resist splitting, and once
    /// split, they are more readily collapsed back unless under extreme conditions of
    /// overcrowding. This parameter is designed to control the depth of the tree,
    /// ensuring that it does not become too deep, which can adversely affect performance.
    ///
    /// By setting an ideal depth limit, this encourages a more balanced tree structure
    /// by delaying splits and favoring collapses at deeper levels, thereby optimizing
    /// the spatial efficiency and query performance of the BSP tree.
    pub ideal_depth_limit: u16,

    /// Determines the influence of the `ideal_depth_limit` on tree optimization
    /// decisions. This coefficient adjusts how aggressively the tree adheres to the
    /// ideal depth limit, affecting the willingness of leaf nodes to split or collapse
    /// based on their depth in the tree.
    ///
    /// A higher coefficient increases the effect of the depth limit, promoting a more
    /// compact tree structure by discouraging deep splits and encouraging early
    /// collapses.
    pub ideal_depth_limit_coeff: ControlIntensity,

    /// Sets the maximum depth that can be collapsed in a single update cycle, taking
    /// into account the node's balance and the number of child nodes. This parameter
    /// plays a significant role in the tree's rebalancing efforts, ensuring that
    /// collapses are performed judiciously to maintain an efficient and balanced tree
    /// structure.
    ///
    /// By limiting the collapse depth, this prevents excessive collapsing that could
    /// destabilize the tree structure, ensuring that rebalancing is both effective and
    /// conservative.
    pub collapse_depth: u16,

    /// Influences how the balance between different branches of the tree affects
    /// optimization decisions. This coefficient determines the weight of balance
    /// considerations in adjusting thresholds for splitting and collapsing nodes,
    /// aiming to maintain a uniformly distributed tree structure.
    ///
    /// A higher balance coefficient places greater emphasis on achieving a balanced
    /// tree, potentially adjusting optimization strategies to prevent the development
    /// of highly unbalanced branches.
    pub balance_coeff: ControlIntensity,

    /// Modifies thresholds for splitting and collapsing based on the node's height in
    /// the tree, to counteract potential imbalances. As nodes are closer to the root,
    /// this coefficient incrementally tightens the criteria for maintaining or dividing
    /// nodes, addressing scenarios where a tree may become unbalanced with active leaf
    /// nodes concentrated at disparate depths.
    ///
    /// This adaptive threshold adjustment ensures that the tree remains balanced, even
    /// in cases where spatial distributions might lead to uneven depth utilization,
    /// thereby enhancing the overall efficiency and effectiveness of the BSP tree's
    /// spatial partitioning capabilities.
    pub node_height_coeff: ControlIntensity,

    /// Specifies the algorithm used to calculate the split value when a node is divided
    /// into two child nodes. This parameter influences the method used to determine the
    /// optimal split point, affecting the spatial distribution of elements within the    
    /// tree and the efficiency of spatial queries.
    pub split_algorithm: SplitAlgorithm,
}

impl Default for OptimizeParameter {
    fn default() -> Self {
        Self {
            split_threshold: 20,
            collapse_threshold: 10,
            ideal_depth_limit: 20, // Won't be treggiered practically.
            ideal_depth_limit_coeff: ControlIntensity::Moderate,
            collapse_depth: 3,
            balance_coeff: ControlIntensity::Moderate,
            node_height_coeff: ControlIntensity::Moderate,
            split_algorithm: SplitAlgorithm::Average,
        }
    }
}

impl OptimizeParameter {
    pub fn collapse_all() -> Self {
        Self::default().tap_mut(|x| {
            x.split_threshold = u32::MAX;
            x.collapse_threshold = u32::MAX;
        })
    }
}

/// This is to manage outer context which relies on tree node indices.
#[derive(EnumAsInner)]
pub enum OptimizationEvent<V: Vector> {
    Split {
        region: AabbRect<V>,
        axis: AxisIndex,
        value: <V as Vector>::Num,

        from: TreeNodeIndex,
        minus: TreeNodeIndex,
        plus: TreeNodeIndex,
    },
    Merge {
        from: TreeNodeIndex,
        into: TreeNodeIndex,
    },
}

impl<T: ElementData> Tree<T> {
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
        mut on_update: impl FnMut(OptimizationEvent<T::Vector>),
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
        recurse_phase_1(&mut (self, &mut on_update, params), 0, root);

        return todo!();
    }
}

/* ---------------------------------------- Inner Methods --------------------------------------- */

pub(crate) struct P1Report {
    height: u16,
    count: u32,
}

// Balance evaluation => 0 at perfect balance. For moderate
// config, unbalance is allowed until 80%. For extreme config,
// it's 40%.
pub(crate) const UNBALANCE_THRES_PCNT: [usize; 2] = [80, 40];

// The tree becomes easy to be collapsed
pub(crate) const HEIGHT_INTENSITY: [f32; 3] = [1.00, 1.04, 1.15];

// The tree becomes hard to be splitted
pub(crate) const OVER_DEPTH_INTENSITY: [f32; 3] = [1.00, 1.10, 1.40];

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

pub(crate) fn recurse_phase_1<T: ElementData>(
    context: &mut (
        &mut Tree<T>,
        &mut impl FnMut(OptimizationEvent<T::Vector>),
        &OptimizeParameter,
    ),
    depth: u16,
    node: TreeNodeIndex,
) -> P1Report {
    match f!(context.tree).nodes[node] {
        TreeNode::Split(TreeNodeSplit { minus, plus, .. }) => {
            let r1 = recurse_phase_1(context, depth + 1, minus);
            let r2 = recurse_phase_1(context, depth + 1, plus);
            let height = r1.height.max(r2.height);

            let params = f!(context.params);
            let collapse = 'collapse: {
                // Just don't try to do anything if lower depth exceeds allowed maximum
                if height > params.collapse_depth {
                    break 'collapse false;
                }

                let cnt1 = r1.count as usize;
                let cnt2 = r2.count as usize;
                let total = cnt1 + cnt2;

                let balance_coeff = params.balance_coeff as usize;
                let unbalanced = if balance_coeff > 0 {
                    let unbalance_threshold = UNBALANCE_THRES_PCNT[balance_coeff - 1];
                    let diff = cnt1.abs_diff(cnt2);

                    let unbalance_percent = diff * 100 / total;
                    unbalance_percent >= unbalance_threshold
                } else {
                    false
                };

                unbalanced || {
                    // Increasing threshold coefficient makes
                    let threshold = {
                        let over_depth = depth.saturating_sub(params.ideal_depth_limit);
                        params.collapse_threshold as f32
                            * OVER_DEPTH_INTENSITY[params.ideal_depth_limit_coeff as usize]
                                .powf(over_depth as _)
                            * HEIGHT_INTENSITY[params.node_height_coeff as usize].powf(height as _)
                    } as usize;

                    // Increment collapse threshold
                    total < threshold
                }
            };

            if collapse {
                // Convert this node into leaf node.
                f!(context.tree).nodes[node] = TreeNode::Leaf(TreeNodeLeaf {
                    // This is just temporary value.
                    bound: AabbRect::maximum(),

                    head: ElementIndex::null(),
                    tail: ElementIndex::null(),
                    len: 0,
                });

                // Subnodes' bound will be merged into this.
                let mut bounds: [<T as ElementData>::Vector; 2] = [
                    <T::Vector as VectorExt>::maximum(),
                    <T::Vector as VectorExt>::minimum(),
                ];

                collapse_recursive(context, &mut bounds, node, minus);
                collapse_recursive(context, &mut bounds, node, plus);

                // It is guaranteed the original range is restored.
                let leaf = f!(context.tree).nodes[node].as_leaf_mut().unwrap();
                leaf.bound = AabbRect::new(bounds[0], bounds[1]);

                P1Report {
                    height: 0,
                    count: leaf.len,
                }
            } else {
                P1Report {
                    height: height + 1,
                    count: r1.count + r2.count,
                }
            }
        }

        TreeNode::Leaf(ref leaf) => P1Report {
            height: 0,
            count: leaf.len,
        },
    }
}

pub(crate) fn collapse_recursive<T: ElementData>(
    context: &mut (
        &mut Tree<T>,
        &mut impl FnMut(OptimizationEvent<T::Vector>),
        &OptimizeParameter,
    ),
    bounds: &mut [T::Vector; 2],
    root_id: TreeNodeIndex,
    node: TreeNodeIndex,
) {
    match f!(context.tree).nodes.remove(node).unwrap() {
        TreeNode::Split(TreeNodeSplit { minus, plus, .. }) => {
            collapse_recursive(context, bounds, root_id, minus);
            collapse_recursive(context, bounds, root_id, plus);

            // Remove itself, as its subnode contents are already merged into root.
            let _r = f!(context.tree).nodes.remove(node);
            debug_assert!(_r.is_some());
        }
        TreeNode::Leaf(leaf) => {
            let (tree, on_update, _) = context;

            on_update(OptimizationEvent::Merge {
                from: node,
                into: root_id,
            });

            // Extend parent bound
            bounds[0] = bounds[0].min_values(leaf.bound.min());
            bounds[1] = bounds[1].max_values(leaf.bound.max());

            if leaf.head.is_null() {
                debug_assert!(leaf.len == 0);
                debug_assert!(leaf.tail.is_null());

                // It was just empty leaf.
                return;
            }

            // Append all elems to root
            let root = tree.nodes[root_id].as_leaf_mut().unwrap();
            let head = &mut tree.elems[leaf.head];

            // Appending list of nodes to another is O(n) when it's first time,
            // O(1) when it is recursively collapsed. The `owner` field decided to
            // be retained as compared to its memory and calculation overhead, the
            // advantage of it was more significant.
            let root_tail = root.tail;
            root.len += leaf.len;
            root.tail = leaf.head;
            head.prev = root_tail;

            if !head.owner.is_null() {
                // Mark the very first owner node as null to indicate this node is
                // being relocated.
                head.owner = TreeNodeIndex::null();
            }

            if let Some(tail) = tree.elems.get_mut(root_tail) {
                tail.next = leaf.head;
            }
        }
    }
}

/* ----------------------------------- Phase 2: Split ----------------------------------- */
pub(crate) fn recurse_phase_2<T: ElementData>(
    context: &mut (
        &mut Tree<T>,
        &mut impl FnMut(OptimizationEvent<T::Vector>),
        &OptimizeParameter,
    ),
    depth: u16,
    node: TreeNodeIndex,
) {
    match f!(context.tree).nodes[node] {
        TreeNode::Split(TreeNodeSplit { minus, plus, .. }) => {
            // This is dead simple; just visit children nodes.
            recurse_phase_2(context, depth + 1, minus);
            recurse_phase_2(context, depth + 1, plus);
        }
        TreeNode::Leaf(TreeNodeLeaf {
            bound,
            head,
            tail,
            len,
        }) => {
            let over_depth = depth.saturating_sub(f!(context.params).ideal_depth_limit);
            let thres = (f!(context.params).split_threshold as f32)
                .tap_mut(|x| {
                    *x *= OVER_DEPTH_INTENSITY[f!(context.params).ideal_depth_limit_coeff as usize]
                        .powf(over_depth as f32)
                })
                .pipe(|x| x as usize);

            if (len as usize) < thres {
                // Just let it be.
                return;
            }

            // Actual split operation.
        }
    }
}
