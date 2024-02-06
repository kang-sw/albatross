use super::*;

#[test]
fn test() {
    struct MyElem {
        index: usize,
        relocated: bool,
    }

    impl ElementData for MyElem {
        type Vector = [i32; 3];

        fn relocated(&mut self, _owner: TreeNodeIndex) {
            self.relocated = true;
        }
    }

    /* ------------------------------------ Verify Insertion ------------------------------------ */

    let mut indexer = 0;
    let mut new_elem = {
        || {
            let elem = MyElem {
                index: indexer,
                relocated: false,
            };
            indexer += 1;
            elem
        }
    };

    let mut bsp = Tree::new();
    let id_1 = bsp.insert([5, 0, 0], new_elem());
    let id_2 = bsp.insert([-5, 0, 0], new_elem());

    assert!(bsp.is_leaf(bsp.root()).unwrap());
    bsp.visit_leaves(|_, leaf| assert!(leaf == bsp.root()));

    let id_3 = bsp.insert([-5, 1, 0], new_elem());

    assert!(bsp[id_1].index == 0);
    assert!(bsp[id_2].index == 1);
    assert!(bsp[id_3].index == 2);

    assert!(bsp.query(&[0, 0, 0]) == bsp.root());

    bsp.leaf_iter(bsp.root())
        .zip(0..3)
        .for_each(|(elem, index)| {
            assert!(elem.index == index);
        });

    let root = bsp.root();
    bsp.__debug_verify_tree_state()
        .map_err(|x| println!("{}", x))
        .unwrap();

    /* -------------------------------------- Verify Split -------------------------------------- */

    let mut new_minus = None;
    let mut new_plus = None;

    bsp.optimize(&OptimizeParameter::moderate(2), |x| match x {
        OptimizationEvent::Split { from, minus, plus } => {
            assert!(from == root);

            assert!(new_minus.is_none());
            assert!(new_plus.is_none());

            new_minus = Some(minus);
            new_plus = Some(plus);
        }
        OptimizationEvent::Merge { .. } => unreachable!(),
    });

    let new_minus = new_minus.unwrap();
    let new_plus = new_plus.unwrap();

    assert!(bsp.leaf_len(new_minus) == 2);
    assert!(bsp.leaf_len(new_plus) == 1);

    bsp.visit_leaves(|_, leaf| {
        assert!(leaf == new_minus || leaf == new_plus);
    });

    bsp.__debug_verify_tree_state()
        .map_err(|x| println!("{}", x))
        .unwrap();

    /* -------------------------------------- Verify Merge -------------------------------------- */

    bsp.optimize(&OptimizeParameter::collapse_all(), |x| match x {
        OptimizationEvent::Split { .. } => unreachable!(),
        OptimizationEvent::Merge { from, into } => {
            assert!(from == new_minus || from == new_plus);
            assert!(into == root);
        }
    });

    assert!(bsp.leaf_len(root) == 3);
    assert!(bsp.is_leaf(new_minus).is_none());
    assert!(bsp.is_leaf(new_plus).is_none());

    bsp.visit_leaves(|_, leaf| assert!(leaf == root));

    bsp.__debug_verify_tree_state()
        .map_err(|x| println!("{}", x))
        .unwrap();
}
