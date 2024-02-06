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

    bsp.__debug_verify_tree_state().unwrap();
}
