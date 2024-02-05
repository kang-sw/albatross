use super::*;

#[test]
fn test() {
    struct MyDriver {
        region_index_alloc: usize,
    }

    struct MyElem {
        index: usize,
    }

    impl ElementData for MyElem {
        type Vector = [f32; 3];
    }
}
