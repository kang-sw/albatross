# Primitive Timer Driver Implementation

This is timer driver implementation. The original idea came from [`tokio`](https://tokio.rs/blog/2018-03-timers)'s hierarchical hashed timing wheel, offering `O(1)` insertion/removal and `O(p)` (where `p` is number of timing wheel hierarchies; i.e. page) timeout complexity.

# Usage

todo

# Features

- `std`(default): Provides [`slab`](https://crates.io/crates/slab) based default implementation.
