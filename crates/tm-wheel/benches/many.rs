use std::time::Instant;

use criterion::{criterion_group, criterion_main, Bencher, Criterion};
use fastrand::Rng;
use tm_wheel::TimerDriver;

/* ---------------------------------------- Insert Remove --------------------------------------- */

fn insert_remove<const PAGE: usize, const PAGE_SIZE: usize>(c: &mut Bencher) {
    // Let timer items distribute evenly
    c.iter_custom(|iters| {
        let mut rng = Rng::with_seed(0);
        let mut tm = TimerDriver::<u32, PAGE, PAGE_SIZE>::default();
        let mut handles = Vec::with_capacity(iters as _);
        tm.reserve(iters as _);

        let now = Instant::now();

        for it in 0..iters {
            handles.push(tm.insert(it as _, rng.u64(0..100_000_000)));
        }

        for id in handles {
            tm.remove(id);
        }

        now.elapsed()
    });
}

fn bench_insert_remove(c: &mut Criterion) {
    let mut c = c.benchmark_group("insertion");
    c.bench_function("16p2b", insert_remove::<16, 4>);
    c.bench_function("11p3b", insert_remove::<11, 8>);
    c.bench_function("8p4b", insert_remove::<8, 16>);
    c.bench_function("7p5b", insert_remove::<7, 32>);
    c.bench_function("6p6b", insert_remove::<6, 64>);
    c.bench_function("5p7b", insert_remove::<5, 128>);
    c.finish();
}

/* ---------------------------------------- Advance Timer --------------------------------------- */

fn advance_batch<const PAGE: usize, const PAGE_SIZE: usize>(c: &mut Bencher) {
    c.iter_custom(|iters| {
        let mut rng = Rng::with_seed(0);
        let mut tm = TimerDriver::<u32, PAGE, PAGE_SIZE>::default();

        tm.reserve(iters as _);

        for it in 0..iters {
            tm.insert(it as _, rng.u64(0..10_000_000));
        }

        let now = Instant::now();
        let items = tm.advance_to(10_000_000);
        let elapsed = now.elapsed();

        // This is benchmark for slab
        drop(items);

        assert!(tm.is_empty());
        elapsed
    });
}

fn advance_step<const PAGE: usize, const PAGE_SIZE: usize>(c: &mut Bencher) {
    c.iter_custom(|iters| {
        let mut rng = Rng::with_seed(0);
        let mut tm = TimerDriver::<u32, PAGE, PAGE_SIZE>::default();

        tm.reserve(iters as _);

        for it in 0..iters {
            tm.insert(it as _, rng.u64(0..10_000));
        }

        let now = Instant::now();

        for tick in 0..10_000 {
            tm.advance_to(tick);
        }

        assert!(tm.is_empty());
        now.elapsed()
    });
}

fn bench_advance(c: &mut Criterion) {
    {
        let mut c = c.benchmark_group("advance batched");
        c.bench_function("16p2b", advance_batch::<16, 4>);
        c.bench_function("11p3b", advance_batch::<11, 8>);
        c.bench_function("8p4b", advance_batch::<8, 16>);
        c.bench_function("7p5b", advance_batch::<7, 32>);
        c.bench_function("6p6b", advance_batch::<6, 64>);
        c.bench_function("5p7b", advance_batch::<5, 128>);
        c.finish();
    }

    {
        let mut c = c.benchmark_group("advance steps");
        c.bench_function("16p2b", advance_step::<16, 4>);
        c.bench_function("11p3b", advance_step::<11, 8>);
        c.bench_function("8p4b", advance_step::<8, 16>);
        c.bench_function("7p5b", advance_step::<7, 32>);
        c.bench_function("6p6b", advance_step::<6, 64>);
        c.bench_function("5p7b", advance_step::<5, 128>);
        c.finish();
    }
}

criterion_main!(benches);
criterion_group!(benches, bench_insert_remove, bench_advance);
