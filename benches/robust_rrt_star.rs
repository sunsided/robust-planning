//! Benchmark for Robust RRT*, whose hot path is proving edges for choose-parent and
//! re-proving moved subtrees for rewiring.

#[path = "../tests/it/affine.rs"]
mod affine;

use std::hint::black_box;

use criterion::Criterion;
use polyzonotope::{Interval, IntervalBox, Polytope, SparsePolyZonotope, Zonotope};
use rand_pcg::Pcg64Mcg;
use robust_planning::{
    Obstacles, PolytopeGoal,
    robust_rrt::Problem,
    robust_rrt_star::{Config, RobustRrtStar},
    sampling_planning::{UniformBox, rand_core::SeedableRng},
};

/// Iterations per benchmarked run.
const ITERATIONS: usize = 500;

/// Runs every benchmark in this file with criterion's command-line flags.
fn main() {
    let mut criterion = Criterion::default().configure_from_args();

    bench_slalom(&mut criterion);
    criterion.final_summary();
}

/// The box `[x0, x1] x [y0, y1]`.
fn rect(x0: f64, x1: f64, y0: f64, y1: f64) -> IntervalBox {
    IntervalBox(vec![Interval::new(x0, x1), Interval::new(y0, y1)])
}

/// The slalom from (1, 1) to (9, 9) in the 10 x 10 world.
fn problem() -> Problem<affine::Affine<2>, UniformBox<2>, 2> {
    let walls = [rect(4.0, 5.0, 0.0, 7.0), rect(6.0, 7.0, 3.0, 10.0)];

    Problem {
        reach: affine::Affine::contracting(0.8, 0.01, 0.02),
        obstacles: Obstacles::new(walls.iter().map(Polytope::from_box).collect(), 4),
        sampler: UniformBox::new([0.0, 0.0], [10.0, 10.0]),
        start: [1.0, 1.0],
        start_set: SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&rect(
            0.9, 1.1, 0.9, 1.1,
        ))),
        goal: PolytopeGoal::from_box(&rect(8.5, 9.5, 8.5, 9.5), 8),
    }
}

/// Grows the slalom tree for [`ITERATIONS`] iterations with the default rewiring budget.
fn bench_slalom(criterion: &mut Criterion) {
    let config = Config::new(10.0, 1.0);

    criterion.bench_function("robust_rrt_star_slalom_500", |b| {
        b.iter(|| {
            let mut planner = RobustRrtStar::new(problem(), config).expect("valid problem");
            planner.run(ITERATIONS, &mut Pcg64Mcg::seed_from_u64(1));
            black_box(planner.path())
        });
    });
}
