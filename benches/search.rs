//! Benchmark for the reachability-guided search, whose hot path is propagating and checking
//! one edge per motion primitive.

#[path = "../tests/it/affine.rs"]
mod affine;

use std::hint::black_box;

use criterion::Criterion;
use polyzonotope::{Interval, IntervalBox, Polytope, SparsePolyZonotope, Zonotope};
use robust_planning::{
    Obstacles, PolytopeGoal,
    search::{Config, GoalRadius, Problem, ReachSearch},
};

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

/// The slalom from (1, 1) to (9, 9) in a framed 10 x 10 world, 8-neighborhood at 0.5.
fn problem() -> Problem<affine::Affine<2>, 2> {
    let walls = [
        rect(4.0, 5.0, 0.0, 7.0),
        rect(6.0, 7.0, 3.0, 10.0),
        rect(-1.0, 11.0, -1.0, 0.0),
        rect(-1.0, 11.0, 10.0, 11.0),
        rect(-1.0, 0.0, -1.0, 11.0),
        rect(10.0, 11.0, -1.0, 11.0),
    ];
    let step = [-0.5, 0.0, 0.5];
    let primitives = step
        .iter()
        .flat_map(|&dx| step.iter().map(move |&dy| [dx, dy]))
        .filter(|p| *p != [0.0, 0.0])
        .collect();

    Problem {
        reach: affine::Affine::contracting(0.8, 0.01, 0.02),
        obstacles: Obstacles::new(walls.iter().map(Polytope::from_box).collect(), 4),
        start: [1.0, 1.0],
        start_set: SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&rect(
            0.9, 1.1, 0.9, 1.1,
        ))),
        goal: PolytopeGoal::from_box(&rect(8.5, 9.5, 8.5, 9.5), 8),
        primitives,
        bounds: None,
    }
}

/// Solves the slalom with A* (goal radius 0.75).
fn bench_slalom(criterion: &mut Criterion) {
    let config = Config {
        goal_radius: GoalRadius::FromGoal,
        ..Config::default()
    };

    criterion.bench_function("reach_search_slalom", |b| {
        b.iter(|| {
            let mut search = ReachSearch::new(problem(), config).expect("valid problem");
            black_box(search.solve())
        });
    });
}
