//! Robust-RRT in a 2D world with two walls.
//!
//! The robot follows `x' = 0.8 x + u + e + w` per time step, with input error `e` and
//! disturbance `w` bounded per axis. Each tree node carries the reachable set the robot can be
//! in when it gets there, and an edge is kept only if the set is proven disjoint from both
//! walls at every time step. The goal counts as reached when the whole set is proven inside
//! the goal box.
//!
//! Run: `cargo run --example robust_rrt_2d` (add `--features sound` for rigorous proofs).

#[path = "../tests/it/affine.rs"]
mod affine;

use polyzonotope::{Interval, IntervalBox, Polytope, SparsePolyZonotope, Zonotope};
use rand_pcg::Pcg64Mcg;
use robust_planning::{
    Obstacles,
    robust_rrt::{Config, Problem, RobustRrt},
};
use sampling_planning::{UniformBox, rand_core::SeedableRng};

/// The box `[x0, x1] x [y0, y1]`.
fn rect(x0: f64, x1: f64, y0: f64, y1: f64) -> IntervalBox {
    IntervalBox(vec![Interval::new(x0, x1), Interval::new(y0, y1)])
}

fn main() {
    let walls = [rect(4.0, 5.0, 0.0, 7.0), rect(6.0, 7.0, 3.0, 10.0)];
    let start = [1.0, 1.0];
    let start_box = rect(0.9, 1.1, 0.9, 1.1);
    let problem = Problem {
        reach: affine::Affine::contracting(0.8, 0.01, 0.02),
        obstacles: Obstacles::new(walls.iter().map(Polytope::from_box).collect(), 4),
        sampler: UniformBox::new([0.0, 0.0], [10.0, 10.0]),
        start,
        start_set: SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&start_box)),
        goal: Polytope::from_box(&rect(8.5, 9.5, 8.5, 9.5)),
        goal_center: [9.0, 9.0],
    };

    let mut planner = RobustRrt::new(problem, Config::default()).expect("the start set is safe");
    let path = planner.solve(5000, &mut Pcg64Mcg::seed_from_u64(7));

    println!("{} nodes", planner.nodes().len());

    let Some(path) = path else {
        println!("no proven-safe path found");
        return;
    };

    println!("path of nominal length {:.2}:", path.cost);
    let goal = planner.goal_node().expect("a path ends at the goal node");

    for id in planner.branch(goal) {
        let node = &planner.nodes()[id];
        let hull = node.set.interval_hull();

        println!(
            "  ({:5.2}, {:5.2})  set {:.3} x {:.3}",
            node.nominal[0],
            node.nominal[1],
            hull.0[0].width(),
            hull.0[1].width()
        );
    }
}
