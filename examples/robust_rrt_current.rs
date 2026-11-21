//! Robust-RRT on validated continuous-time reachable sets of a nonlinear system.
//!
//! The robot moves with velocity input `u` through a current that pushes it along `y`,
//! stronger the further it is along `x`: `x' = u₀`, `y' = u₁ + k x²`. Each input carries a
//! bounded error. The model is [`Nonlinear`] on polyzonotope-reach, so every cover set
//! encloses the robot's continuous motion during one time step, not only the states at the
//! step boundaries. The steering compensates the current along a straight nominal path, but
//! not its spread: the robot's uncertainty along `x` turns into a growing spread along `y`.
//!
//! Run: `cargo run --example robust_rrt_current --features reach` (add `sound` for rigorous
//! proofs).

use nalgebra::{DMatrix, DVector};
use polyzonotope::{Interval, IntervalBox, Polytope, SparsePolyZonotope, Zonotope};
use polyzonotope_reach::{NonlinearSettings, Polynomial};
use rand_pcg::Pcg64Mcg;
use robust_planning::{
    Obstacles, PolytopeGoal, Steer, WithSteering,
    models::Nonlinear,
    robust_rrt::{Config, Problem, RobustRrt},
};
use sampling_planning::{UniformBox, rand_core::SeedableRng};

/// Strength `k` of the current `k x²` along `y`.
const CURRENT: f64 = 0.02;

/// Length of one time step.
const STEP: f64 = 0.25;

/// Half width of the error box around every nominal input, per axis.
const INPUT_ERROR: f64 = 0.005;

/// Straight-line steering that cancels the current along the nominal path.
struct Compensate {
    /// Nominal speed along the path.
    speed: f64,
    /// Most time steps per edge. Positive.
    max_steps: u32,
}

impl Steer<2, Vec<Zonotope>> for Compensate {
    /// Moves toward `toward` at [`Compensate::speed`] for at most
    /// [`Compensate::max_steps`] steps, one input set per step.
    fn steer(&self, from: &[f64; 2], toward: &[f64; 2]) -> (Vec<Zonotope>, [f64; 2]) {
        let (dx, dy) = (toward[0] - from[0], toward[1] - from[1]);
        let distance = dx.hypot(dy);
        let steps = steps_for(distance / (self.speed * STEP), self.max_steps);
        let duration = f64::from(steps) * STEP;
        let scale = (self.speed * duration / distance).min(1.0);
        let end = [from[0] + dx * scale, from[1] + dy * scale];
        let velocity = [(end[0] - from[0]) / duration, (end[1] - from[1]) / duration];

        let inputs = (0..steps)
            .map(|k| {
                let a = from[0] + velocity[0] * STEP * f64::from(k);
                let b = a + velocity[0] * STEP;
                // The mean of `x²` over the step, for `x` moving linearly from `a` to `b`.
                let push = CURRENT * (a * a + a * b + b * b) / 3.0;

                input_set([velocity[0], velocity[1] - push])
            })
            .collect();

        (inputs, end)
    }
}

/// The number of steps for `ratio` step lengths: at least one, at most `max_steps`.
fn steps_for(ratio: f64, max_steps: u32) -> u32 {
    (1..=max_steps)
        .find(|&k| f64::from(k) >= ratio)
        .unwrap_or(max_steps)
}

/// The nominal input `u` with its error box.
fn input_set(u: [f64; 2]) -> Zonotope {
    Zonotope::new(
        DVector::from_column_slice(&u),
        DMatrix::from_diagonal_element(2, 2, INPUT_ERROR),
    )
}

/// `x' = u₀`, `y' = u₁ + k x²` over the variables `(x, y, u₀, u₁)`.
fn current() -> Polynomial {
    Polynomial::new(2, 2)
        .term(0, 1.0, &[0, 0, 1, 0])
        .term(1, 1.0, &[0, 0, 0, 1])
        .term(1, CURRENT, &[2, 0, 0, 0])
}

/// The box `[x0, x1] x [y0, y1]`.
fn rect(x0: f64, x1: f64, y0: f64, y1: f64) -> IntervalBox {
    IntervalBox(vec![Interval::new(x0, x1), Interval::new(y0, y1)])
}

fn main() {
    let walls = [rect(4.0, 5.0, 0.0, 7.0), rect(6.0, 7.0, 3.0, 10.0)];
    let start = [1.0, 1.0];
    let start_box = rect(0.98, 1.02, 0.98, 1.02);
    let model = Nonlinear::new(current(), NonlinearSettings::new(STEP));
    let steering = Compensate {
        speed: 1.0,
        max_steps: 4,
    };
    let problem = Problem {
        reach: WithSteering::new(model, steering),
        obstacles: Obstacles::new(walls.iter().map(Polytope::from_box).collect(), 4),
        sampler: UniformBox::new([0.0, 0.0], [10.0, 10.0]),
        start,
        start_set: SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&start_box)),
        goal: PolytopeGoal::from_box(&rect(8.0, 10.0, 8.0, 10.0), 8),
    };

    let mut planner = RobustRrt::new(problem, Config::default()).expect("the start set is safe");
    let path = planner.solve(3000, &mut Pcg64Mcg::seed_from_u64(7));

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
