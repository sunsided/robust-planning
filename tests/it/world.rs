//! 2D test worlds: box obstacles, start and goal boxes, and a point-robot checker.

use nalgebra::DVector;
use polyzonotope::{Interval, IntervalBox, Polytope, SparsePolyZonotope, Zonotope};
use robust_planning::{
    Obstacles, PolytopeGoal,
    robust_rrt::{Config, Problem, RobustRrt},
    robust_rrt_star::{Config as StarConfig, RobustRrtStar},
};
use sampling_planning::{Euclidean, UniformBox, Validity, ValidityChecker, discretized_motion};

use crate::affine::Affine;

/// An axis-aligned box given by its lower and upper corners.
pub type Rect = ([f64; 2], [f64; 2]);

/// The Robust-RRT type all tests use.
pub type Planner = RobustRrt<Affine<2>, UniformBox<2>, 2>;

/// Split budget per set and obstacle in all tests.
pub const SPLITS: usize = 4;

/// Two walls that force a slalom from (1, 1) to (9, 9) in the 10 x 10 world.
pub const SLALOM: [Rect; 2] = [([4.0, 0.0], [5.0, 7.0]), ([6.0, 3.0], [7.0, 10.0])];

/// The interval box of `rect`.
pub fn interval_box(rect: &Rect) -> IntervalBox {
    IntervalBox(
        (0..2)
            .map(|i| Interval::new(rect.0[i], rect.1[i]))
            .collect(),
    )
}

/// The polytopes of `rects`.
pub fn polytopes(rects: &[Rect]) -> Vec<Polytope> {
    rects
        .iter()
        .map(|r| Polytope::from_box(&interval_box(r)))
        .collect()
}

/// The box `center ± half_width` in both axes.
pub fn square(center: [f64; 2], half_width: f64) -> Rect {
    (
        [center[0] - half_width, center[1] - half_width],
        [center[0] + half_width, center[1] + half_width],
    )
}

/// A wall at `x ∈ [4.5, 5.5]` across the whole world except for a gap of width `gap`
/// centered at `y = 5`.
pub fn wall_with_gap(gap: f64) -> Vec<Rect> {
    let half = gap / 2.0;

    vec![
        ([4.5, -1.0], [5.5, 5.0 - half]),
        ([4.5, 5.0 + half], [5.5, 11.0]),
    ]
}

/// The contracting model used by the tests: errors shrink by 0.8 per step, input error 0.01
/// and disturbance 0.02 per axis, so the error box settles at a half width of 0.15.
pub fn model() -> Affine<2> {
    Affine::contracting(0.8, 0.01, 0.02)
}

/// A Robust-RRT in the 10 x 10 world around `obstacles`, from a start box of half width 0.1
/// around `start` to a goal box of half width 0.5 around `goal`.
pub fn planner(obstacles: &[Rect], start: [f64; 2], goal: [f64; 2]) -> Planner {
    let start_set =
        SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&interval_box(&square(start, 0.1))));
    let problem = Problem {
        reach: model(),
        obstacles: Obstacles::new(polytopes(obstacles), SPLITS),
        sampler: UniformBox::new([0.0, 0.0], [10.0, 10.0]),
        start,
        start_set,
        goal: PolytopeGoal::new(
            Polytope::from_box(&interval_box(&square(goal, 0.5))),
            goal,
            8,
        ),
    };

    RobustRrt::new(problem, Config::default()).expect("the start set is safe")
}

/// Whether `point` lies in any of `obstacles`.
pub fn hits(obstacles: &[Polytope], point: &[f64; 2]) -> bool {
    let point = DVector::from_column_slice(point);

    obstacles.iter().any(|o| o.contains(&point, 0.0))
}

/// A point robot without uncertainty, checked at a fine resolution.
pub struct PointChecker {
    /// The obstacles.
    pub obstacles: Vec<Polytope>,
}

impl ValidityChecker<Euclidean<2>> for PointChecker {
    fn check_state(&self, state: &[f64; 2]) -> Validity {
        if hits(&self.obstacles, state) {
            Validity::Invalid
        } else {
            Validity::Valid
        }
    }

    fn check_motion(&self, space: &Euclidean<2>, from: &[f64; 2], to: &[f64; 2]) -> Validity {
        discretized_motion(space, from, to, 0.01, |state| self.check_state(state))
    }
}

/// The Robust RRT* type the tests use.
pub type StarPlanner = RobustRrtStar<Affine<2>, UniformBox<2>, 2>;

/// A Robust RRT* on the same world as [`planner`], with `gamma = 10` (above the RRT* bound
/// of about 9.8 for the 10 x 10 world) and the radius capped at the model's 1.0 reach.
pub fn star_planner(obstacles: &[Rect], start: [f64; 2], goal: [f64; 2]) -> StarPlanner {
    let problem = planner(obstacles, start, goal).problem().clone();

    RobustRrtStar::new(problem, StarConfig::new(10.0, 1.0)).expect("the start set is safe")
}
