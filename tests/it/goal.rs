//! Custom goals: only a proven containment reaches the goal.

use polyzonotope::{Containment, SparsePolyZonotope, Zonotope};
use rand_pcg::Pcg64Mcg;
use robust_planning::{
    Goal, Obstacles,
    robust_rrt::{Config, Problem, RobustRrt},
};
use sampling_planning::{UniformBox, rand_core::SeedableRng};

use crate::world::{SLALOM, SPLITS, interval_box, model, polytopes, square};

/// A goal around (9, 9) that gives the same answer for every set.
struct Fixed(Containment);

impl Goal<2> for Fixed {
    fn center(&self) -> [f64; 2] {
        [9.0, 9.0]
    }

    fn containment(&self, _: &SparsePolyZonotope) -> Containment {
        self.0
    }
}

/// Runs a Robust-RRT on the slalom toward `goal` and returns it.
fn run(goal: Fixed) -> RobustRrt<crate::affine::Affine<2>, UniformBox<2>, 2, Fixed> {
    let start = [1.0, 1.0];
    let problem = Problem {
        reach: model(),
        obstacles: Obstacles::new(polytopes(&SLALOM), SPLITS),
        sampler: UniformBox::new([0.0, 0.0], [10.0, 10.0]),
        start,
        start_set: SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&interval_box(&square(
            start, 0.1,
        )))),
        goal,
    };
    let mut planner = RobustRrt::new(problem, Config::default()).expect("the start set is safe");
    planner.solve(1500, &mut Pcg64Mcg::seed_from_u64(2));

    planner
}

#[test]
fn unproven_containment_never_reaches_the_goal() {
    for answer in [Containment::Unknown, Containment::NotInside] {
        let planner = run(Fixed(answer));

        assert!(planner.goal_node().is_none(), "{answer:?} reached the goal");
        assert!(planner.path().is_none());
        assert!(
            planner.nodes().iter().any(|n| n.nominal[0] > 8.0),
            "the tree gets near the goal, so the goal test was asked"
        );
    }
}

#[test]
fn proven_containment_reaches_the_goal_at_the_root() {
    let planner = run(Fixed(Containment::Inside));

    assert_eq!(planner.goal_node(), Some(0));
    assert_eq!(
        planner.nodes().len(),
        1,
        "solve stops at the first goal node"
    );
}
