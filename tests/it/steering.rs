//! Custom steering: the planner proves every edge, whatever the steering function picks.

use polyzonotope::{Intersection, SparsePolyZonotope, Zonotope};
use rand_pcg::Pcg64Mcg;
use robust_planning::{
    Obstacles, PolytopeGoal, Reach, Steer, WithSteering,
    robust_rrt::{Config, Problem, RobustRrt},
};
use sampling_planning::{UniformBox, rand_core::SeedableRng};

use crate::{
    affine::Affine,
    world::{SLALOM, SPLITS, interval_box, model, polytopes, square},
};

/// A steering function that ignores the target and always aims at a point inside the first
/// slalom wall, using the affine model's own steering to get there.
struct IntoWall(Affine<2>);

impl Steer<2, Vec<[f64; 2]>> for IntoWall {
    /// The affine model's steering toward `(4.5, 3.5)`, inside the wall `[4, 5] x [0, 7]`.
    fn steer(&self, from: &[f64; 2], _: &[f64; 2]) -> (Vec<[f64; 2]>, [f64; 2]) {
        self.0.steer(from, &[4.5, 3.5])
    }
}

#[test]
fn edges_from_a_steering_into_a_wall_are_still_proven_disjoint() {
    let start = [1.0, 1.0];
    let problem = Problem {
        reach: WithSteering::new(model(), IntoWall(model())),
        obstacles: Obstacles::new(polytopes(&SLALOM), SPLITS),
        sampler: UniformBox::new([0.0, 0.0], [10.0, 10.0]),
        start,
        start_set: SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&interval_box(&square(
            start, 0.1,
        )))),
        goal: PolytopeGoal::from_box(&interval_box(&square([9.0, 9.0], 0.5)), 8),
    };
    let mut planner = RobustRrt::new(problem, Config::default()).expect("the start set is safe");
    let mut rng = Pcg64Mcg::seed_from_u64(11);

    for _ in 0..200 {
        planner.step(&mut rng);
    }

    assert!(
        planner.nodes().len() > 1,
        "edges short of the wall are accepted"
    );
    assert!(
        planner.nodes().len() < 201,
        "edges into the wall are rejected"
    );
    assert!(planner.path().is_none(), "the goal is never steered toward");

    let reach = &planner.problem().reach;
    let obstacles = polytopes(&SLALOM);
    let mut cover = Vec::new();

    for node in &planner.nodes()[1..] {
        let parent = &planner.nodes()[node.parent.expect("non-root")];

        cover.clear();
        reach.propagate(
            &parent.set,
            node.control.as_ref().expect("non-root"),
            &mut cover,
        );

        for set in &cover {
            for obstacle in &obstacles {
                assert_eq!(set.intersection(obstacle, SPLITS), Intersection::Disjoint);
            }
        }
    }
}
