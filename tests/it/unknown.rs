//! Checks whose sets the obstacle test cannot decide without refinement.

use nalgebra::{DMatrix, DVector};
use polyzonotope::{Interval, IntervalBox, Polytope, SparsePolyZonotope, Zonotope};
use rand_pcg::Pcg64Mcg;
use robust_planning::{
    Obstacles, PolytopeGoal, Reach, ReachChecker, WithSteering,
    robust_rrt::{Config, Problem, RobustRrt},
};
use sampling_planning::{
    Euclidean, StraightLine, UniformBox, Validity, ValidityChecker, rand_core::SeedableRng,
};

/// A model whose every edge is covered by the diamond `|x| + |y| <= 1` and ends at a point.
///
/// The diamond is disjoint from [`corner`], but proving it needs a split, so with a split
/// budget of zero every edge check is [`Intersection::Unknown`](polyzonotope::Intersection).
/// It only propagates; [`model`] pairs it with straight-line steering.
struct Diamond;

impl Reach<2> for Diamond {
    /// The end point of the edge.
    type Control = [f64; 2];

    fn propagate(
        &self,
        _: &SparsePolyZonotope,
        control: &[f64; 2],
        cover: &mut Vec<SparsePolyZonotope>,
    ) -> SparsePolyZonotope {
        let generators = DMatrix::from_column_slice(2, 2, &[0.5, 0.5, 0.5, -0.5]);
        cover.push(SparsePolyZonotope::from_zonotope(&Zonotope::new(
            DVector::zeros(2),
            generators,
        )));

        let end = Zonotope::new(DVector::from_column_slice(control), DMatrix::zeros(2, 0));

        SparsePolyZonotope::from_zonotope(&end)
    }
}

/// [`Diamond`] steered in a straight line, with steps long enough to reach every target.
fn model() -> WithSteering<Diamond, StraightLine> {
    WithSteering::new(Diamond, StraightLine::new(100.0))
}

/// The obstacle `[0.6, 2]²` near the diamond's upper right edge.
fn corner() -> Polytope {
    Polytope::from_box(&IntervalBox(vec![Interval::new(0.6, 2.0); 2]))
}

/// A checker around [`Diamond`] with the given split budget and a tube of half width 0.1.
fn checker(max_splits: usize) -> ReachChecker<WithSteering<Diamond, StraightLine>, 2> {
    ReachChecker::new(
        Obstacles::new(vec![corner()], max_splits),
        model(),
        [0.1, 0.1],
    )
}

#[test]
fn unknown_motion_is_rejected_and_refinement_accepts_it() {
    let (from, to) = ([-3.0, -3.0], [-2.0, -3.0]);

    assert_eq!(checker(0).check_state(&from), Validity::Valid);
    assert_eq!(
        checker(0).check_motion(&Euclidean::<2>, &from, &to),
        Validity::Unknown
    );
    assert_eq!(
        checker(4).check_motion(&Euclidean::<2>, &from, &to),
        Validity::Valid
    );
}

#[test]
fn robust_rrt_adds_no_edge_on_unknown() {
    let start = [-3.0, -3.0];
    let problem = Problem {
        reach: model(),
        obstacles: Obstacles::new(vec![corner()], 0),
        sampler: UniformBox::new([-5.0, -5.0], [-1.0, -1.0]),
        start,
        start_set: SparsePolyZonotope::from_zonotope(&Zonotope::new(
            DVector::from_column_slice(&start),
            DMatrix::zeros(2, 0),
        )),
        goal: PolytopeGoal::from_box(&IntervalBox(vec![Interval::new(-2.0, -1.0); 2]), 8),
    };
    let mut planner = RobustRrt::new(problem, Config::default()).expect("start is safe");
    let mut rng = Pcg64Mcg::seed_from_u64(5);

    for _ in 0..50 {
        assert!(!planner.step(&mut rng), "an unknown edge was accepted");
    }

    assert_eq!(planner.nodes().len(), 1);
    assert!(planner.path().is_none());
}

#[test]
fn steering_that_stops_short_is_unknown_in_the_checker() {
    let (from, to) = ([-3.0, -3.0], [-2.0, -3.0]);
    let short = ReachChecker::new(
        Obstacles::new(vec![corner()], 4),
        WithSteering::new(Diamond, StraightLine::new(0.5)),
        [0.1, 0.1],
    );

    assert_eq!(
        short.check_motion(&Euclidean::<2>, &from, &to),
        Validity::Unknown,
        "the motion ends short of `to`, so the tube at `to` is not reached"
    );
    assert_eq!(
        checker(4).check_motion(&Euclidean::<2>, &from, &to),
        Validity::Valid
    );
}
