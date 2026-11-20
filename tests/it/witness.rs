//! Certified collisions: `Unsafe` carries a witness only when the model certifies one.

use polyzonotope::{Interval, IntervalBox, Polytope, SparsePolyZonotope, Zonotope};
use robust_planning::{
    Obstacles, Reach, ReachChecker, SafetyVerdict, WithSteering, Witness, check_edge,
};
use sampling_planning::{Euclidean, StraightLine, Validity, ValidityChecker};

/// The box `[lo, hi]²` as a set.
fn square(lo: f64, hi: f64) -> SparsePolyZonotope {
    SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&IntervalBox(vec![
        Interval::new(lo, hi);
        2
    ])))
}

/// The obstacle `[0, 3]²`, which contains every cover set of [`Overlap`].
fn obstacles() -> Obstacles {
    let far = Polytope::from_box(&IntervalBox(vec![Interval::new(20.0, 30.0); 2]));
    let near = Polytope::from_box(&IntervalBox(vec![Interval::new(0.0, 3.0); 2]));

    Obstacles::new(vec![far, near], 4)
}

/// A model whose every edge is covered by the box `[1, 2]²`, which lies inside an obstacle,
/// and whose collision hook returns `witness` for every set and obstacle.
struct Overlap {
    /// What [`Reach::certify_collision`] returns; `None` keeps the default behavior.
    witness: Option<Vec<f64>>,
}

impl Reach<2> for Overlap {
    /// The end point of the edge.
    type Control = [f64; 2];

    fn propagate(
        &self,
        initial: &SparsePolyZonotope,
        _: &[f64; 2],
        cover: &mut Vec<SparsePolyZonotope>,
    ) -> SparsePolyZonotope {
        cover.push(square(1.0, 2.0));

        initial.clone()
    }

    fn certify_collision(
        &self,
        _: &SparsePolyZonotope,
        _: &[f64; 2],
        _: &SparsePolyZonotope,
        _: &Polytope,
    ) -> Option<Vec<f64>> {
        self.witness.clone()
    }
}

/// Checks one edge of `model` from the box `[1, 2]²`.
fn verdict(model: &impl Reach<2, Control = [f64; 2]>) -> SafetyVerdict {
    check_edge(
        model,
        &obstacles(),
        &square(1.0, 2.0),
        &[1.5, 1.5],
        &mut Vec::new(),
    )
    .0
}

#[test]
fn overlap_without_a_witness_is_indeterminate() {
    /// [`Overlap`] without a collision hook, so it uses the default.
    struct NoHook;

    impl Reach<2> for NoHook {
        /// The end point of the edge.
        type Control = [f64; 2];

        fn propagate(
            &self,
            initial: &SparsePolyZonotope,
            _: &[f64; 2],
            cover: &mut Vec<SparsePolyZonotope>,
        ) -> SparsePolyZonotope {
            cover.push(square(1.0, 2.0));

            initial.clone()
        }
    }

    assert_eq!(verdict(&NoHook), SafetyVerdict::Indeterminate);
    assert_eq!(
        verdict(&Overlap { witness: None }),
        SafetyVerdict::Indeterminate
    );
}

#[test]
fn certified_witness_makes_the_edge_unsafe() {
    let model = Overlap {
        witness: Some(vec![1.5, 1.5]),
    };

    assert_eq!(
        verdict(&model),
        SafetyVerdict::Unsafe(Witness {
            point: vec![1.5, 1.5],
            set: 0,
            obstacle: 1,
        }),
        "the far obstacle rejects the point, the near one accepts it"
    );
}

#[test]
fn implausible_witness_is_dropped() {
    for point in [vec![5.0, 5.0], vec![2.5, 2.5], vec![1.5]] {
        let model = Overlap {
            witness: Some(point.clone()),
        };

        assert_eq!(
            verdict(&model),
            SafetyVerdict::Indeterminate,
            "{point:?} is outside the obstacle, outside the set or of the wrong dimension"
        );
    }
}

#[test]
fn safe_edges_never_become_unsafe() {
    let model = Overlap {
        witness: Some(vec![1.5, 1.5]),
    };
    let clear = Obstacles::new(
        vec![Polytope::from_box(&IntervalBox(vec![
            Interval::new(
                5.0, 6.0
            );
            2
        ]))],
        4,
    );
    let (verdict, _) = check_edge(
        &model,
        &clear,
        &square(1.0, 2.0),
        &[1.5, 1.5],
        &mut Vec::new(),
    );

    assert_eq!(verdict, SafetyVerdict::Safe);
}

#[test]
fn checker_reports_a_certified_collision_as_invalid_through_with_steering() {
    let checker = |witness| {
        ReachChecker::new(
            obstacles(),
            WithSteering::new(Overlap { witness }, StraightLine::new(10.0)),
            [0.5, 0.5],
        )
    };
    let (from, to) = ([1.5, 1.5], [1.6, 1.5]);

    assert_eq!(
        checker(Some(vec![1.5, 1.5])).check_motion(&Euclidean::<2>, &from, &to),
        Validity::Invalid
    );
    assert_eq!(
        checker(None).check_motion(&Euclidean::<2>, &from, &to),
        Validity::Unknown
    );
}
