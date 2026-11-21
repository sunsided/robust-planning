//! Property tests for the verdict lattice, the edge costs and proven containment.
//!
//! The planner tests run on a few fixed worlds; these cover the building blocks the planners
//! trust over their whole domain instead. `task test:prop` raises the case count.

use nalgebra::{DMatrix, DVector};
use polyzonotope::{Interval, IntervalBox, SparsePolyZonotope, Zonotope};
use proptest::prelude::*;
use robust_planning::{Cost, SafetyVerdict, Witness, decide_within};

/// Splitting budget of every containment check.
const SPLITS: usize = 4;

/// Any verdict; unsafe ones name obstacle `0..4` so that witnesses can be told apart.
fn verdict() -> impl Strategy<Value = SafetyVerdict> {
    prop_oneof![
        Just(SafetyVerdict::Safe),
        Just(SafetyVerdict::Indeterminate),
        (0..4_usize).prop_map(|obstacle| SafetyVerdict::Unsafe(Witness {
            point: vec![0.0, 0.0],
            set: 0,
            obstacle,
        })),
    ]
}

/// The rank of a verdict in the lattice: Safe below Indeterminate below Unsafe.
fn rank(verdict: &SafetyVerdict) -> u8 {
    match verdict {
        SafetyVerdict::Safe => 0,
        SafetyVerdict::Indeterminate => 1,
        SafetyVerdict::Unsafe(_) => 2,
    }
}

/// A box in `[-10, 10]^dim` with sides up to 4, possibly flat.
fn interval_box(dim: usize) -> impl Strategy<Value = IntervalBox> {
    prop::collection::vec((-10.0..10.0_f64, 0.0..4.0_f64), dim).prop_map(|sides| {
        IntervalBox(
            sides
                .into_iter()
                .map(|(lo, width)| Interval::new(lo, lo + width))
                .collect(),
        )
    })
}

/// A zonotope in `dim` dimensions with up to `max_generators` generators in `[-2, 2]^dim`
/// around a center in `[-3, 3]^dim`.
fn zonotope(dim: usize, max_generators: usize) -> impl Strategy<Value = Zonotope> {
    (
        prop::collection::vec(-3.0..3.0_f64, dim),
        prop::collection::vec(
            prop::collection::vec(-2.0..2.0_f64, dim),
            0..=max_generators,
        ),
    )
        .prop_map(move |(center, generators)| {
            let g = DMatrix::from_fn(dim, generators.len(), |i, j| generators[j][i]);

            Zonotope::new(DVector::from_vec(center), g)
        })
}

/// A dimension, then an inner and an outer zonotope in it. Half the inner sets are the
/// outer one scaled around its center by up to 1.2, so that proofs are common and some
/// scaled sets stick out just a little.
fn zonotope_pair() -> impl Strategy<Value = (Zonotope, Zonotope)> {
    (2..=3_usize).prop_flat_map(|dim| {
        prop_oneof![
            (zonotope(dim, 4), zonotope(dim, 6)),
            (zonotope(dim, 6), 0.0..1.2_f64).prop_map(|(outer, factor)| {
                let inner = Zonotope::new(outer.center().clone(), outer.generators() * factor);

                (inner, outer)
            }),
        ]
    })
}

/// The support function `max dᵀx` of `z` along `d`, and the magnitude of its terms.
fn support(z: &Zonotope, d: &DVector<f64>) -> (f64, f64) {
    let terms = core::iter::once(d.dot(z.center()))
        .chain(z.generators().column_iter().map(|g| d.dot(&g).abs()));

    terms.fold((0.0, 0.0), |(sum, scale), t| (sum + t, scale + t.abs()))
}

/// The directions supports are compared along: both signs of every axis and the diagonals.
fn directions(dim: usize) -> Vec<DVector<f64>> {
    let axes = (0..dim).flat_map(|i| {
        [1.0, -1.0].map(|sign| DVector::from_fn(dim, |k, _| if k == i { sign } else { 0.0 }))
    });
    let diagonals = (0..1_u32 << dim)
        .map(|signs| DVector::from_fn(dim, |k, _| if signs >> k & 1 == 1 { -1.0 } else { 1.0 }));

    axes.chain(diagonals).collect()
}

/// `b` shrunk by `factor` around its midpoint.
fn shrunk(b: &IntervalBox, factor: f64) -> IntervalBox {
    IntervalBox(
        b.0.iter()
            .map(|r| {
                let (mid, radius) = (f64::midpoint(r.lo, r.hi), 0.5 * (r.hi - r.lo) * factor);

                Interval::new(mid - radius, mid + radius)
            })
            .collect(),
    )
}

/// The box `b` as a zonotope in sparse form.
fn box_set(b: &IntervalBox) -> SparsePolyZonotope {
    SparsePolyZonotope::from_zonotope(&Zonotope::from_box(b))
}

proptest! {
    #[test]
    fn and_takes_the_worse_verdict_and_is_associative(
        a in verdict(),
        b in verdict(),
        c in verdict(),
    ) {
        let ab = a.clone().and(b.clone());

        prop_assert_eq!(rank(&ab), rank(&a).max(rank(&b)));
        prop_assert_eq!(ab.is_safe(), a.is_safe() && b.is_safe());
        prop_assert_eq!(
            ab.clone().and(c.clone()),
            a.clone().and(b.clone().and(c)),
            "associative, witnesses included"
        );

        if let SafetyVerdict::Unsafe(witness) = &a {
            prop_assert_eq!(&ab, &SafetyVerdict::Unsafe(witness.clone()), "left witness wins");
        }
    }

    #[test]
    fn edge_costs_never_undercut_the_length_and_grow_with_the_weight(
        b in interval_box(2),
        length in 0.0..100.0_f64,
        weight in 0.0..10.0_f64,
        extra in 0.0..10.0_f64,
    ) {
        let set = box_set(&b);
        let lighter = Cost::WidthWeighted { weight };
        let heavier = Cost::WidthWeighted { weight: weight + extra };

        prop_assert!(lighter.is_valid() && heavier.is_valid());
        prop_assert_eq!(Cost::Length.edge(length, &set).to_bits(), length.to_bits());
        prop_assert!(lighter.edge(length, &set) >= length);
        prop_assert!(heavier.edge(length, &set) >= lighter.edge(length, &set));
    }

    #[test]
    fn a_proof_of_containment_holds_along_every_sampled_direction(
        (inner, outer) in zonotope_pair(),
        tolerance in prop_oneof![Just(0.0), 0.0..0.5_f64],
    ) {
        let decided = decide_within(
            &SparsePolyZonotope::from_zonotope(&inner),
            &SparsePolyZonotope::from_zonotope(&outer),
            tolerance,
            SPLITS,
        );

        if decided == Some(true) {
            for d in directions(inner.dim()) {
                let (inside, inner_scale) = support(&inner, &d);
                let (bound, outer_scale) = support(&outer, &d);
                let bound = bound + tolerance * d.lp_norm(1);
                let slack = 1e-9 * (1.0 + inner_scale + outer_scale);

                prop_assert!(inside <= bound + slack, "sticks out along {d:?}: {inside} > {bound}");
            }
        }
    }

    #[test]
    fn a_box_shrunk_inside_a_full_box_is_proven_inside(
        b in interval_box(3).prop_filter("full-dimensional", |b| b.0.iter().all(|r| r.hi - r.lo > 1e-3)),
        factor in 0.0..0.99_f64,
        planar in any::<bool>(),
    ) {
        let b = if planar { IntervalBox(b.0[..2].to_vec()) } else { b };

        prop_assert_eq!(
            decide_within(&box_set(&shrunk(&b, factor)), &box_set(&b), 0.0, SPLITS),
            Some(true)
        );
    }
}
