//! Tests of proven containment: hand-checked planar cases, the planar and the general
//! facets cross-checked in 2D, and sampling in 3D.

use nalgebra::DMatrix;
use polyzonotope::{Interval, IntervalBox, Zonotope};

use super::*;

/// The zonotope with `center` and generator columns `generators`, in sparse form.
fn zonotope(center: [f64; 2], generators: &[[f64; 2]]) -> SparsePolyZonotope {
    let g = DMatrix::from_fn(2, generators.len(), |i, j| generators[j][i]);

    SparsePolyZonotope::from_zonotope(&Zonotope::new(DVector::from_column_slice(&center), g))
}

/// The box `center ± half_width` as a set.
fn square(center: [f64; 2], half_width: f64) -> SparsePolyZonotope {
    let b = IntervalBox(
        center
            .iter()
            .map(|c| Interval::new(c - half_width, c + half_width))
            .collect(),
    );

    SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&b))
}

#[test]
fn a_smaller_box_is_inside_a_rotated_square() {
    // The diamond |x| + |y| <= 1.
    let diamond = zonotope([0.0, 0.0], &[[0.5, 0.5], [0.5, -0.5]]);

    assert!(proven_within(&square([0.0, 0.0], 0.45), &diamond, 4));
    assert!(proven_within(&square([0.2, 0.0], 0.3), &diamond, 4));
}

#[test]
fn a_box_whose_hull_fits_but_corners_stick_out_is_not_inside() {
    // The diamond's hull is [-1, 1]², which contains the box; the box's corners
    // (0.6, 0.6) are outside the diamond. The hull test would call it inside.
    let diamond = zonotope([0.0, 0.0], &[[0.5, 0.5], [0.5, -0.5]]);

    assert!(!proven_within(&square([0.0, 0.0], 0.6), &diamond, 8));
}

#[test]
fn equal_representations_are_inside_but_touching_sets_are_not() {
    let a = square([0.3, 0.7], 0.25);
    let swapped = zonotope([0.3, 0.7], &[[0.0, 0.25], [0.25, 0.0]]);

    assert!(
        proven_within(&a, &square([0.3, 0.7], 0.25), 8),
        "same entries"
    );
    // The same set with its generators swapped: rounding the offsets down makes the
    // boundary strict, so the touching set is not proven inside.
    assert!(!proven_within(&swapped, &a, 8));
    assert!(proven_within(&square([0.3, 0.7], 0.2499), &a, 8));
}

#[test]
fn a_segment_bounds_its_own_direction() {
    let segment = zonotope([0.0, 0.0], &[[1.0, 1.0]]);
    let point_far_along = zonotope([3.0, 3.0], &[[1e-3, 0.0]]);

    assert!(!proven_within(&point_far_along, &segment, 8));
}

#[test]
fn shared_factors_and_other_dimensions_are_not_decided() {
    let shared = SparsePolyZonotope::new(
        DVector::from_column_slice(&[0.0, 0.0]),
        DMatrix::from_column_slice(2, 2, &[1.0, 0.0, 0.0, 1.0]),
        DMatrix::zeros(2, 0),
        DMatrix::from_column_slice(1, 2, &[1, 1]),
        vec![polyzonotope::FactorId::fresh()],
    )
    .expect("valid shape");
    let _cube = SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&IntervalBox(vec![
        Interval::new(-1.0, 1.0);
        3
    ])));

    assert!(!is_zonotope(&shared), "one factor drives two generators");
    assert!(!proven_within(&square([0.0, 0.0], 0.01), &shared, 8));
    let hypercube = SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&IntervalBox(vec![
    Interval::new(-1.0, 1.0);
    5
])));
    assert_eq!(
        decide_within(&hypercube, &hypercube.with_fresh_ids(), 0.0, 8),
        Some(true)
    );
    assert_eq!(
        decide_within(
            &hypercube.linear_map(&DMatrix::from_diagonal_element(5, 5, 0.5)),
            &hypercube,
            0.0,
            8
        ),
        None,
        "five dimensions are not decided"
    );
}

#[test]
fn non_finite_outer_sets_are_not_decided() {
    let huge = zonotope([0.0, 0.0], &[[1e308, 1e308], [1e308, -1e308]]);
    let nan = zonotope([0.0, 0.0], &[[f64::NAN, 1.0], [1.0, 0.0]]);
    let small = square([0.0, 0.0], 0.1);

    assert!(
        !proven_within(&small, &huge, 4),
        "overflowing offsets cut nothing"
    );
    assert!(
        !proven_within(&small, &nan, 4),
        "NaN facets are never refuted"
    );
}

/// Decides `inner ⊆ outer` like [`decide_within`] without inflation, but with the general
/// cofactor facets in every dimension, including the plane.
fn decide_with_cofactors(inner: &SparsePolyZonotope, outer: &SparsePolyZonotope) -> Option<bool> {
    let generators = inflated_generators(outer, 0.0)?;
    let normals = facets::cofactors(&generators, outer.dim())?;
    let hull = inner.interval_hull();
    let polytope = inner_polytope(outer.center(), &generators, &normals, &hull)?;

    Some(hull_inside(&hull, &polytope) || inner.containment(&polytope, 8) == Containment::Inside)
}

/// A pseudo-random generator for reproducible test inputs.
fn rng() -> rand_pcg::Pcg64Mcg {
    use sampling_planning::rand_core::SeedableRng;

    rand_pcg::Pcg64Mcg::seed_from_u64(2024)
}

/// A value uniform in `[lo, hi)`.
fn uniform(rng: &mut rand_pcg::Pcg64Mcg, lo: f64, hi: f64) -> f64 {
    lo + (hi - lo) * sampling_planning::unit_f64(rng)
}

/// The zonotope with `center` and generator columns `generators` in any dimension.
fn zonotope_n(center: &[f64], generators: &[Vec<f64>]) -> SparsePolyZonotope {
    let n = center.len();
    let g = DMatrix::from_fn(n, generators.len(), |i, j| generators[j][i]);

    SparsePolyZonotope::from_zonotope(&Zonotope::new(DVector::from_column_slice(center), g))
}

/// The axis-aligned box `center ± half_width` in any dimension.
fn cube(center: &[f64], half_width: f64) -> SparsePolyZonotope {
    let b = IntervalBox(
        center
            .iter()
            .map(|c| Interval::new(c - half_width, c + half_width))
            .collect(),
    );

    SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&b))
}

/// Random generators: `count` vectors with entries uniform in `[-1, 1)`.
fn random_generators(rng: &mut rand_pcg::Pcg64Mcg, n: usize, count: usize) -> Vec<Vec<f64>> {
    (0..count)
        .map(|_| (0..n).map(|_| uniform(rng, -1.0, 1.0)).collect())
        .collect()
}

#[test]
fn planar_and_general_facets_agree_in_the_plane() {
    let mut rng = rng();
    let mut proven = 0;

    for _ in 0..300 {
        let generators = random_generators(&mut rng, 2, 3);
        let outer = zonotope_n(&[0.0, 0.0], &generators);
        let center = [uniform(&mut rng, -0.5, 0.5), uniform(&mut rng, -0.5, 0.5)];
        let inner = cube(&center, uniform(&mut rng, 0.01, 0.4));
        let planar = decide_within(&inner, &outer, 0.0, 8);

        assert_eq!(
            planar,
            decide_with_cofactors(&inner, &outer),
            "{generators:?} {center:?}"
        );
        proven += usize::from(planar == Some(true));
    }

    assert!(
        proven > 20,
        "the cross-check is not vacuous: {proven} proven"
    );
}

#[test]
fn every_sampled_point_of_a_proven_box_lies_in_the_3d_zonotope() {
    let mut rng = rng();
    let mut proven = 0;

    for _ in 0..200 {
        let generators = random_generators(&mut rng, 3, 5);
        let outer = zonotope_n(&[0.0; 3], &generators);
        let center: Vec<f64> = (0..3).map(|_| uniform(&mut rng, -0.4, 0.4)).collect();
        let half = uniform(&mut rng, 0.01, 0.3);

        if decide_within(&cube(&center, half), &outer, 0.0, 8) != Some(true) {
            continue;
        }

        proven += 1;
        let normals = facets::cofactors(&generators, 3).expect("random generators have rank 3");

        for k in 0..64 {
            let point: Vec<f64> = (0..3)
                .map(|i| {
                    let offset = if k < 8 {
                        if k >> i & 1 == 0 { -half } else { half }
                    } else {
                        uniform(&mut rng, -half, half)
                    };

                    center[i] + offset
                })
                .collect();

            for n in &normals {
                let along = |v: &[f64]| n.normal.iter().zip(v).map(|(a, b)| a * b).sum::<f64>();
                let support: f64 = generators.iter().map(|g| along(g).abs()).sum();

                assert!(
                    along(&point).abs() <= support * (1.0 + 1e-9),
                    "sampled point {point:?} lies outside a facet"
                );
            }
        }
    }

    assert!(proven > 10, "the sampling is not vacuous: {proven} proven");
}

#[test]
fn a_flat_3d_zonotope_is_not_decided() {
    // All generators in the plane z = 0: rank 2, so the cofactor facets only cut a slab.
    let flat = zonotope_n(
        &[0.0; 3],
        &[
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![1.0, 1.0, 0.0],
        ],
    );

    assert_eq!(decide_within(&cube(&[0.0; 3], 0.1), &flat, 0.0, 8), None);
}

#[test]
fn a_unit_cube_contains_a_smaller_cube_but_not_a_corner_box() {
    let unit = cube(&[0.0; 3], 1.0);
    let rotated = zonotope_n(
        &[0.0; 3],
        &[
            vec![0.5, 0.5, 0.0],
            vec![0.5, -0.5, 0.0],
            vec![0.0, 0.0, 1.0],
        ],
    );

    assert_eq!(
        decide_within(&cube(&[0.2; 3], 0.5), &unit, 0.0, 8),
        Some(true)
    );
    assert_eq!(
        decide_within(&cube(&[0.9; 3], 0.5), &unit, 0.0, 8),
        Some(false)
    );
    // |x| + |y| <= 1 in the plane, |z| <= 1: the hull fits, the corners do not.
    assert_eq!(
        decide_within(&cube(&[0.0; 3], 0.6), &rotated, 0.0, 8),
        Some(false)
    );
    assert_eq!(
        decide_within(&cube(&[0.0; 3], 0.45), &rotated, 0.0, 8),
        Some(true)
    );
}

#[test]
fn tolerance_merges_touching_sets_only_up_to_the_tolerance() {
    let a = square([0.3, 0.7], 0.25);
    let swapped = zonotope([0.3, 0.7], &[[0.0, 0.25], [0.25, 0.0]]);

    assert_eq!(decide_within(&swapped, &a, 0.0, 8), Some(false));
    assert_eq!(decide_within(&swapped, &a, 1e-9, 8), Some(true));
    assert_eq!(
        decide_within(&square([0.3, 0.7], 0.26), &a, 1e-9, 8),
        Some(false)
    );
    assert_eq!(decide_within(&swapped, &a, f64::NAN, 8), None);
}

#[test]
fn too_many_generators_are_not_decided() {
    let mut rng = rng();
    let generators = random_generators(&mut rng, 2, MAX_GENERATORS + 1);
    let outer = zonotope_n(&[0.0, 0.0], &generators);

    assert_eq!(
        decide_within(&square([0.0, 0.0], 0.01), &outer, 0.0, 8),
        None
    );
}

#[test]
fn determinants_and_their_error_bounds_are_tight_on_exact_inputs() {
    let identity = |k: usize| -> Vec<Vec<f64>> {
        (0..k)
            .map(|i| (0..k).map(|j| f64::from(u8::from(i == j))).collect())
            .collect()
    };

    for k in 1..=4 {
        let (det, error) = facets::determinant(&identity(k));

        assert!((det - 1.0).abs() <= error, "k = {k}");
        assert!(error < 1e-14);
    }

    let swapped = vec![vec![0.0, 1.0], vec![1.0, 0.0]];
    assert_eq!(facets::determinant(&swapped).0, -1.0);
    assert_eq!(facets::subsets(4, 2).len(), 6);
}
