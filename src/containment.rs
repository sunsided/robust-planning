//! Proven set-in-set containment for planar zonotopes, built on polyzonotope's
//! set-in-polytope test.
//!
//! polyzonotope 0.2 can prove a set inside a polytope, but not inside another set. When the
//! outer set is a planar zonotope, its exact H-representation is known in closed form: one
//! pair of facets per generator, with normal perpendicular to it. The normals are computed
//! exactly (a swap and a negation); only the offsets round, and they are rounded *down* with
//! a rigorous error bound, so the polytope lies inside the outer set. A proof that the inner
//! set lies in that polytope is then a proof that it lies in the outer set.

use nalgebra::DVector;
use polyzonotope::{Containment, Halfspace, Polytope, SparsePolyZonotope};

/// Whether `inner` is proven to lie inside `outer`.
///
/// Answers `false` unless both sets are planar and `outer` is a zonotope in sparse form:
/// every dependent monomial is a single factor to the power one, and no factor appears in
/// two monomials. Other shapes are not decided, which only costs a deduplication. The proof
/// is rigorous with the `sound` feature; without it, the final set-in-polytope test is up
/// to floating-point rounding, as every check in this crate.
///
/// `max_splits` bounds the splits of the set-in-polytope test.
#[must_use]
pub fn proven_within(
    inner: &SparsePolyZonotope,
    outer: &SparsePolyZonotope,
    max_splits: usize,
) -> bool {
    if same_representation(inner, outer) {
        return true;
    }

    if inner.dim() != 2 || outer.dim() != 2 || !is_zonotope(outer) {
        return false;
    }

    let generators = generators(outer);

    if generators.is_empty() {
        return false;
    }

    let polytope = inner_polytope(outer.center(), &generators);

    inner.containment(&polytope, max_splits) == Containment::Inside
}

/// Whether both sets have the same center, generators and exponents, entry for entry.
///
/// Factor ids may differ: two representations with equal entries describe the same set,
/// since each factor ranges over `[-1, 1]` whatever its id. The strict facet test cannot
/// show such touching sets inside each other.
fn same_representation(a: &SparsePolyZonotope, b: &SparsePolyZonotope) -> bool {
    a.center() == b.center()
        && a.dependent_generators() == b.dependent_generators()
        && a.independent_generators() == b.independent_generators()
        && a.exponents() == b.exponents()
}

/// Whether every dependent monomial of `set` is one factor to the power one, and every
/// factor appears in at most one monomial, so that the set is the zonotope of its
/// generators.
fn is_zonotope(set: &SparsePolyZonotope) -> bool {
    let e = set.exponents();
    let linear_columns = e.column_iter().all(|column| {
        column.iter().filter(|&&p| p != 0).count() == 1 && column.iter().all(|&p| p <= 1)
    });
    let distinct_factors = e
        .row_iter()
        .all(|row| row.iter().filter(|&&p| p != 0).count() <= 1);

    linear_columns && distinct_factors
}

/// The non-zero generators of a set that [`is_zonotope`], dependent and independent.
fn generators(set: &SparsePolyZonotope) -> Vec<[f64; 2]> {
    set.dependent_generators()
        .column_iter()
        .chain(set.independent_generators().column_iter())
        .map(|g| [g[0], g[1]])
        .filter(|g| *g != [0.0, 0.0])
        .collect()
}

/// A polytope inside the zonotope `center + Σ βᵢ gᵢ`, `βᵢ ∈ [-1, 1]`, equal to it up to the
/// downward rounding of its offsets.
///
/// For each generator, the facet pair perpendicular to it (the exact H-representation of a
/// planar zonotope of rank 2), plus the support pair along it, which makes a rank-1
/// zonotope (a segment) bounded too. Every halfspace contains the zonotope before rounding,
/// and the exact facets alone cut it out, so the intersection is the zonotope.
fn inner_polytope(center: &DVector<f64>, generators: &[[f64; 2]]) -> Polytope {
    let c = [center[0], center[1]];
    let facets = generators
        .iter()
        .flat_map(|g| [[-g[1], g[0]], *g])
        .flat_map(|n| [n, [-n[0], -n[1]]])
        .map(|n| Halfspace {
            normal: DVector::from_column_slice(&n),
            offset: support_lower_bound(&n, &c, generators),
        })
        .collect();

    Polytope { facets }
}

/// A lower bound on the support function `nᵀ c + Σ |nᵀ gᵢ|` of the zonotope in direction
/// `n`, from the plain-`f64` value minus a rigorous bound on its rounding error.
///
/// Each of the `m + 1` dot products and the sum over them round to nearest; the total error
/// is at most `γ_{m+3} Σ |terms|` (Higham, *Accuracy and Stability of Numerical
/// Algorithms*, §3.1), with `γ_k = k u / (1 - k u)` and `u = 2⁻⁵³`. The bound is doubled
/// against the rounding of its own evaluation, and an absolute term covers underflow.
fn support_lower_bound(n: &[f64; 2], c: &[f64; 2], generators: &[[f64; 2]]) -> f64 {
    let dot = |v: &[f64; 2]| n[0] * v[0] + n[1] * v[1];
    let magnitude = |v: &[f64; 2]| (n[0] * v[0]).abs() + (n[1] * v[1]).abs();
    let value = dot(c) + generators.iter().map(|g| dot(g).abs()).sum::<f64>();
    let scale = magnitude(c) + generators.iter().map(magnitude).sum::<f64>();
    let terms = count_to_f64(generators.len() + 3);
    let u = f64::EPSILON / 2.0;
    let gamma = terms * u / (1.0 - terms * u);
    let error = 2.0 * gamma * scale + terms * f64::MIN_POSITIVE;

    value - error
}

/// Converts a count to `f64`, saturating at `u32::MAX`.
fn count_to_f64(count: usize) -> f64 {
    f64::from(u32::try_from(count).unwrap_or(u32::MAX))
}

#[cfg(test)]
mod tests {
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
        let cube = SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&IntervalBox(vec![
            Interval::new(-1.0, 1.0);
            3
        ])));

        assert!(!is_zonotope(&shared), "one factor drives two generators");
        assert!(!proven_within(&square([0.0, 0.0], 0.01), &shared, 8));
        let small_cube =
            SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&IntervalBox(vec![
                Interval::new(
                    -0.5, 0.5
                );
                3
            ])));
        assert!(!proven_within(&small_cube, &cube, 8), "3D is not decided");
    }

    #[test]
    fn offsets_are_rounded_down() {
        let generators = [[0.1, 0.2], [0.3, -0.7]];
        let c = [1.0 / 3.0, 2.0 / 3.0];

        for n in [[0.7, 0.1], [-0.2, 0.9]] {
            let exact = n[0] * c[0] + n[1] * c[1];
            let bound = support_lower_bound(&n, &c, &generators);
            let sum: f64 = generators
                .iter()
                .map(|g| (n[0] * g[0] + n[1] * g[1]).abs())
                .sum();

            assert!(bound < exact + sum, "strictly below the f64 value");
            assert!(exact + sum - bound < 1e-14, "but only by rounding error");
        }
    }
}
