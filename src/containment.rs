//! Proven set-in-set containment: a set inside a zonotope, built on polyzonotope's
//! set-in-polytope test.
//!
//! polyzonotope can prove a set inside a polytope, but not inside another set. Testing
//! against an *outer* polytope of the other set would be unsound, so this module builds an
//! *inner* one. A zonotope of rank `n` in `n` dimensions is exactly the intersection of the
//! facet pairs whose normals are the generalised cross products of every `n - 1` of its
//! generators. The normals come with rigorous error bounds (zero in the
//! plane), and every offset is lowered by the error of the normal and its own rounding, so
//! that the inner set satisfying the computed facets implies it satisfies the exact ones.
//!
//! ```mermaid
//! graph LR
//!     Shape{"zonotope, 2 to 4 dims, at most 12 generators?"} -->|no| Undecided["None"]
//!     Shape -->|yes| Facets["facets with error bounds"]
//!     Facets -->|rank not proven| Undecided
//!     Facets --> Box{"inner hull inside?"}
//!     Box -->|yes| Inside["Some(true)"]
//!     Box -->|no| Split["polyzonotope containment"]
//! ```

mod facets;
#[cfg(test)]
mod tests;

use nalgebra::DVector;
use polyzonotope::{Containment, Halfspace, IntervalBox, Polytope, SparsePolyZonotope};

use facets::{Normal, count_to_f64, gamma};

/// The largest dimension [`decide_within`] handles. Determinants are expanded by Leibniz,
/// whose cost grows with the factorial of the dimension.
pub const MAX_DIMENSION: usize = 4;

/// The most generators (including the inflation) [`decide_within`] handles. The facet
/// count grows like `generators^(dimension - 1)`.
pub const MAX_GENERATORS: usize = 12;

/// Whether `inner` is proven to lie inside `outer`: [`decide_within`] without inflation,
/// with an undecided shape read as not proven.
#[must_use]
pub fn proven_within(
    inner: &SparsePolyZonotope,
    outer: &SparsePolyZonotope,
    max_splits: usize,
) -> bool {
    decide_within(inner, outer, 0.0, max_splits) == Some(true)
}

/// Tries to prove `inner` inside `outer ⊕ [-tolerance, tolerance]ⁿ`.
///
/// - `None`: the shape is not handled, and nothing is claimed. `outer` must be a zonotope
///   in sparse form (every dependent monomial one factor to the power one, no factor in
///   two monomials) in 2 to [`MAX_DIMENSION`] dimensions, with at most
///   [`MAX_GENERATORS`] non-zero generators counting the `n` inflation generators, finite
///   entries, and in 3 or more dimensions a rank proven to be full; `inner` must have the
///   same dimension, and `tolerance` must be finite and non-negative.
/// - `Some(true)`: proven inside. Rigorous with the `sound` feature; without it the final
///   set-in-polytope step is up to floating-point rounding, as every check in this crate.
///   The facet offsets are rounded down rigorously in both modes.
/// - `Some(false)`: not proven, which need not mean it sticks out.
///
/// Equal representations (ids aside) are proven inside each other directly: rounding the
/// offsets down makes the boundary strict, so touching sets are otherwise not proven.
/// A positive `tolerance` merges them as well, at the price of a dominance that is only
/// true up to `tolerance`. The cheap test that the interval hull of `inner` lies inside the
/// facets runs before polyzonotope's splitting test, whose budget is `max_splits`.
#[must_use]
pub fn decide_within(
    inner: &SparsePolyZonotope,
    outer: &SparsePolyZonotope,
    tolerance: f64,
    max_splits: usize,
) -> Option<bool> {
    if same_representation(inner, outer) && tolerance >= 0.0 {
        return Some(true);
    }

    let n = outer.dim();
    let shape_ok = inner.dim() == n
        && (2..=MAX_DIMENSION).contains(&n)
        && tolerance.is_finite()
        && tolerance >= 0.0
        && is_zonotope(outer);

    if !shape_ok {
        return None;
    }

    let generators = inflated_generators(outer, tolerance)?;
    let normals = if n == 2 {
        facets::planar(&generators)
    } else {
        facets::cofactors(&generators, n)?
    };
    let hull = inner.interval_hull();
    let polytope = inner_polytope(outer.center(), &generators, &normals, &hull)?;

    if hull_inside(&hull, &polytope) {
        return Some(true);
    }

    Some(inner.containment(&polytope, max_splits) == Containment::Inside)
}

/// Whether both sets have the same center, generators and exponents, entry for entry.
///
/// Factor ids may differ: two representations with equal entries describe the same set,
/// since each factor ranges over `[-1, 1]` whatever its id.
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

/// The non-zero generators of a set that [`is_zonotope`], dependent and independent, plus
/// `tolerance` along every axis if it is positive; `None` if any entry of the center or a
/// generator is not finite or there are none or too many.
fn inflated_generators(set: &SparsePolyZonotope, tolerance: f64) -> Option<Vec<Vec<f64>>> {
    let n = set.dim();
    let mut generators: Vec<Vec<f64>> = set
        .dependent_generators()
        .column_iter()
        .chain(set.independent_generators().column_iter())
        .map(|g| g.iter().copied().collect::<Vec<f64>>())
        .filter(|g| g.iter().any(|&x| x != 0.0))
        .collect();

    if tolerance > 0.0 {
        generators.extend((0..n).map(|i| {
            let mut axis = vec![0.0; n];
            axis[i] = tolerance;
            axis
        }));
    }

    let finite = set.center().iter().all(|x| x.is_finite())
        && generators.iter().flatten().all(|x| x.is_finite());

    (finite && !generators.is_empty() && generators.len() <= MAX_GENERATORS).then_some(generators)
}

/// The polytope of both facets of every normal, with offsets lowered so that it lies
/// inside the zonotope for every set whose interval hull is `hull`; `None` if an offset is
/// not finite (an infinite one would cut nothing and a NaN one is never refuted).
fn inner_polytope(
    center: &DVector<f64>,
    generators: &[Vec<f64>],
    normals: &[Normal],
    hull: &IntervalBox,
) -> Option<Polytope> {
    let reach = hull_reach(center, hull);
    let facets: Vec<Halfspace> = normals
        .iter()
        .flat_map(|n| {
            let negated = Normal {
                normal: n.normal.iter().map(|x| -x).collect(),
                error: n.error.clone(),
            };

            [n.clone(), negated]
        })
        .map(|n| Halfspace {
            offset: offset_lower_bound(&n, center, generators, &reach),
            normal: DVector::from_vec(n.normal),
        })
        .collect();

    facets
        .iter()
        .all(|h| h.offset.is_finite())
        .then_some(Polytope { facets })
}

/// By axis, an upper bound on `|x - center|` over the box `hull`.
fn hull_reach(center: &DVector<f64>, hull: &IntervalBox) -> Vec<f64> {
    hull.0
        .iter()
        .zip(center.iter())
        .map(|(range, c)| (range.hi - c).abs().max((range.lo - c).abs()) * (1.0 + f64::EPSILON))
        .collect()
}

/// A lower bound on the offset that makes the computed facet imply the exact one.
///
/// With exact normal `c = ĉ - e`, `|eᵢ| <= εᵢ`, a point `x` of the inner set with
/// `|xᵢ - centerᵢ| <= rᵢ` satisfies the exact facet `cᵀ (x - center) <= Σⱼ |cᵀ gⱼ|` if
/// `ĉᵀ x <= ĉᵀ center + Σⱼ |ĉᵀ gⱼ| - Σᵢ εᵢ (Σⱼ |gⱼᵢ| + rᵢ)`. That right-hand side is
/// evaluated in `f64` and lowered by `2 γ_{n + m + 2}` times the sum of the magnitudes of
/// its terms (Higham §3.1), with the slack term rounded up, plus an absolute term for
/// underflow.
fn offset_lower_bound(
    n: &Normal,
    center: &DVector<f64>,
    generators: &[Vec<f64>],
    reach: &[f64],
) -> f64 {
    let c = &n.normal;
    let dot = |v: &[f64]| c.iter().zip(v).map(|(a, b)| a * b).sum::<f64>();
    let magnitude = |v: &[f64]| c.iter().zip(v).map(|(a, b)| (a * b).abs()).sum::<f64>();
    let center = center.as_slice();
    let value = dot(center) + generators.iter().map(|g| dot(g).abs()).sum::<f64>();
    let scale = magnitude(center) + generators.iter().map(|g| magnitude(g)).sum::<f64>();
    let slack: f64 = (0..c.len())
        .map(|i| n.error[i] * (generators.iter().map(|g| g[i].abs()).sum::<f64>() + reach[i]))
        .sum();
    let terms = c.len() + generators.len() + 2;
    let g = gamma(terms);

    value - 2.0 * g * scale - slack * (1.0 + 4.0 * g) - count_to_f64(terms) * f64::MIN_POSITIVE
}

/// Whether every point of the box `hull` lies in `polytope`, with the maximum of each facet
/// over the box rounded up.
fn hull_inside(hull: &IntervalBox, polytope: &Polytope) -> bool {
    polytope.facets.iter().all(|h| {
        let (value, scale) = h
            .normal
            .iter()
            .zip(&hull.0)
            .fold((0.0, 0.0), |(v, s), (a, r)| {
                let term = (a * r.lo).max(a * r.hi);

                (v + term, s + term.abs())
            });
        let terms = h.normal.len() + 1;

        value + 2.0 * gamma(terms) * scale + count_to_f64(terms) * f64::MIN_POSITIVE <= h.offset
    })
}
