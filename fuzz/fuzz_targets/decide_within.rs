//! Fuzzes [`decide_within`] on arbitrary zonotopes in 2 to 4 dimensions.
//!
//! Inputs stay inside the shapes `decide_within` handles (at most [`MAX_GENERATORS`]
//! generators with the inflation), so most runs reach the facet construction instead of
//! returning `None`. Coordinates sit on a coarse grid, so parallel generators, zero
//! generators and touching sets are common: they are where the facets degenerate.
//!
//! Two properties are checked:
//!
//! - an equal representation is always proven inside itself;
//! - a proof is never wrong: if `inner` is proven inside `outer ⊕ [-tolerance, tolerance]ⁿ`,
//!   then along every direction `d` the support of `inner` is at most the support of
//!   `outer` plus `tolerance · ‖d‖₁`, up to a small rounding slack.

#![no_main]
#![forbid(unsafe_code)]

use libfuzzer_sys::fuzz_target;
use nalgebra::{DMatrix, DVector};
use robust_planning::containment::{MAX_DIMENSION, MAX_GENERATORS};
use robust_planning::polyzonotope::{SparsePolyZonotope, Zonotope};
use robust_planning::{decide_within, proven_within};

/// The most generators of the inner set.
const MAX_INNER_GENERATORS: usize = 6;

/// Relative slack of the support comparison. Generous against rounding, tiny against the
/// grid spacing of 1/16, so a wrong proof still fails.
const SLACK: f64 = 1e-9;

/// Reads fuzz bytes one at a time, with zeros once the input runs out.
struct Bytes<'a>(core::slice::Iter<'a, u8>);

impl Bytes<'_> {
    /// The next byte, or zero past the end.
    fn byte(&mut self) -> u8 {
        self.0.next().copied().unwrap_or(0)
    }

    /// A count in `0..bound`.
    fn below(&mut self, bound: usize) -> usize {
        usize::from(self.byte()) % bound
    }

    /// A coordinate on the grid of 1/16 in `[-8, 8)`.
    fn coordinate(&mut self) -> f64 {
        f64::from(i8::from_ne_bytes([self.byte()])) / 16.0
    }

    /// A zonotope in `n` dimensions with `count` generators.
    fn zonotope(&mut self, n: usize, count: usize) -> Zonotope {
        let center = DVector::from_fn(n, |_, _| self.coordinate());
        let generators = DMatrix::from_fn(n, count, |_, _| self.coordinate());

        Zonotope::new(center, generators)
    }
}

/// The support function `max dᵀx` of `z` along `d`.
fn support(z: &Zonotope, d: &DVector<f64>) -> f64 {
    d.dot(z.center())
        + z.generators()
            .column_iter()
            .map(|g| d.dot(&g).abs())
            .sum::<f64>()
}

/// The magnitude of every term of [`support`], the scale of its rounding error.
fn support_scale(z: &Zonotope, d: &DVector<f64>) -> f64 {
    magnitude(d, z.center().iter())
        + z.generators()
            .column_iter()
            .map(|g| magnitude(d, g.iter()))
            .sum::<f64>()
}

/// The sum of `|dᵢ vᵢ|` over the entries of `v`.
fn magnitude<'a>(d: &DVector<f64>, v: impl Iterator<Item = &'a f64>) -> f64 {
    d.iter().zip(v).map(|(a, b)| (a * b).abs()).sum()
}

/// The directions the supports are compared along: both signs of every axis, then a few
/// read from the input.
fn directions(bytes: &mut Bytes<'_>, n: usize) -> Vec<DVector<f64>> {
    let axes = (0..n).flat_map(|i| {
        [1.0, -1.0].map(|sign| DVector::from_fn(n, |k, _| if k == i { sign } else { 0.0 }))
    });
    let read: Vec<DVector<f64>> = (0..4)
        .map(|_| DVector::from_fn(n, |_, _| bytes.coordinate()))
        .collect();

    axes.chain(read).collect()
}

/// Panics unless `inner` lies inside `outer ⊕ [-tolerance, tolerance]ⁿ` along `directions`.
fn assert_supports_fit(
    inner: &Zonotope,
    outer: &Zonotope,
    tolerance: f64,
    directions: &[DVector<f64>],
) {
    for d in directions {
        let inflation = tolerance * d.lp_norm(1);
        let bound = support(outer, d) + inflation;
        let slack = SLACK * (1.0 + support_scale(inner, d) + support_scale(outer, d) + inflation);

        assert!(
            support(inner, d) <= bound + slack,
            "proven inside, but sticks out along {d:?}: {} > {bound}\ninner {inner:?}\nouter {outer:?}",
            support(inner, d),
        );
    }
}

fuzz_target!(|data: &[u8]| {
    let mut bytes = Bytes(data.iter());
    let n = 2 + bytes.below(MAX_DIMENSION - 1);
    let inner_count = bytes.below(MAX_INNER_GENERATORS + 1);
    let outer_count = 1 + bytes.below(MAX_GENERATORS - MAX_DIMENSION);
    let inner = bytes.zonotope(n, inner_count);
    let outer = bytes.zonotope(n, outer_count);
    let tolerance = f64::from(bytes.byte() % 4) / 8.0;
    let max_splits = bytes.below(6);
    let sparse_inner = SparsePolyZonotope::from_zonotope(&inner);
    let sparse_outer = SparsePolyZonotope::from_zonotope(&outer);

    assert!(
        proven_within(&sparse_outer, &sparse_outer, 0),
        "an equal representation is inside itself"
    );

    if decide_within(&sparse_inner, &sparse_outer, tolerance, max_splits) == Some(true) {
        let directions = directions(&mut bytes, n);

        assert_supports_fit(&inner, &outer, tolerance, &directions);
    }
});
