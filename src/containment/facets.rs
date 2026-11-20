//! Facet normals of zonotopes, each with a rigorous bound on its rounding error.
//!
//! A zonotope of rank `n` in `n` dimensions is the intersection of the facet pairs whose
//! normals are the generalised cross products of every `n - 1` of its generators. In the
//! plane those normals are exact (a swap and a negation); in higher dimensions they are
//! determinants, computed by Leibniz expansion with a componentwise error bound.

/// A facet normal as computed, with a bound on how far each entry is from the exact one.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct Normal {
    /// The computed normal; never all zero in a returned list.
    pub(super) normal: Vec<f64>,
    /// By entry, an upper bound on `|computed - exact|`. Non-negative.
    pub(super) error: Vec<f64>,
}

/// The exact normals of a planar zonotope: for each generator, the one perpendicular to
/// it (its facet pair) and the one along it (its support pair, which bounds a segment).
pub(super) fn planar(generators: &[Vec<f64>]) -> Vec<Normal> {
    generators
        .iter()
        .flat_map(|g| [vec![-g[1], g[0]], g.clone()])
        .map(|normal| Normal {
            normal,
            error: vec![0.0; 2],
        })
        .collect()
}

/// The cofactor normal of every `n - 1` of the `n`-dimensional `generators`, or `None` if
/// the zonotope is not proven to have rank `n` (some `n` generators with a determinant
/// proven non-zero).
///
/// Below rank `n` the facet pairs only cut out a slab around a lower-dimensional zonotope
/// and are no longer its H-representation, so nothing could be proven from them.
pub(super) fn cofactors(generators: &[Vec<f64>], n: usize) -> Option<Vec<Normal>> {
    if !has_full_rank(generators, n) {
        return None;
    }

    let normals = subsets(generators.len(), n - 1)
        .into_iter()
        .map(|subset| cofactor(generators, &subset, n))
        .filter(|normal| normal.normal.iter().any(|&x| x != 0.0))
        .collect();

    Some(normals)
}

/// Whether some `n` generators have a determinant whose magnitude exceeds its error bound.
fn has_full_rank(generators: &[Vec<f64>], n: usize) -> bool {
    subsets(generators.len(), n).into_iter().any(|subset| {
        let rows: Vec<Vec<f64>> = (0..n)
            .map(|i| subset.iter().map(|&j| generators[j][i]).collect())
            .collect();
        let (det, error) = determinant(&rows);

        det.abs() > error
    })
}

/// The generalised cross product of the generators in `subset`: entry `i` is
/// `(-1)^i` times the determinant of their matrix without row `i`.
fn cofactor(generators: &[Vec<f64>], subset: &[usize], n: usize) -> Normal {
    let (normal, error) = (0..n)
        .map(|skip| {
            let rows: Vec<Vec<f64>> = (0..n)
                .filter(|&i| i != skip)
                .map(|i| subset.iter().map(|&j| generators[j][i]).collect())
                .collect();
            let (det, error) = determinant(&rows);
            let sign = if skip.is_multiple_of(2) { 1.0 } else { -1.0 };

            (sign * det, error)
        })
        .unzip();

    Normal { normal, error }
}

/// The determinant of the square matrix `rows` by Leibniz expansion, with an upper bound on
/// its rounding error.
///
/// Each of the `k!` products has `k - 1` roundings and the sum `k! - 1` more, so the error is
/// at most `γ_{k + k! - 1} Σ |products|` (Higham, *Accuracy and Stability of Numerical
/// Algorithms*, §3.1); the bound is doubled against its own rounding, and an absolute term
/// covers underflow. Meant for `k <= 4`.
pub(super) fn determinant(rows: &[Vec<f64>]) -> (f64, f64) {
    let k = rows.len();
    let mut value = 0.0;
    let mut magnitude = 0.0;

    for (permutation, sign) in permutations(k) {
        let product: f64 = permutation
            .iter()
            .enumerate()
            .map(|(i, &j)| rows[i][j])
            .product();

        value += sign * product;
        magnitude += product.abs();
    }

    let roundings = k + factorial(k) - 1;
    let error = 2.0 * gamma(roundings) * magnitude + count_to_f64(roundings) * f64::MIN_POSITIVE;

    (value, error)
}

/// Every permutation of `0..k` with its sign, by recursive insertion.
fn permutations(k: usize) -> Vec<(Vec<usize>, f64)> {
    if k == 0 {
        return vec![(Vec::new(), 1.0)];
    }

    let mut out = Vec::new();

    for (smaller, sign) in permutations(k - 1) {
        // Inserting `k - 1` at position `p` moves it past `k - 1 - p` larger positions.
        for p in 0..k {
            let mut permutation = smaller.clone();
            permutation.insert(p, k - 1);
            let flips = k - 1 - p;
            let parity = if flips.is_multiple_of(2) { 1.0 } else { -1.0 };
            out.push((permutation, sign * parity));
        }
    }

    out
}

/// Every increasing `size`-subset of `0..count`, in lexicographic order.
pub(super) fn subsets(count: usize, size: usize) -> Vec<Vec<usize>> {
    let mut out = Vec::new();
    let mut current = Vec::with_capacity(size);
    extend_subsets(0, count, size, &mut current, &mut out);

    out
}

/// Appends to `out` every way to complete `current` to `size` entries from `start..count`.
fn extend_subsets(
    start: usize,
    count: usize,
    size: usize,
    current: &mut Vec<usize>,
    out: &mut Vec<Vec<usize>>,
) {
    if current.len() == size {
        out.push(current.clone());
        return;
    }

    for next in start..count {
        current.push(next);
        extend_subsets(next + 1, count, size, current, out);
        current.pop();
    }
}

/// `k!`, for the small `k` of Leibniz expansion.
fn factorial(k: usize) -> usize {
    (1..=k).product()
}

/// Higham's `γ_k = k u / (1 - k u)` with `u = 2⁻⁵³`: the relative error bound of `k`
/// roundings to nearest.
pub(super) fn gamma(k: usize) -> f64 {
    let k = count_to_f64(k);
    let u = f64::EPSILON / 2.0;

    k * u / (1.0 - k * u)
}

/// Converts a count to `f64`, saturating at `u32::MAX`.
pub(super) fn count_to_f64(count: usize) -> f64 {
    f64::from(u32::try_from(count).unwrap_or(u32::MAX))
}
