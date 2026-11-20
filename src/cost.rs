//! Edge costs: nominal length, optionally weighted by how wide the reachable set gets.

use polyzonotope::SparsePolyZonotope;

/// How a planner prices one edge.
///
/// Both options are additive along a path and never below the nominal length, so a
/// Euclidean distance stays an admissible heuristic for them and RRT* rewiring stays
/// well-defined.
///
/// # Examples
///
/// ```
/// use robust_planning::Cost;
/// use robust_planning::polyzonotope::{Interval, IntervalBox, SparsePolyZonotope, Zonotope};
///
/// let set = SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&IntervalBox(vec![
///     Interval::new(0.0, 0.2),
///     Interval::new(0.0, 0.4),
/// ])));
///
/// assert_eq!(Cost::Length.edge(1.5, &set), 1.5);
/// let weighted = Cost::WidthWeighted { weight: 2.0 }.edge(1.5, &set);
/// assert!((weighted - (1.5 + 2.0 * 0.4)).abs() < 1e-12);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Cost {
    /// The nominal path length. The default.
    #[default]
    Length,
    /// The nominal length plus `weight` times the width of the edge's end set.
    ///
    /// The width is the largest side of the set's interval hull, so a path that keeps its
    /// sets narrow, and with them its distance to the obstacles that bounded the proof,
    /// costs less. A robustness-weighted cost: `weight` is length per unit of width.
    /// `weight` is finite and non-negative; `0` prices like [`Cost::Length`] up to the
    /// added zero.
    ///
    /// The width term is charged per edge, not per unit of length, so the same path cut
    /// into more edges costs more: the cost also prefers fewer, longer edges. Because of
    /// that, RRT*'s optimality argument does not carry over to this cost.
    WidthWeighted {
        /// Length units charged per unit of end-set width. Finite and non-negative.
        weight: f64,
    },
}

impl Cost {
    /// The cost of an edge of nominal `length` that ends in `end`.
    ///
    /// [`Cost::Length`] returns `length` unchanged, so costs summed from the root equal the
    /// nominal path length bit for bit.
    #[must_use]
    pub fn edge(&self, length: f64, end: &SparsePolyZonotope) -> f64 {
        match *self {
            Self::Length => length,
            Self::WidthWeighted { weight } => length + weight * width(end),
        }
    }

    /// Whether the weight, if any, is finite and non-negative.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        match *self {
            Self::Length => true,
            Self::WidthWeighted { weight } => weight.is_finite() && weight >= 0.0,
        }
    }
}

/// The largest side of the interval hull of `set`.
fn width(set: &SparsePolyZonotope) -> f64 {
    set.interval_hull()
        .0
        .iter()
        .map(polyzonotope::Interval::width)
        .fold(0.0, f64::max)
}

#[cfg(test)]
mod tests {
    use polyzonotope::{Interval, IntervalBox, Zonotope};

    use super::*;

    /// The box `[0, w] x [0, h]` as a set.
    fn rect(w: f64, h: f64) -> SparsePolyZonotope {
        SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&IntervalBox(vec![
            Interval::new(0.0, w),
            Interval::new(0.0, h),
        ])))
    }

    #[test]
    fn length_ignores_the_set() {
        assert_eq!(
            Cost::Length.edge(0.75, &rect(3.0, 4.0)).to_bits(),
            0.75f64.to_bits()
        );
    }

    #[test]
    fn width_weighted_adds_the_largest_hull_side() {
        let cost = Cost::WidthWeighted { weight: 0.5 };

        assert!((cost.edge(1.0, &rect(0.2, 0.6)) - 1.3).abs() < 1e-12);
        assert!(
            cost.edge(1.0, &rect(0.2, 0.6)) >= 1.0,
            "never below the length"
        );
    }

    #[test]
    fn only_finite_non_negative_weights_are_valid() {
        assert!(Cost::Length.is_valid());
        assert!(Cost::WidthWeighted { weight: 0.0 }.is_valid());
        assert!(!Cost::WidthWeighted { weight: -1.0 }.is_valid());
        assert!(!Cost::WidthWeighted { weight: f64::NAN }.is_valid());
        assert!(
            !Cost::WidthWeighted {
                weight: f64::INFINITY
            }
            .is_valid()
        );
    }
}
