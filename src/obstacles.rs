//! Polytope obstacles and the safety check of reachable sets against them.

use polyzonotope::{Polytope, SparsePolyZonotope};

use crate::SafetyVerdict;

/// A set of convex polytope obstacles, checked against reachable sets.
///
/// Each check calls [`SparsePolyZonotope::intersection`] with a split budget of
/// [`Obstacles::max_splits`]. Splitting refines the enclosure where it is too coarse to
/// separate a set from an obstacle, so a set is only given up on after the budget is spent.
///
/// # Examples
///
/// ```
/// use robust_planning::{Obstacles, SafetyVerdict};
/// use robust_planning::polyzonotope::{
///     Interval, IntervalBox, Polytope, SparsePolyZonotope, Zonotope,
/// };
///
/// let square = |lo: f64, hi: f64| IntervalBox(vec![Interval::new(lo, hi); 2]);
/// let obstacles = Obstacles::new(vec![Polytope::from_box(&square(2.0, 3.0))], 4);
///
/// let clear = SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&square(0.0, 1.0)));
/// let touching = SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&square(1.5, 2.5)));
///
/// assert_eq!(obstacles.check_set(&clear), SafetyVerdict::Safe);
/// assert_eq!(obstacles.check_set(&touching), SafetyVerdict::Indeterminate);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Obstacles {
    /// The obstacles. A state inside any of them is unsafe.
    polytopes: Vec<Polytope>,
    /// Split budget per set and obstacle; `0` disables refinement.
    max_splits: usize,
}

impl Obstacles {
    /// Creates an obstacle set with the given split budget per set and obstacle.
    #[must_use]
    pub const fn new(polytopes: Vec<Polytope>, max_splits: usize) -> Self {
        Self {
            polytopes,
            max_splits,
        }
    }

    /// The obstacles.
    #[must_use]
    pub fn polytopes(&self) -> &[Polytope] {
        &self.polytopes
    }

    /// Split budget per set and obstacle.
    #[must_use]
    pub const fn max_splits(&self) -> usize {
        self.max_splits
    }

    /// Checks one set against every obstacle.
    ///
    /// Returns [`SafetyVerdict::Safe`] only if the set is proven disjoint from every obstacle.
    /// Stops at the first obstacle that is not proven disjoint and returns its verdict.
    ///
    /// # Panics
    ///
    /// Panics if an obstacle's facet normals do not have the set's dimension.
    pub fn check_set(&self, set: &SparsePolyZonotope) -> SafetyVerdict {
        for obstacle in &self.polytopes {
            let verdict =
                SafetyVerdict::from_intersection(set.intersection(obstacle, self.max_splits));

            if !verdict.is_safe() {
                return verdict;
            }
        }

        SafetyVerdict::Safe
    }

    /// Checks every set, for example all sets covering one edge.
    ///
    /// Returns [`SafetyVerdict::Safe`] only if every set is proven disjoint from every
    /// obstacle; an empty list is safe. Stops at the first set that is not proven safe.
    ///
    /// # Panics
    ///
    /// Panics if an obstacle's facet normals do not have a set's dimension.
    pub fn check_sets(&self, sets: &[SparsePolyZonotope]) -> SafetyVerdict {
        for set in sets {
            let verdict = self.check_set(set);

            if !verdict.is_safe() {
                return verdict;
            }
        }

        SafetyVerdict::Safe
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{DMatrix, DVector};
    use polyzonotope::{Interval, IntervalBox, Zonotope};

    use super::*;

    /// A diamond `|x| + |y| <= 1` near a box corner: disjoint, but no single facet of the box
    /// separates it, so proving disjointness needs a split.
    fn diamond_near_corner() -> (SparsePolyZonotope, Polytope) {
        let generators = DMatrix::from_column_slice(2, 2, &[0.5, 0.5, 0.5, -0.5]);
        let diamond = Zonotope::new(DVector::zeros(2), generators);
        let corner = IntervalBox(vec![Interval::new(0.6, 2.0); 2]);

        (
            SparsePolyZonotope::from_zonotope(&diamond),
            Polytope::from_box(&corner),
        )
    }

    #[test]
    fn unknown_without_splits_is_indeterminate() {
        let (diamond, obstacle) = diamond_near_corner();
        let obstacles = Obstacles::new(vec![obstacle], 0);

        assert_eq!(obstacles.check_set(&diamond), SafetyVerdict::Indeterminate);
    }

    #[test]
    fn splitting_proves_disjointness() {
        let (diamond, obstacle) = diamond_near_corner();
        let obstacles = Obstacles::new(vec![obstacle], 4);

        assert_eq!(obstacles.check_set(&diamond), SafetyVerdict::Safe);
    }

    #[test]
    fn overlapping_set_is_not_safe() {
        let (diamond, _) = diamond_near_corner();
        let overlap = Polytope::from_box(&IntervalBox(vec![Interval::new(0.2, 2.0); 2]));
        let obstacles = Obstacles::new(vec![overlap], 8);

        assert_eq!(obstacles.check_set(&diamond), SafetyVerdict::Indeterminate);
        assert!(obstacles.check_sets(&[]).is_safe());
    }
}
