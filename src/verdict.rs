//! The three-valued result of a conservative safety check.

use polyzonotope::Intersection;
use sampling_planning::Validity;

/// The result of a conservative safety check.
///
/// Over-approximations can only prove safety. A check that cannot prove it
/// returns [`SafetyVerdict::Indeterminate`], never [`SafetyVerdict::Safe`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[must_use]
pub enum SafetyVerdict {
    /// The reachable set provably avoids every unsafe region.
    ///
    /// A proof only with the `sound` feature; without it, up to floating-point rounding.
    Safe,
    /// Some reachable state provably lies in an unsafe region.
    ///
    /// Never produced from an over-approximated set: a point of the enclosure inside an
    /// obstacle need not be reachable. No check in this crate produces it yet.
    Unsafe,
    /// Neither could be proven; refine or reject.
    Indeterminate,
}

impl SafetyVerdict {
    /// The verdict for one over-approximated set against one obstacle.
    ///
    /// Only [`Intersection::Disjoint`] proves anything, so it is the only answer that maps to
    /// [`SafetyVerdict::Safe`]. [`Intersection::Intersects`] maps to
    /// [`SafetyVerdict::Indeterminate`]: its witness is evaluated in plain `f64` and lies in
    /// the enclosure, not necessarily in the true reachable set.
    pub const fn from_intersection(intersection: Intersection) -> Self {
        match intersection {
            Intersection::Disjoint => Self::Safe,
            Intersection::Intersects | Intersection::Unknown => Self::Indeterminate,
        }
    }

    /// Whether a planner may accept this result. Only [`SafetyVerdict::Safe`] is accepted.
    #[must_use]
    pub const fn is_safe(self) -> bool {
        matches!(self, Self::Safe)
    }

    /// Combines two verdicts for a compound check: [`SafetyVerdict::Unsafe`] dominates
    /// [`SafetyVerdict::Indeterminate`], which dominates [`SafetyVerdict::Safe`].
    pub const fn and(self, other: Self) -> Self {
        match (self, other) {
            (Self::Unsafe, _) | (_, Self::Unsafe) => Self::Unsafe,
            (Self::Indeterminate, _) | (_, Self::Indeterminate) => Self::Indeterminate,
            (Self::Safe, Self::Safe) => Self::Safe,
        }
    }
}

impl From<SafetyVerdict> for Validity {
    /// Safe becomes [`Validity::Valid`], Unsafe [`Validity::Invalid`] and Indeterminate
    /// [`Validity::Unknown`], which `sampling-planning`'s planners reject.
    fn from(verdict: SafetyVerdict) -> Self {
        match verdict {
            SafetyVerdict::Safe => Self::Valid,
            SafetyVerdict::Unsafe => Self::Invalid,
            SafetyVerdict::Indeterminate => Self::Unknown,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_disjoint_is_safe() {
        assert_eq!(
            SafetyVerdict::from_intersection(Intersection::Disjoint),
            SafetyVerdict::Safe
        );
        assert_eq!(
            SafetyVerdict::from_intersection(Intersection::Intersects),
            SafetyVerdict::Indeterminate
        );
        assert_eq!(
            SafetyVerdict::from_intersection(Intersection::Unknown),
            SafetyVerdict::Indeterminate
        );
    }

    #[test]
    fn unsafe_dominates_indeterminate_dominates_safe() {
        use SafetyVerdict::{Indeterminate, Safe, Unsafe};

        assert_eq!(Safe.and(Safe), Safe);
        assert_eq!(Safe.and(Indeterminate), Indeterminate);
        assert_eq!(Indeterminate.and(Unsafe), Unsafe);
        assert_eq!(Unsafe.and(Safe), Unsafe);
    }

    #[test]
    fn maps_to_validity() {
        assert_eq!(Validity::from(SafetyVerdict::Safe), Validity::Valid);
        assert_eq!(Validity::from(SafetyVerdict::Unsafe), Validity::Invalid);
        assert_eq!(
            Validity::from(SafetyVerdict::Indeterminate),
            Validity::Unknown
        );
    }
}
