//! The three-valued result of a conservative safety check.

use polyzonotope::Intersection;
use sampling_planning::Validity;

/// The result of a conservative safety check.
///
/// Over-approximations can only prove safety. A check that cannot prove it
/// returns [`SafetyVerdict::Indeterminate`], never [`SafetyVerdict::Safe`].
#[derive(Debug, Clone, PartialEq)]
#[must_use]
pub enum SafetyVerdict {
    /// The reachable set provably avoids every unsafe region.
    ///
    /// A proof only with the `sound` feature; without it, up to floating-point rounding.
    Safe,
    /// Some reachable state provably lies in an unsafe region; carries that state.
    ///
    /// Never produced from an over-approximated set alone: a point of the enclosure inside
    /// an obstacle need not be reachable. Only a model's
    /// [`Reach::certify_collision`](crate::Reach::certify_collision) hook can produce it;
    /// without a certified witness the verdict stays [`SafetyVerdict::Indeterminate`].
    Unsafe(Witness),
    /// Neither could be proven; refine or reject.
    Indeterminate,
}

/// A certified collision: a state the real system reaches that lies in an obstacle.
///
/// Produced only from a point returned by
/// [`Reach::certify_collision`](crate::Reach::certify_collision).
#[derive(Debug, Clone, PartialEq)]
pub struct Witness {
    /// The colliding state; has the sets' dimension.
    pub point: Vec<f64>,
    /// Index of the cover set the state lies in, in the edge's time order.
    pub set: usize,
    /// Index of the obstacle in [`Obstacles::polytopes`](crate::Obstacles::polytopes).
    pub obstacle: usize,
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
    pub const fn is_safe(&self) -> bool {
        matches!(self, Self::Safe)
    }

    /// Combines two verdicts for a compound check: [`SafetyVerdict::Unsafe`] dominates
    /// [`SafetyVerdict::Indeterminate`], which dominates [`SafetyVerdict::Safe`].
    /// The left witness wins if both are unsafe.
    pub fn and(self, other: Self) -> Self {
        match (self, other) {
            (Self::Unsafe(witness), _) | (_, Self::Unsafe(witness)) => Self::Unsafe(witness),
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
            SafetyVerdict::Unsafe(_) => Self::Invalid,
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

        let unsafe_at = |obstacle| {
            Unsafe(Witness {
                point: vec![0.0, 0.0],
                set: 0,
                obstacle,
            })
        };

        assert_eq!(Safe.and(Safe), Safe);
        assert_eq!(Safe.and(Indeterminate), Indeterminate);
        assert_eq!(Indeterminate.and(unsafe_at(1)), unsafe_at(1));
        assert_eq!(unsafe_at(1).and(Safe), unsafe_at(1));
        assert_eq!(unsafe_at(1).and(unsafe_at(2)), unsafe_at(1));
    }

    #[test]
    fn maps_to_validity() {
        assert_eq!(Validity::from(SafetyVerdict::Safe), Validity::Valid);
        let witness = Witness {
            point: vec![0.0],
            set: 0,
            obstacle: 0,
        };

        assert_eq!(
            Validity::from(SafetyVerdict::Unsafe(witness)),
            Validity::Invalid
        );
        assert_eq!(
            Validity::from(SafetyVerdict::Indeterminate),
            Validity::Unknown
        );
    }
}
