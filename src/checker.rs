//! A [`ValidityChecker`] that proves motions safe with reachable sets.

use polyzonotope::{Containment, Interval, IntervalBox, Polytope, SparsePolyZonotope, Zonotope};
use sampling_planning::{Euclidean, Validity, ValidityChecker};

use crate::{Obstacles, Reach, SafetyVerdict, Steer, check_edge};

/// Checks states and motions of `sampling-planning`'s planners with reachable sets.
///
/// `sampling-planning` hands a checker only two bare states per motion, so this adapter
/// assumes a fixed uncertainty *tube*: the real state at every tree node lies in the box
/// `state ± half_widths`. It then proves each motion against that assumption:
///
/// 1. the model's nominal steering from `from` toward `to` ends exactly at `to`;
/// 2. every set covering the motion, propagated from the box at `from`, is proven disjoint
///    from every obstacle, at every intermediate time step;
/// 3. the end set is proven inside the box at `to`, so the assumption holds at the next node
///    as well and edges chain soundly.
///
/// Only then is the motion [`Validity::Valid`]. Anything unproven is [`Validity::Unknown`]
/// and rejected by the planners. For uncertainty that grows along a branch, use
/// [`RobustRrt`](crate::RobustRrt), which keeps the propagated set per node.
#[derive(Debug, Clone)]
pub struct ReachChecker<R, const N: usize> {
    /// The obstacles and their split budget.
    obstacles: Obstacles,
    /// The dynamics model that steers and propagates.
    reach: R,
    /// Half widths of the uncertainty box around every node state. Finite and non-negative.
    half_widths: [f64; N],
}

impl<R: Reach<N>, const N: usize> ReachChecker<R, N> {
    /// Creates a checker with the box `state ± half_widths` as the uncertainty at each node.
    ///
    /// The split budget of `obstacles` also bounds the containment check of end sets.
    ///
    /// # Panics
    ///
    /// Panics if a half width is negative or not finite.
    #[must_use]
    pub fn new(obstacles: Obstacles, reach: R, half_widths: [f64; N]) -> Self {
        assert!(
            half_widths.iter().all(|r| r.is_finite() && *r >= 0.0),
            "half widths must be non-negative and finite"
        );

        Self {
            obstacles,
            reach,
            half_widths,
        }
    }

    /// The obstacles.
    #[must_use]
    pub const fn obstacles(&self) -> &Obstacles {
        &self.obstacles
    }

    /// The dynamics model.
    #[must_use]
    pub const fn reach(&self) -> &R {
        &self.reach
    }

    /// The uncertainty box around `state`.
    ///
    /// # Panics
    ///
    /// Panics if `state` has a NaN coordinate.
    #[must_use]
    pub fn tube(&self, state: &[f64; N]) -> IntervalBox {
        IntervalBox(
            state
                .iter()
                .zip(&self.half_widths)
                .map(|(x, r)| Interval::new(x - r, x + r))
                .collect(),
        )
    }

    /// Whether `end` is proven inside the uncertainty box around `to`.
    fn returns_into_tube(&self, end: &SparsePolyZonotope, to: &[f64; N]) -> Validity {
        let tube = Polytope::from_box(&self.tube(to));

        match end.containment(&tube, self.obstacles.max_splits()) {
            Containment::Inside => Validity::Valid,
            Containment::NotInside | Containment::Unknown => Validity::Unknown,
        }
    }
}

impl<R: Reach<N> + Steer<N, R::Control>, const N: usize> ValidityChecker<Euclidean<N>>
    for ReachChecker<R, N>
{
    /// Valid only if the uncertainty box around `state` is proven disjoint from every obstacle.
    fn check_state(&self, state: &[f64; N]) -> Validity {
        let tube = SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&self.tube(state)));

        self.obstacles.check_set(&tube).into()
    }

    /// Valid only if the motion is proven safe and returns into the tube at `to`; see the
    /// type docs.
    fn check_motion(&self, _: &Euclidean<N>, from: &[f64; N], to: &[f64; N]) -> Validity {
        let (control, end) = self.reach.steer(from, to);

        #[expect(
            clippy::float_cmp,
            reason = "the motion must end exactly at `to`, where the next tube is centered"
        )]
        let misses_target = end != *to;

        if misses_target {
            return Validity::Unknown;
        }

        let initial = SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&self.tube(from)));
        let mut cover = Vec::new();
        let (verdict, end_set) =
            check_edge(&self.reach, &self.obstacles, &initial, &control, &mut cover);

        if verdict != SafetyVerdict::Safe {
            return verdict.into();
        }

        self.returns_into_tube(&end_set, to)
    }
}
