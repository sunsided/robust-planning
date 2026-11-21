//! Steering: how a planner picks the control for one edge.
//!
//! Steering is split from [`Reach`] so that the same reachability model can be driven by
//! different steering functions. A model that knows how to steer itself implements both
//! traits. A model that only propagates sets is paired with a steering function through
//! [`WithSteering`].

use polyzonotope::{Polytope, SparsePolyZonotope};
use sampling_planning::{Euclidean, Steering, StraightLine};

use crate::Reach;

/// Picks the control of type `C` that moves a nominal state toward a target.
///
/// Planners never trust steering for safety: every control is propagated by a [`Reach`]
/// model and checked against the obstacles, so a poor steering function costs progress, not
/// soundness.
pub trait Steer<const N: usize, C> {
    /// Picks the control that moves the nominal state `from` toward `toward`.
    ///
    /// Returns the control and the nominal end state, which may stop short of `toward`.
    fn steer(&self, from: &[f64; N], toward: &[f64; N]) -> (C, [f64; N]);
}

/// Steers toward the target in a straight line and hands the end point to the model as its
/// control.
///
/// Fits models whose control is a setpoint, that is, models whose [`Reach::Control`] is the
/// nominal end state `[f64; N]` and which track it with their own controller. The end point
/// is `toward` itself when it is at most [`StraightLine::max_step`] away, so edges toward
/// near targets end exactly at the target.
impl<const N: usize> Steer<N, [f64; N]> for StraightLine {
    fn steer(&self, from: &[f64; N], toward: &[f64; N]) -> ([f64; N], [f64; N]) {
        let end = Steering::steer(self, &Euclidean::<N>, from, toward);

        (end, end)
    }
}

/// A reachability model paired with a separate steering function.
///
/// Implements [`Reach`] by delegating to [`WithSteering::reach`] and [`Steer`] by delegating
/// to [`WithSteering::steering`], so it can be used wherever a planner expects a model that
/// steers itself.
///
/// # Examples
///
/// ```
/// use robust_planning::{Reach, Steer, WithSteering};
/// use robust_planning::polyzonotope::SparsePolyZonotope;
/// use robust_planning::sampling_planning::StraightLine;
///
/// /// A model whose control is the nominal end state.
/// struct Setpoint;
///
/// impl Reach<2> for Setpoint {
///     type Control = [f64; 2];
///
///     fn propagate(
///         &self,
///         initial: &SparsePolyZonotope,
///         _: &[f64; 2],
///         cover: &mut Vec<SparsePolyZonotope>,
///     ) -> Option<SparsePolyZonotope> {
///         cover.push(initial.clone());
///         Some(initial.clone())
///     }
/// }
///
/// let model = WithSteering::new(Setpoint, StraightLine::new(1.0));
/// let (control, end) = model.steer(&[0.0, 0.0], &[3.0, 4.0]);
///
/// assert_eq!(control, end);
/// assert!((end[0] - 0.6).abs() < 1e-12 && (end[1] - 0.8).abs() < 1e-12);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct WithSteering<R, S> {
    /// The model that propagates reachable sets.
    pub reach: R,
    /// The steering function that picks each edge's control.
    pub steering: S,
}

impl<R, S> WithSteering<R, S> {
    /// Pairs `reach` with `steering`.
    #[must_use]
    pub const fn new(reach: R, steering: S) -> Self {
        Self { reach, steering }
    }
}

impl<R: Reach<N>, S, const N: usize> Reach<N> for WithSteering<R, S> {
    /// The control of the wrapped model.
    type Control = R::Control;

    fn propagate(
        &self,
        initial: &SparsePolyZonotope,
        control: &Self::Control,
        cover: &mut Vec<SparsePolyZonotope>,
    ) -> Option<SparsePolyZonotope> {
        self.reach.propagate(initial, control, cover)
    }

    /// Forwards to the wrapped model, so pairing a model with a steering function keeps its
    /// collision certificates.
    fn certify_collision(
        &self,
        initial: &SparsePolyZonotope,
        control: &Self::Control,
        set: &SparsePolyZonotope,
        obstacle: &Polytope,
    ) -> Option<Vec<f64>> {
        self.reach
            .certify_collision(initial, control, set, obstacle)
    }
}

impl<R, S: Steer<N, C>, C, const N: usize> Steer<N, C> for WithSteering<R, S> {
    fn steer(&self, from: &[f64; N], toward: &[f64; N]) -> (C, [f64; N]) {
        self.steering.steer(from, toward)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_line_reaches_near_targets_exactly_and_clamps_far_ones() {
        let steering = StraightLine::new(1.0);

        assert_eq!(
            Steer::steer(&steering, &[0.0, 0.0], &[0.6, 0.8]),
            ([0.6, 0.8], [0.6, 0.8])
        );

        let (control, end) = Steer::steer(&steering, &[0.0, 0.0], &[30.0, 40.0]);
        assert_eq!(control, end);
        assert!((end[0] - 0.6).abs() < 1e-12 && (end[1] - 0.8).abs() < 1e-12);
    }
}
