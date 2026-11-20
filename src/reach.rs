//! Reachable sets along one edge: the seam to the dynamics model.

use polyzonotope::SparsePolyZonotope;

use crate::{Obstacles, SafetyVerdict};

/// Steers a system and over-approximates its reachable set along one edge.
///
/// A placeholder until `polyzonotope-reach` provides validated reachability; implement it for
/// your dynamics model. States are points in `N` dimensions; the reachable sets live in the
/// same space.
///
/// # Contract
///
/// [`Reach::propagate`] must over-approximate: every state the real system can occupy while
/// it executes the control, including between time steps, must lie in one of the cover sets,
/// and every state it can end in must lie in the returned end set. Planners prove safety from
/// these sets only, so an under-approximation makes their proofs void.
pub trait Reach<const N: usize> {
    /// What drives the system along one edge, for example a sequence of nominal inputs.
    type Control: Clone;

    /// Picks the control that moves the nominal state `from` toward `toward`.
    ///
    /// Returns the control and the nominal end state, which may stop short of `toward`.
    fn steer(&self, from: &[f64; N], toward: &[f64; N]) -> (Self::Control, [f64; N]);

    /// Propagates `initial` under `control`.
    ///
    /// Appends sets to `cover` that together contain every state on the way, from the start
    /// of the edge to its end, in time order. Returns the set of states at the end of the
    /// edge, which becomes the next edge's `initial`.
    fn propagate(
        &self,
        initial: &SparsePolyZonotope,
        control: &Self::Control,
        cover: &mut Vec<SparsePolyZonotope>,
    ) -> SparsePolyZonotope;
}

/// Propagates `initial` under `control` and checks the cover against `obstacles`.
///
/// Clears `cover` first and leaves the edge's cover sets in it. Returns the verdict and the
/// end set. An empty cover is [`SafetyVerdict::Indeterminate`]: a model that reports no sets
/// has not shown where the system goes.
pub(crate) fn check_edge<R: Reach<N>, const N: usize>(
    reach: &R,
    obstacles: &Obstacles,
    initial: &SparsePolyZonotope,
    control: &R::Control,
    cover: &mut Vec<SparsePolyZonotope>,
) -> (SafetyVerdict, SparsePolyZonotope) {
    cover.clear();
    let end = reach.propagate(initial, control, cover);

    let verdict = if cover.is_empty() {
        SafetyVerdict::Indeterminate
    } else {
        obstacles.check_sets(cover)
    };

    (verdict, end)
}
