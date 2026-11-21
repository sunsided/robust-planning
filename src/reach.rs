//! Reachable sets along one edge: the seam to the dynamics model.

use polyzonotope::{Polytope, SparsePolyZonotope};

use crate::{Obstacles, SafetyVerdict, Witness};

/// Over-approximates the reachable set of a system along one edge.
///
/// Implement it for your dynamics model, for example on top of `polyzonotope-reach`, whose
/// time-interval sets are cover sets and whose time-point sets are end sets. States are points in `N` dimensions; the reachable sets live in the
/// same space. Planners also need a [`Steer`](crate::Steer) implementation for
/// [`Reach::Control`]: implement it on the model, or pair the model with a steering function
/// through [`WithSteering`](crate::WithSteering).
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

    /// Certifies a state the real system reaches in `set` that lies inside `obstacle`.
    ///
    /// `set` is one of the cover sets [`Reach::propagate`] produced from `initial` under
    /// `control`, which identify the edge.
    /// The hook behind [`SafetyVerdict::Unsafe`]. Edge checks call it only for edges that
    /// are not proven safe, once per cover set and obstacle, and turn the first point it
    /// returns into a [`Witness`]. The default returns `None`, so such edges stay
    /// [`SafetyVerdict::Indeterminate`].
    ///
    /// Return `Some(point)` only if `point` is proven to be a state the real system can
    /// actually reach on the edge, and proven to lie in `obstacle`. Cover sets are
    /// over-approximations, so a point of `set` is in general *not* such a state; only a
    /// model whose cover set is exact or under-approximated can certify one. polyzonotope
    /// has no certified witness, so this is for models that prove it themselves.
    ///
    /// The returned point is sanity-checked in plain `f64` (inside `obstacle` and inside the
    /// interval hull of `set`) and dropped if it fails, which keeps the verdict
    /// `Indeterminate`. That check is a guard against hook bugs, not a proof.
    fn certify_collision(
        &self,
        initial: &SparsePolyZonotope,
        control: &Self::Control,
        set: &SparsePolyZonotope,
        obstacle: &Polytope,
    ) -> Option<Vec<f64>> {
        let _ = (initial, control, set, obstacle);

        None
    }
}

/// Propagates `initial` under `control` and checks the cover against `obstacles`.
///
/// Clears `cover` first and leaves the edge's cover sets in it. Returns the verdict and the
/// end set. The verdict is [`SafetyVerdict::Safe`] only if every cover set is proven
/// disjoint from every obstacle. An edge that is not proven safe is
/// [`SafetyVerdict::Unsafe`] if [`Reach::certify_collision`] certifies a collision, and
/// [`SafetyVerdict::Indeterminate`] otherwise. An empty cover is `Indeterminate`: a model
/// that reports no sets has not shown where the system goes.
///
/// # Panics
///
/// Panics if the model's sets do not have the obstacles' dimension.
pub fn check_edge<R: Reach<N>, const N: usize>(
    reach: &R,
    obstacles: &Obstacles,
    initial: &SparsePolyZonotope,
    control: &R::Control,
    cover: &mut Vec<SparsePolyZonotope>,
) -> (SafetyVerdict, SparsePolyZonotope) {
    cover.clear();
    let end = reach.propagate(initial, control, cover);

    if cover.is_empty() {
        return (SafetyVerdict::Indeterminate, end);
    }

    let verdict = match obstacles.check_sets(cover) {
        SafetyVerdict::Safe => SafetyVerdict::Safe,
        unproven => find_witness(reach, obstacles, initial, control, cover)
            .map_or(unproven, SafetyVerdict::Unsafe),
    };

    (verdict, end)
}

/// The first collision [`Reach::certify_collision`] certifies over every cover set and
/// obstacle, in time order, if it passes [`is_plausible`].
fn find_witness<R: Reach<N>, const N: usize>(
    reach: &R,
    obstacles: &Obstacles,
    initial: &SparsePolyZonotope,
    control: &R::Control,
    cover: &[SparsePolyZonotope],
) -> Option<Witness> {
    cover.iter().enumerate().find_map(|(set_index, set)| {
        obstacles
            .polytopes()
            .iter()
            .enumerate()
            .find_map(|(obstacle_index, obstacle)| {
                let point = reach.certify_collision(initial, control, set, obstacle)?;

                is_plausible(&point, set, obstacle).then_some(Witness {
                    point,
                    set: set_index,
                    obstacle: obstacle_index,
                })
            })
    })
}

/// Whether `point` has the set's dimension and lies, in plain `f64`, inside `obstacle` and
/// inside the interval hull of `set`.
///
/// A guard against hooks that return garbage, not a proof: near a boundary it may reject a
/// correctly certified point, which only costs the witness.
fn is_plausible(point: &[f64], set: &SparsePolyZonotope, obstacle: &Polytope) -> bool {
    let hull = set.interval_hull();
    let in_hull = point.len() == hull.dim()
        && point
            .iter()
            .zip(&hull.0)
            .all(|(x, range)| range.contains(*x, 0.0));
    let in_obstacle = obstacle.facets.iter().all(|h| {
        h.normal.len() == point.len()
            && h.normal.iter().zip(point).map(|(a, x)| a * x).sum::<f64>() <= h.offset
    });

    in_hull && in_obstacle
}
