//! Linear systems `x' = A x + B u` on `polyzonotope-reach`'s [`LinearReach`].

use polyzonotope::{Reduction, SparsePolyZonotope, Zonotope};
use polyzonotope_reach::{LinearReach, LinearSystem, ReachError};

use crate::Reach;

/// A [`Reach`] model of the linear system `x' = A x + B u` with `N` states.
///
/// The control of an edge is one input set `U_k` per time step of the configured length;
/// see the [module docs](super). Every step is proven, so [`Reach::propagate`] always
/// returns an end set. The carried time-point set is reduced to the configured order after
/// each step (Girard), so the sets stay small along long edges.
///
/// # Examples
///
/// ```
/// use robust_planning::models::Linear;
/// use robust_planning::polyzonotope::{SparsePolyZonotope, Zonotope};
/// use robust_planning::polyzonotope_reach::LinearSystem;
/// use robust_planning::Reach;
/// use nalgebra::{DMatrix, DVector};
///
/// // A single integrator `x' = u` with inputs `u ∈ [0.9, 1.1]²`, 4 steps of 0.25.
/// let system = LinearSystem { a: DMatrix::zeros(2, 2), b: DMatrix::identity(2, 2) };
/// let model = Linear::<2>::new(system, 0.25, 4, 8.0)?;
/// let input = Zonotope::new(DVector::from_element(2, 1.0), DMatrix::identity(2, 2) * 0.1);
/// let start = SparsePolyZonotope::from_zonotope(&Zonotope::new(
///     DVector::zeros(2),
///     DMatrix::zeros(2, 0),
/// ));
///
/// let mut cover = Vec::new();
/// let end = model.propagate(&start, &vec![input; 4], &mut cover).expect("linear steps are proven");
/// let hull = end.interval_hull();
///
/// assert_eq!(cover.len(), 4);
/// assert!(hull.0[0].lo <= 0.9 && hull.0[0].hi >= 1.1);
/// # Ok::<(), robust_planning::polyzonotope_reach::ReachError>(())
/// ```
pub struct Linear<const N: usize> {
    /// One step of the system, precomputed for the configured step length.
    reach: LinearReach,
    /// Zonotope order (generators per dimension, at least 1) the time-point set is reduced
    /// to after every step.
    order: f64,
}

impl<const N: usize> Linear<N> {
    /// The model of `system` in steps of `step`, with `taylor_terms` terms of the matrix
    /// exponential after the first, reducing to zonotope order `order` after every step.
    ///
    /// # Errors
    ///
    /// [`ReachError::InvalidStep`] if `step` is negative, infinite or NaN, and
    /// [`ReachError::StepTooLarge`] if the Taylor remainder bound does not hold for this step
    /// and term count; see [`LinearReach::new`].
    ///
    /// # Panics
    ///
    /// Panics if `A` is not `N × N`, if `B` does not have `N` rows, or if `order` is below 1
    /// or NaN.
    pub fn new(
        system: LinearSystem,
        step: f64,
        taylor_terms: usize,
        order: f64,
    ) -> Result<Self, ReachError> {
        assert_eq!(system.a.nrows(), N, "A needs one row per state");
        assert!(order >= 1.0, "order must be at least 1");

        Ok(Self {
            reach: LinearReach::new(system, step, taylor_terms)?,
            order,
        })
    }
}

impl<const N: usize> Reach<N> for Linear<N> {
    /// One input set per time step, each with one dimension per column of `B`.
    type Control = Vec<Zonotope>;

    /// # Panics
    ///
    /// Panics if `initial` does not have `N` dimensions or an input set does not have one
    /// per column of `B`.
    fn propagate(
        &self,
        initial: &SparsePolyZonotope,
        control: &Self::Control,
        cover: &mut Vec<SparsePolyZonotope>,
    ) -> Option<SparsePolyZonotope> {
        super::chain(initial, control, cover, |set, input| {
            let mut step = self.reach.step(set, input);
            step.time_point = step.time_point.reduce(self.order, Reduction::Girard);

            Some(step)
        })
    }
}
