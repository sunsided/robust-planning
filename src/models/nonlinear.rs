//! Nonlinear systems `x' = f(x, u)` on `polyzonotope-reach`'s [`NonlinearReach`].

use polyzonotope::{SparsePolyZonotope, Zonotope};
use polyzonotope_reach::{Dynamics, NonlinearReach, NonlinearSettings};

use crate::Reach;

/// A [`Reach`] model of the nonlinear system `x' = f(x, u)` with `N` states.
///
/// The control of an edge is one input set `U_k` per time step of
/// [`NonlinearSettings::step`]; see the [module docs](super). Each step linearizes or
/// polynomializes `f` around the current set ([`NonlinearSettings::method`]) and encloses
/// the abstraction error. A step polyzonotope-reach cannot prove (the set or the step is
/// too large for the abstraction, or the step for the Taylor bound) makes
/// [`Reach::propagate`] return `None`, so the edge is `Indeterminate`, never `Safe`.
///
/// Sets are stepped whole: [`NonlinearSettings::max_pieces`], `splitting`,
/// `merge_tolerance` and `adaptive` do not apply, since a split set has no single end set
/// to hand to the next edge.
///
/// # Examples
///
/// ```
/// use robust_planning::models::Nonlinear;
/// use nalgebra::{DMatrix, DVector};
/// use robust_planning::polyzonotope::{SparsePolyZonotope, Zonotope};
/// use robust_planning::polyzonotope_reach::{NonlinearSettings, Polynomial};
/// use robust_planning::Reach;
///
/// // `x' = u - x³` per axis, from the box `[-0.1, 0.1]²` under inputs `u ∈ [0.9, 1.1]²`.
/// let drag = Polynomial::new(2, 2)
///     .term(0, 1.0, &[0, 0, 1, 0])
///     .term(0, -1.0, &[3, 0, 0, 0])
///     .term(1, 1.0, &[0, 0, 0, 1])
///     .term(1, -1.0, &[0, 3, 0, 0]);
/// let model = Nonlinear::<_, 2>::new(drag, NonlinearSettings::new(0.1));
/// let start = SparsePolyZonotope::from_zonotope(&Zonotope::new(
///     DVector::zeros(2),
///     DMatrix::identity(2, 2) * 0.1,
/// ));
/// let input = Zonotope::new(DVector::from_element(2, 1.0), DMatrix::identity(2, 2) * 0.1);
///
/// let mut cover = Vec::new();
/// let end = model.propagate(&start, &vec![input; 5], &mut cover).expect("short steps are proven");
///
/// assert_eq!(cover.len(), 5);
/// assert!(end.interval_hull().0[0].lo > 0.0);
/// ```
pub struct Nonlinear<D, const N: usize> {
    /// The dynamics with their step settings.
    reach: NonlinearReach<D>,
}

impl<D: Dynamics, const N: usize> Nonlinear<D, N> {
    /// The model of `dynamics`, stepped with `settings`.
    ///
    /// # Panics
    ///
    /// Panics if `dynamics` does not have `N` states, or if `settings` are out of range; see
    /// [`NonlinearReach::new`].
    #[must_use]
    pub fn new(dynamics: D, settings: NonlinearSettings) -> Self {
        assert_eq!(dynamics.states(), N, "the dynamics need N states");

        Self {
            reach: NonlinearReach::new(dynamics, settings),
        }
    }
}

impl<D: Dynamics, const N: usize> Reach<N> for Nonlinear<D, N> {
    /// One input set per time step, each with one dimension per input of the dynamics.
    type Control = Vec<Zonotope>;

    /// # Panics
    ///
    /// Panics if `initial` does not have `N` dimensions or an input set does not have one
    /// per input of the dynamics.
    fn propagate(
        &self,
        initial: &SparsePolyZonotope,
        control: &Self::Control,
        cover: &mut Vec<SparsePolyZonotope>,
    ) -> Option<SparsePolyZonotope> {
        // shortcut: whole sets, fixed steps; split and adapt once polyzonotope can enclose
        // the union of pieces in one set.
        super::chain(initial, control, cover, |set, input| {
            self.reach.step(set, input).ok()
        })
    }
}
