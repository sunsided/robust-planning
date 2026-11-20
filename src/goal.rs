//! Goal regions over reachable sets: a node reaches the goal only when its whole set is
//! proven inside.

use polyzonotope::{Containment, IntervalBox, Polytope, SparsePolyZonotope};

/// A goal region that whole reachable sets are tested against.
///
/// The set counterpart of `sampling-planning`'s
/// [`GoalRegion`](sampling_planning::GoalRegion): a [`Goal::center`] for goal-biased
/// samples, and a containment test that takes a set instead of a state.
///
/// # Contract
///
/// [`Goal::containment`] may answer [`Containment::Inside`] only if every point of the set
/// is proven to lie in the goal. Planners accept nothing else as reaching the goal, so a
/// wrong `Inside` makes their goal proofs void. When in doubt, answer
/// [`Containment::Unknown`].
pub trait Goal<const N: usize> {
    /// The state drawn for goal-biased samples, usually inside the goal.
    fn center(&self) -> [f64; N];

    /// Whether every point of `set` lies in the goal.
    ///
    /// Only [`Containment::Inside`] is a proof; planners treat
    /// [`Containment::NotInside`] and [`Containment::Unknown`] alike.
    fn containment(&self, set: &SparsePolyZonotope) -> Containment;

    /// An upper bound on the distance from [`Goal::center`] to any point of the goal, if
    /// the goal knows one.
    ///
    /// [`ReachSearch`](crate::ReachSearch) derives its heuristic radius from it with
    /// [`GoalRadius::FromGoal`](crate::search::GoalRadius::FromGoal). It affects only the
    /// search order, never safety. The default is `None`.
    fn radius(&self) -> Option<f64> {
        None
    }
}

/// A convex polytope goal in H-representation, `{ x | aᵢᵀ x <= dᵢ for every facet i }`.
///
/// Containment is decided by [`SparsePolyZonotope::containment`] with a split budget of
/// [`PolytopeGoal::max_splits`]. It is a proof with the `sound` feature, and up to
/// floating-point rounding without it.
///
/// # Examples
///
/// ```
/// use robust_planning::{Goal, PolytopeGoal};
/// use robust_planning::polyzonotope::{
///     Containment, Interval, IntervalBox, SparsePolyZonotope, Zonotope,
/// };
///
/// let square = |lo: f64, hi: f64| IntervalBox(vec![Interval::new(lo, hi); 2]);
/// let goal = PolytopeGoal::<2>::from_box(&square(0.0, 4.0), 8);
/// let set = |b| SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&b));
///
/// assert_eq!(goal.center(), [2.0, 2.0]);
/// assert_eq!(goal.containment(&set(square(1.0, 2.0))), Containment::Inside);
/// assert_ne!(goal.containment(&set(square(3.0, 5.0))), Containment::Inside);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct PolytopeGoal<const N: usize> {
    /// The goal region. Every facet normal has `N` finite entries and every offset is finite.
    polytope: Polytope,
    /// The state drawn for goal-biased samples.
    center: [f64; N],
    /// Split budget for proving a set inside; `0` disables refinement.
    max_splits: usize,
}

impl<const N: usize> PolytopeGoal<N> {
    /// Creates a goal from its facets, the state for goal-biased samples and a split budget.
    ///
    /// # Panics
    ///
    /// Panics if a facet normal does not have `N` entries, or if a normal entry or an offset
    /// is not finite. A NaN facet would never refute containment, so it is rejected here
    /// instead of silently proving every set inside.
    #[must_use]
    pub fn new(polytope: Polytope, center: [f64; N], max_splits: usize) -> Self {
        for facet in &polytope.facets {
            assert_eq!(facet.normal.len(), N, "facet normals must have {N} entries");
            assert!(
                facet.normal.iter().all(|a| a.is_finite()) && facet.offset.is_finite(),
                "facet normals and offsets must be finite"
            );
        }

        Self {
            polytope,
            center,
            max_splits,
        }
    }

    /// Creates a box goal, centered for goal-biased samples at the box midpoint.
    ///
    /// # Panics
    ///
    /// Panics if the box does not have `N` dimensions or has an infinite bound.
    #[must_use]
    pub fn from_box(b: &IntervalBox, max_splits: usize) -> Self {
        assert_eq!(b.dim(), N, "the box must have {N} dimensions");
        let center = core::array::from_fn(|i| b.0[i].lo + b.0[i].width() / 2.0);

        Self::new(Polytope::from_box(b), center, max_splits)
    }

    /// The goal region.
    #[must_use]
    pub const fn polytope(&self) -> &Polytope {
        &self.polytope
    }

    /// Split budget for proving a set inside.
    #[must_use]
    pub const fn max_splits(&self) -> usize {
        self.max_splits
    }
}

impl<const N: usize> Goal<N> for PolytopeGoal<N> {
    fn center(&self) -> [f64; N] {
        self.center
    }

    /// The distance from the center to the farthest corner of the box that the polytope's
    /// axis-aligned facets bound, or `None` if some axis is not bounded on both sides by
    /// such a facet.
    ///
    /// Facets with other normals only cut the polytope further, so the box contains it and
    /// the radius is an upper bound. It is widened by a few ULPs against rounding.
    fn radius(&self) -> Option<f64> {
        let (lower, upper) = axis_bounds::<N>(&self.polytope);
        let squared: f64 = (0..N)
            .map(|i| {
                let reach = (self.center[i] - lower[i]).max(upper[i] - self.center[i]);

                reach * reach
            })
            .sum();
        let radius = squared.sqrt() * (1.0 + 8.0 * f64::EPSILON);

        radius.is_finite().then_some(radius)
    }

    /// Delegates to [`SparsePolyZonotope::containment`] within the split budget.
    ///
    /// # Panics
    ///
    /// Panics if `set` does not have `N` dimensions.
    fn containment(&self, set: &SparsePolyZonotope) -> Containment {
        set.containment(&self.polytope, self.max_splits)
    }
}

/// The tightest lower and upper bound per axis among the facets of `polytope` whose normal
/// has exactly one non-zero entry; infinite where no such facet bounds that side.
fn axis_bounds<const N: usize>(polytope: &Polytope) -> ([f64; N], [f64; N]) {
    let mut lower = [f64::NEG_INFINITY; N];
    let mut upper = [f64::INFINITY; N];

    for facet in &polytope.facets {
        let mut nonzero = facet.normal.iter().enumerate().filter(|(_, a)| **a != 0.0);
        let (Some((axis, &a)), None) = (nonzero.next(), nonzero.next()) else {
            continue;
        };
        let bound = facet.offset / a;

        if a > 0.0 {
            upper[axis] = upper[axis].min(bound);
        } else {
            lower[axis] = lower[axis].max(bound);
        }
    }

    (lower, upper)
}

#[cfg(test)]
mod tests {
    use nalgebra::DVector;
    use polyzonotope::{Halfspace, Interval, Zonotope};

    use super::*;

    /// The box `[lo, hi]` in every axis as a set.
    fn square_set(lo: f64, hi: f64) -> SparsePolyZonotope {
        SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&IntervalBox(vec![
            Interval::new(
                lo, hi
            );
            2
        ])))
    }

    /// The unit box goal `[0, 1]²`.
    fn unit_goal() -> PolytopeGoal<2> {
        PolytopeGoal::from_box(&IntervalBox(vec![Interval::new(0.0, 1.0); 2]), 8)
    }

    /// The triangle `x >= 0, y >= 0, x + y <= 1`, a goal with a facet no box has.
    fn triangle() -> PolytopeGoal<2> {
        let facet = |a: f64, b: f64, offset: f64| Halfspace {
            normal: DVector::from_column_slice(&[a, b]),
            offset,
        };
        let polytope = Polytope {
            facets: vec![
                facet(-1.0, 0.0, 0.0),
                facet(0.0, -1.0, 0.0),
                facet(1.0, 1.0, 1.0),
            ],
        };

        PolytopeGoal::new(polytope, [0.25, 0.25], 8)
    }

    #[test]
    fn set_well_inside_is_contained() {
        assert_eq!(unit_goal().center(), [0.5, 0.5]);
        assert_eq!(
            unit_goal().containment(&square_set(0.25, 0.75)),
            Containment::Inside
        );
    }

    #[test]
    fn straddling_set_is_not_contained() {
        assert_ne!(
            unit_goal().containment(&square_set(0.5, 1.5)),
            Containment::Inside
        );
    }

    #[test]
    fn set_one_ulp_outside_is_not_contained() {
        assert_ne!(
            unit_goal().containment(&square_set(0.5, 1.0_f64.next_up())),
            Containment::Inside
        );
    }

    #[test]
    fn slanted_facet_decides_containment() {
        let goal = triangle();

        assert_eq!(goal.containment(&square_set(0.1, 0.3)), Containment::Inside);
        assert_ne!(
            goal.containment(&square_set(0.35, 0.55)),
            Containment::Inside,
            "the corner (0.55, 0.55) is beyond x + y <= 1"
        );
    }

    #[test]
    #[should_panic(expected = "finite")]
    fn nan_offset_is_rejected() {
        let mut polytope = unit_goal().polytope().clone();
        polytope.facets[0].offset = f64::NAN;

        let _ = PolytopeGoal::new(polytope, [0.5, 0.5], 8);
    }

    #[test]
    #[should_panic(expected = "entries")]
    fn wrong_normal_length_is_rejected() {
        let polytope = Polytope::from_box(&IntervalBox(vec![Interval::new(0.0, 1.0); 3]));

        let _ = PolytopeGoal::<2>::new(polytope, [0.5, 0.5], 8);
    }

    #[test]
    fn radius_bounds_the_box_and_needs_axis_facets() {
        let unit = unit_goal().radius().expect("a box bounds every axis");

        assert!(unit >= 0.5f64.sqrt() && unit < 0.5f64.sqrt() + 1e-12);
        assert_eq!(
            triangle().radius(),
            None,
            "x + y <= 1 bounds no axis from above"
        );

        let mut cut = unit_goal();
        cut.polytope.facets.extend(triangle().polytope.facets);
        let radius = cut.radius().expect("the box facets are still there");
        assert!(
            radius >= 0.5f64.sqrt(),
            "extra facets never shrink the bound below the box"
        );
    }
}
