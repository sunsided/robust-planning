//! A discrete-time affine system `x' = A x + B (u + e) + w` with bounded input error `e` and
//! disturbance `w`, and its reachable sets `X' = A X ⊕ B (u ⊕ E) ⊕ W`.
//!
//! A stand-in for `polyzonotope-reach`, shared by the tests and the example. Between time
//! steps the system is assumed to move on the straight segment from `x` to `x'`, so the box
//! around two consecutive sets covers it.

use nalgebra::{DMatrix, DVector};
use polyzonotope::{Interval, IntervalBox, Reduction, SparsePolyZonotope, Zonotope};
use robust_planning::Reach;

/// The affine system and the steering parameters of one edge.
#[derive(Debug, Clone)]
pub struct Affine<const N: usize> {
    /// State matrix `A`.
    pub a: DMatrix<f64>,
    /// Input matrix `B`. Must be invertible: steering solves `B u = r' - A r`.
    pub b: DMatrix<f64>,
    /// Half widths of the input error box `E` around each nominal input.
    pub input_error: [f64; N],
    /// Half widths of the disturbance box `W`, centered at the origin.
    pub disturbance: [f64; N],
    /// Largest nominal distance covered per time step. Positive.
    pub step: f64,
    /// Most time steps per edge. Positive.
    pub max_steps: u32,
    /// Zonotope order each set is reduced to after a step. At least 1.
    pub order: f64,
}

impl<const N: usize> Affine<N> {
    /// A system with `A = contraction · I` and `B = I`: tracking errors shrink by
    /// `contraction` per step while input error and disturbance add up to their half widths.
    pub fn contracting(contraction: f64, input_error: f64, disturbance: f64) -> Self {
        Self {
            a: DMatrix::from_diagonal_element(N, N, contraction),
            b: DMatrix::identity(N, N),
            input_error: [input_error; N],
            disturbance: [disturbance; N],
            step: 0.25,
            max_steps: 4,
            order: 4.0,
        }
    }

    /// The nominal states of one edge: `steps + 1` points on the segment from `from` toward
    /// `toward`, at most [`Affine::step`] apart and at most [`Affine::max_steps`] steps long.
    /// The last point is exactly `toward` when the segment is short enough.
    fn reference(&self, from: &[f64; N], toward: &[f64; N]) -> Vec<[f64; N]> {
        let distance = distance(from, toward);
        let reach = self.step * f64::from(self.max_steps);
        let (end, length) = if distance <= reach {
            (*toward, distance)
        } else {
            (lerp(from, toward, reach / distance), reach)
        };
        let steps = step_count(length / self.step, self.max_steps);

        let mut points: Vec<[f64; N]> = (0..steps)
            .map(|k| lerp(from, &end, f64::from(k) / f64::from(steps)))
            .collect();
        points.push(end);

        points
    }

    /// The nominal input that moves the reference from `r` to `next`: `B⁻¹ (next - A r)`.
    fn input(&self, r: &[f64; N], next: &[f64; N]) -> [f64; N] {
        let rhs = DVector::from_column_slice(next) - &self.a * DVector::from_column_slice(r);
        let u = self.b.clone().lu().solve(&rhs).expect("B is invertible");

        core::array::from_fn(|i| u[i])
    }

    /// The set after one step from `set` under nominal input `u`.
    fn advance(&self, set: &SparsePolyZonotope, u: &[f64; N]) -> SparsePolyZonotope {
        let inputs = SparsePolyZonotope::from_zonotope(&centered_box(u, &self.input_error));
        let disturbance =
            SparsePolyZonotope::from_zonotope(&centered_box(&[0.0; N], &self.disturbance));

        set.linear_map(&self.a)
            .minkowski_sum(&inputs.linear_map(&self.b))
            .minkowski_sum(&disturbance)
            .reduce(self.order, Reduction::Girard)
    }
}

impl<const N: usize> Reach<N> for Affine<N> {
    /// One nominal input per time step.
    type Control = Vec<[f64; N]>;

    fn steer(&self, from: &[f64; N], toward: &[f64; N]) -> (Self::Control, [f64; N]) {
        let reference = self.reference(from, toward);
        let inputs = reference
            .windows(2)
            .map(|pair| self.input(&pair[0], &pair[1]))
            .collect();
        let end = *reference
            .last()
            .expect("the reference has at least two points");

        (inputs, end)
    }

    fn propagate(
        &self,
        initial: &SparsePolyZonotope,
        control: &Self::Control,
        cover: &mut Vec<SparsePolyZonotope>,
    ) -> SparsePolyZonotope {
        let mut set = initial.clone();

        for u in control {
            let next = self.advance(&set, u);
            cover.push(hull_box(&set, &next));
            set = next;
        }

        set
    }
}

/// The box `center ± half_widths` as a zonotope: center `center`, one axis-aligned generator
/// per half width. Built without arithmetic, so no rounding narrows it.
fn centered_box<const N: usize>(center: &[f64; N], half_widths: &[f64; N]) -> Zonotope {
    let generators = DMatrix::from_diagonal(&DVector::from_column_slice(half_widths));

    Zonotope::new(DVector::from_column_slice(center), generators)
}

/// The smallest axis-aligned box around both sets, from their outward-rounded interval hulls.
///
/// Contains the segment between any point of `a` and any point of `b`, since the box is
/// convex.
fn hull_box(a: &SparsePolyZonotope, b: &SparsePolyZonotope) -> SparsePolyZonotope {
    let (ha, hb) = (a.interval_hull(), b.interval_hull());
    let union =
        ha.0.iter()
            .zip(&hb.0)
            .map(|(x, y)| Interval::new(x.lo.min(y.lo), x.hi.max(y.hi)));

    SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&IntervalBox(union.collect())))
}

/// The number of steps for `ratio` step lengths: at least one, at most `max_steps`.
fn step_count(ratio: f64, max_steps: u32) -> u32 {
    (1..=max_steps)
        .find(|&k| f64::from(k) >= ratio)
        .unwrap_or(max_steps)
}

/// The Euclidean distance between two points.
fn distance<const N: usize>(a: &[f64; N], b: &[f64; N]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y) * (x - y))
        .sum::<f64>()
        .sqrt()
}

/// The point at fraction `t` of the way from `a` to `b`.
fn lerp<const N: usize>(a: &[f64; N], b: &[f64; N], t: f64) -> [f64; N] {
    core::array::from_fn(|i| a[i] + (b[i] - a[i]) * t)
}
