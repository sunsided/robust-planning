//! The polyzonotope-reach models: sampled trajectories stay in their sets, the linear model
//! agrees with the affine stand-in, and refused steps never make an edge safe.

use nalgebra::{DMatrix, DVector};
use polyzonotope::{Intersection, Interval, IntervalBox, Polytope, SparsePolyZonotope, Zonotope};
use polyzonotope_reach::{LinearSystem, NonlinearSettings, Polynomial};
use rand_pcg::Pcg64Mcg;
use robust_planning::models::{Linear, Nonlinear};
use robust_planning::{Obstacles, Reach, SafetyVerdict, check_edge};
use sampling_planning::{rand_core::SeedableRng, unit_f64};

use crate::affine::Affine;

/// Length of one time step of every model here.
const STEP: f64 = 0.1;

/// Integration substeps per time step when simulating a trajectory.
const SUBSTEPS: u32 = 10;

/// Half width of the box around a simulated state that must meet the set it should lie in;
/// covers the integration error of the simulation.
const TOLERANCE: f64 = 1e-7;

/// Splits allowed when testing whether a simulated state meets a set. A state inside the set
/// uses them all (its tiny box rarely holds a witness), so the budget sets the test's cost.
const SPLITS: usize = 2;

/// A vector field `f(x, u)` of two states and two inputs.
type Field = fn(&[f64; 2], &[f64; 2]) -> [f64; 2];

/// The damped rotation `x' = A x + u` with `A = [[-0.2, 1], [-1, -0.2]]`.
fn rotation() -> LinearSystem {
    LinearSystem {
        a: DMatrix::from_row_slice(2, 2, &[-0.2, 1.0, -1.0, -0.2]),
        b: DMatrix::identity(2, 2),
    }
}

/// The vector field of [`rotation`].
fn rotation_field(x: &[f64; 2], u: &[f64; 2]) -> [f64; 2] {
    [-0.2 * x[0] + x[1] + u[0], -x[0] - 0.2 * x[1] + u[1]]
}

/// The cubic drag `x' = u - x³` per axis, over the variables `(x₀, x₁, u₀, u₁)`.
fn drag() -> Polynomial {
    Polynomial::new(2, 2)
        .term(0, 1.0, &[0, 0, 1, 0])
        .term(0, -1.0, &[3, 0, 0, 0])
        .term(1, 1.0, &[0, 0, 0, 1])
        .term(1, -1.0, &[0, 3, 0, 0])
}

/// The vector field of [`drag`].
fn drag_field(x: &[f64; 2], u: &[f64; 2]) -> [f64; 2] {
    [u[0] - x[0].powi(3), u[1] - x[1].powi(3)]
}

/// The box `center ± half_width` in both axes.
fn square(center: [f64; 2], half_width: f64) -> IntervalBox {
    IntervalBox(
        center
            .iter()
            .map(|c| Interval::new(c - half_width, c + half_width))
            .collect(),
    )
}

/// The box `center ± half_width` as a set.
fn set(center: [f64; 2], half_width: f64) -> SparsePolyZonotope {
    SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&square(center, half_width)))
}

/// The input set `center ± half_width` in both axes.
fn input(center: [f64; 2], half_width: f64) -> Zonotope {
    Zonotope::new(
        DVector::from_column_slice(&center),
        DMatrix::from_diagonal_element(2, 2, half_width),
    )
}

/// A point of the interval `center ± half_width`, at an end a quarter of the time.
fn draw(center: f64, half_width: f64, rng: &mut Pcg64Mcg) -> f64 {
    let t = 2.0 * unit_f64(rng) - 1.0;
    let t = if unit_f64(rng) < 0.25 { t.signum() } else { t };

    center + half_width * t
}

/// A point of the box `center ± half_width`.
fn draw_point(center: [f64; 2], half_width: f64, rng: &mut Pcg64Mcg) -> [f64; 2] {
    [
        draw(center[0], half_width, rng),
        draw(center[1], half_width, rng),
    ]
}

/// One classical Runge-Kutta step of length `h` of `field` under the constant input `u`.
fn rk4(field: Field, x: &[f64; 2], u: &[f64; 2], h: f64) -> [f64; 2] {
    let at = |x: &[f64; 2], k: &[f64; 2], s: f64| [x[0] + s * k[0], x[1] + s * k[1]];
    let k1 = field(x, u);
    let k2 = field(&at(x, &k1, h / 2.0), u);
    let k3 = field(&at(x, &k2, h / 2.0), u);
    let k4 = field(&at(x, &k3, h), u);

    core::array::from_fn(|i| x[i] + h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]))
}

/// Panics unless the box of half width [`TOLERANCE`] around `x` meets `set`: a proven
/// [`Intersection::Disjoint`] means the state escaped it.
fn assert_meets(set: &SparsePolyZonotope, x: &[f64; 2], what: &str) {
    let point = Polytope::from_box(&square(*x, TOLERANCE));

    assert_ne!(
        set.intersection(&point, SPLITS),
        Intersection::Disjoint,
        "{what}: state {x:?} escaped"
    );
}

/// Simulates `trajectories` random runs of `field` from the box `start ± start_radius`
/// under inputs drawn from `inputs`, a new one per substep, and checks every state against
/// the cover set of its time step and the final state against the end set.
fn assert_trajectories_inside(
    model: &impl Reach<2, Control = Vec<Zonotope>>,
    field: Field,
    (start, start_radius): ([f64; 2], f64),
    (center, error, steps): ([f64; 2], f64, usize),
    trajectories: u64,
) {
    let mut cover = Vec::new();
    let end = model
        .propagate(
            &set(start, start_radius),
            &vec![input(center, error); steps],
            &mut cover,
        )
        .expect("every step is proven");
    assert_eq!(cover.len(), steps);

    for seed in 0..trajectories {
        let rng = &mut Pcg64Mcg::seed_from_u64(seed);
        let mut x = draw_point(start, start_radius, rng);

        for (k, cover_set) in cover.iter().enumerate() {
            assert_meets(cover_set, &x, &format!("seed {seed}, start of step {k}"));

            for _ in 0..SUBSTEPS {
                let u = draw_point(center, error, rng);
                x = rk4(field, &x, &u, STEP / f64::from(SUBSTEPS));
                assert_meets(cover_set, &x, &format!("seed {seed}, step {k}"));
            }
        }

        assert_meets(&end, &x, &format!("seed {seed}, end set"));
    }
}

#[test]
fn linear_sets_contain_sampled_trajectories() {
    let model = Linear::<2>::new(rotation(), STEP, 4, 20.0).expect("the step is valid");

    assert_trajectories_inside(
        &model,
        rotation_field,
        ([1.0, 0.0], 0.1),
        ([0.5, 0.0], 0.1, 10),
        8,
    );
}

#[test]
fn nonlinear_sets_contain_sampled_trajectories() {
    let model = Nonlinear::<_, 2>::new(drag(), NonlinearSettings::new(STEP));

    assert_trajectories_inside(
        &model,
        drag_field,
        ([0.0, 0.5], 0.1),
        ([1.0, 0.0], 0.1, 8),
        8,
    );
}

#[test]
fn linear_end_set_contains_the_affine_stand_ins() {
    // `x' = u` over one step of length r is exactly `x + r u`: the stand-in with `A = I` and
    // `B = r I` computes the same set, and with a high order it never reduces.
    let integrator = LinearSystem {
        a: DMatrix::zeros(2, 2),
        b: DMatrix::identity(2, 2),
    };
    let model = Linear::<2>::new(integrator, STEP, 4, 50.0).expect("the step is valid");
    let stand_in = Affine::<2> {
        a: DMatrix::identity(2, 2),
        b: DMatrix::identity(2, 2) * STEP,
        input_error: [0.1; 2],
        disturbance: [0.0; 2],
        step: 1.0,
        max_steps: 6,
        order: 50.0,
    };
    let start = set([1.0, 2.0], 0.1);
    let nominal = [0.5, -0.3];
    let (mut model_cover, mut stand_in_cover) = (Vec::new(), Vec::new());

    let model_end = model
        .propagate(&start, &vec![input(nominal, 0.1); 6], &mut model_cover)
        .expect("linear steps are proven");
    let stand_in_end = stand_in
        .propagate(&start, &vec![nominal; 6], &mut stand_in_cover)
        .expect("the stand-in always encloses its edges");
    let (outer, inner) = (model_end.interval_hull(), stand_in_end.interval_hull());

    for (o, i) in outer.0.iter().zip(&inner.0) {
        assert!(
            o.lo <= i.lo + 1e-12 && o.hi >= i.hi - 1e-12,
            "{o:?} misses {i:?}"
        );
    }
}

#[test]
fn edges_through_an_obstacle_are_never_safe() {
    let model = Linear::<2>::new(rotation(), STEP, 4, 20.0).expect("the step is valid");
    let control = vec![input([0.5, 0.0], 0.05); 10];
    let start = set([1.0, 0.0], 0.05);
    let mut cover = Vec::new();
    let check = |obstacle: IntervalBox, cover: &mut Vec<SparsePolyZonotope>| {
        let obstacles = Obstacles::new(vec![Polytope::from_box(&obstacle)], 8);

        check_edge(&model, &obstacles, &start, &control, cover).0
    };

    // A small box around the middle of the set after half the edge lies on the path.
    let midway = model
        .propagate(&start, &control[..5].to_vec(), &mut Vec::new())
        .expect("linear steps are proven")
        .interval_hull();
    let on_path = IntervalBox(
        midway
            .0
            .iter()
            .map(|r| Interval::new(r.lo.midpoint(r.hi) - 0.01, r.lo.midpoint(r.hi) + 0.01))
            .collect(),
    );

    assert_ne!(check(on_path, &mut cover), SafetyVerdict::Safe);
    assert_eq!(
        check(square([5.0, 5.0], 0.5), &mut cover),
        SafetyVerdict::Safe
    );
}

#[test]
fn refused_steps_leave_the_edge_indeterminate() {
    // A wide set and a long step are too much for the linearization of the cubic drag.
    let model = Nonlinear::<_, 2>::new(drag(), NonlinearSettings::new(2.0));
    let start = set([0.0, 0.0], 3.0);
    let control = vec![input([1.0, 0.0], 0.1); 3];
    let clear = Obstacles::new(vec![Polytope::from_box(&square([50.0, 50.0], 1.0))], 4);

    assert_eq!(model.propagate(&start, &control, &mut Vec::new()), None);

    let (verdict, end) = check_edge(&model, &clear, &start, &control, &mut Vec::new());

    assert_eq!(verdict, SafetyVerdict::Indeterminate);
    assert_eq!(end, None);
}
