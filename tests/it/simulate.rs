//! Monte-Carlo simulation of the affine system along tree branches.

use nalgebra::DVector;
use polyzonotope::{IntervalBox, Polytope};
use rand_pcg::Pcg64Mcg;
use sampling_planning::unit_f64;

use robust_planning::Node;

use crate::{affine::Affine, world::hits};

/// Points checked on each segment between two time steps, ends excluded.
const SEGMENT_POINTS: u32 = 8;

/// Simulates one random trajectory of `model` along the branch from the root to `node` of a
/// planner tree.
///
/// Draws the start state from the root set's interval hull (the start box) and an input error
/// and a disturbance per step, a quarter of them at a corner of their box. Panics if a state
/// or a point between two states lies in an obstacle, or if a state at a node lies outside
/// that node's interval hull.
pub fn trajectory(
    model: &Affine<2>,
    nodes: &[Node<Vec<[f64; 2]>, 2>],
    obstacles: &[Polytope],
    node: usize,
    rng: &mut Pcg64Mcg,
) {
    let mut x = point_in(&nodes[0].set.interval_hull(), rng);

    for id in branch(nodes, node) {
        let inputs = nodes[id]
            .control
            .as_ref()
            .expect("non-root nodes have a control");

        for u in inputs {
            let next = advance(model, &x, u, rng);
            assert_segment_free(obstacles, &x, &next);
            x = next;
        }

        assert!(
            nodes[id]
                .set
                .interval_hull()
                .contains(&DVector::from_column_slice(&x), 1e-9),
            "state {x:?} escaped the set of node {id}"
        );
    }
}

/// The node ids from the root's first child to `node`, inclusive.
fn branch(nodes: &[Node<Vec<[f64; 2]>, 2>], node: usize) -> Vec<usize> {
    let mut ids: Vec<usize> = core::iter::successors(Some(node), |&id| nodes[id].parent).collect();
    ids.pop();
    ids.reverse();

    ids
}

/// One step of `x' = A x + B (u + e) + w` with random `e` and `w`.
fn advance(model: &Affine<2>, x: &[f64; 2], u: &[f64; 2], rng: &mut Pcg64Mcg) -> [f64; 2] {
    let input = DVector::from_fn(2, |i, _| u[i] + perturbation(model.input_error[i], rng));
    let disturbance = DVector::from_fn(2, |i, _| perturbation(model.disturbance[i], rng));
    let next = &model.a * DVector::from_column_slice(x) + &model.b * input + disturbance;

    [next[0], next[1]]
}

/// Panics if `from`, `to` or a point between them lies in an obstacle.
fn assert_segment_free(obstacles: &[Polytope], from: &[f64; 2], to: &[f64; 2]) {
    for k in 0..=SEGMENT_POINTS + 1 {
        let t = f64::from(k) / f64::from(SEGMENT_POINTS + 1);
        let point = [
            from[0] + (to[0] - from[0]) * t,
            from[1] + (to[1] - from[1]) * t,
        ];

        assert!(
            !hits(obstacles, &point),
            "trajectory hit an obstacle at {point:?}"
        );
    }
}

/// A random point of `hull`, each coordinate offset from the midpoint by [`perturbation`].
fn point_in(hull: &IntervalBox, rng: &mut Pcg64Mcg) -> [f64; 2] {
    let mut coordinate = |i: usize| {
        let half = hull.0[i].width() / 2.0;

        hull.0[i].lo + half + perturbation(half, rng)
    };

    [coordinate(0), coordinate(1)]
}

/// A random value in `[-half_width, half_width]`: one of the two ends with probability 1/4,
/// otherwise uniform.
fn perturbation(half_width: f64, rng: &mut Pcg64Mcg) -> f64 {
    let pick = unit_f64(rng);

    if pick < 0.125 {
        -half_width
    } else if pick < 0.25 {
        half_width
    } else {
        half_width * (2.0 * unit_f64(rng) - 1.0)
    }
}
