//! Rewiring transactions on hand-built trees with a model whose set grows per edge.

use polyzonotope::{Interval, IntervalBox, Polytope, SparsePolyZonotope, Zonotope};
use sampling_planning::{StraightLine, UniformBox};

use super::{Config, Problem, Rejection, RobustRrtStar};
use crate::{Cost, Obstacles, PolytopeGoal, Reach, WithSteering};

/// How much the half width of the set grows on every edge.
const GROWTH: f64 = 0.1;

/// Jumps to the setpoint; the end set is the start set's hull, moved to the setpoint and
/// widened by [`GROWTH`]. The cover is the box around both, so it contains the jump.
struct Grow;

impl Reach<2> for Grow {
    type Control = [f64; 2];

    fn propagate(
        &self,
        initial: &SparsePolyZonotope,
        end: &[f64; 2],
        cover: &mut Vec<SparsePolyZonotope>,
    ) -> SparsePolyZonotope {
        let from = initial.interval_hull();
        let end_box = IntervalBox(
            (0..2)
                .map(|i| {
                    let half = from.0[i].width() / 2.0 + GROWTH;

                    Interval::new(end[i] - half, end[i] + half)
                })
                .collect(),
        );
        let hull = IntervalBox(
            (0..2)
                .map(|i| {
                    Interval::new(
                        from.0[i].lo.min(end_box.0[i].lo),
                        from.0[i].hi.max(end_box.0[i].hi),
                    )
                })
                .collect(),
        );

        cover.push(set(&hull));
        set(&end_box)
    }
}

/// The planner type of these tests.
type Planner = RobustRrtStar<WithSteering<Grow, StraightLine>, UniformBox<2>, 2>;

/// The box as a polynomial zonotope.
fn set(b: &IntervalBox) -> SparsePolyZonotope {
    SparsePolyZonotope::from_zonotope(&Zonotope::from_box(b))
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

/// The ids of the hand-built tree: two branches from the root, and `d` below `v`.
struct Tree {
    /// The detour branch `root -> (0, 1) -> v -> d`: `(0, 1)`.
    up: usize,
    /// The node `(1, 1)`, two edges from the root, set half width 0.25.
    v: usize,
    /// The node `(1, 2)`, the child of `v`, set half width 0.35.
    d: usize,
    /// The node `(0.75, 0.75)` at the end of a three-edge diagonal branch, half width 0.35.
    /// Under it `v` would be cheaper but wider: half width 0.45, and `d` 0.55.
    p: usize,
}

/// A planner around `obstacles` with `goal`, a start set of half width 0.05 at the origin,
/// and the tree of [`Tree`] built by hand.
fn hand_built(obstacles: Vec<Polytope>, goal: &IntervalBox) -> (Planner, Tree) {
    hand_built_with(obstacles, goal, Cost::Length)
}

/// [`hand_built`] with edges priced by `cost`.
fn hand_built_with(obstacles: Vec<Polytope>, goal: &IntervalBox, cost: Cost) -> (Planner, Tree) {
    let problem = Problem {
        reach: WithSteering::new(Grow, StraightLine::new(10.0)),
        obstacles: Obstacles::new(obstacles, 4),
        sampler: UniformBox::new([0.0, 0.0], [3.0, 3.0]),
        start: [0.0, 0.0],
        start_set: set(&square([0.0, 0.0], 0.05)),
        goal: PolytopeGoal::from_box(goal, 4),
    };
    let config = Config {
        cost,
        ..Config::new(1.0, 1.0)
    };
    let mut planner = Planner::new(problem, config).expect("valid");

    let up = attach(&mut planner, 0, [0.0, 1.0]);
    let v = attach(&mut planner, up, [1.0, 1.0]);
    let d = attach(&mut planner, v, [1.0, 2.0]);
    let diagonal = attach(&mut planner, 0, [0.25, 0.25]);
    let diagonal = attach(&mut planner, diagonal, [0.5, 0.5]);
    let p = attach(&mut planner, diagonal, [0.75, 0.75]);

    (planner, Tree { up, v, d, p })
}

/// Adds a node at `nominal` under `parent`; panics if the edge is not proven safe.
fn attach(planner: &mut Planner, parent: usize, nominal: [f64; 2]) -> usize {
    let edge = planner
        .connect(parent, &nominal)
        .expect("hand-built edge is safe");

    planner.insert(edge, nominal)
}

/// A goal far from every node.
fn far_goal() -> IntervalBox {
    square([2.5, 2.5], 0.3)
}

/// Everything a rewire may change, per node.
type Snapshot = Vec<(
    Option<usize>,
    Option<[f64; 2]>,
    IntervalBox,
    f64,
    Vec<usize>,
    bool,
)>;

/// The parent, control, set hull, cost, children and goal flag of every node.
fn snapshot(planner: &Planner) -> Snapshot {
    (0..planner.nodes.len())
        .map(|id| {
            let node = &planner.nodes[id];

            (
                node.parent,
                node.control,
                node.set.interval_hull(),
                planner.costs[id],
                planner.children[id].clone(),
                planner.in_goal[id],
            )
        })
        .collect()
}

/// Rewires `node` under `parent` with `budget` and checks that a rejection changes nothing.
fn rewire(
    planner: &mut Planner,
    node: usize,
    parent: usize,
    budget: usize,
) -> Result<(), Rejection> {
    let before = snapshot(planner);
    let mut budget = budget;
    let outcome = planner.try_rewire(node, parent, &mut budget);

    if outcome.is_err() {
        assert_eq!(
            snapshot(planner),
            before,
            "a rejected rewire changed the tree"
        );
    }

    outcome
}

/// The half width of the hull of node `id`'s set along the first axis.
fn half_width(planner: &Planner, id: usize) -> f64 {
    planner.nodes[id].set.interval_hull().0[0].width() / 2.0
}

#[test]
fn a_descendant_that_becomes_unproven_rejects_the_rewire() {
    // Clear of d's old set (x <= 1.35) and of v's new set (y <= 1.45), but not of d's new
    // set (x up to 1.55, y from 1.45).
    let wall = Polytope::from_box(&IntervalBox(vec![
        Interval::new(1.45, 2.0),
        Interval::new(1.5, 2.5),
    ]));
    let (mut planner, tree) = hand_built(vec![wall], &far_goal());

    assert_eq!(
        rewire(&mut planner, tree.v, tree.p, 64),
        Err(Rejection::DescendantUnproven)
    );
}

#[test]
fn a_goal_node_that_would_lose_the_goal_rejects_the_rewire() {
    let (mut planner, tree) = hand_built(Vec::new(), &square([1.0, 2.0], 0.45));

    assert!(
        planner.in_goal[tree.d],
        "d's set (half width 0.35) is inside"
    );
    assert_eq!(
        rewire(&mut planner, tree.v, tree.p, 64),
        Err(Rejection::GoalLost)
    );
}

#[test]
fn an_unproven_edge_to_the_new_parent_rejects_the_rewire() {
    let (mut planner, tree) = hand_built(Vec::new(), &far_goal());
    let wall = Polytope::from_box(&square([0.5, 1.5], 0.1));
    planner.problem.obstacles = Obstacles::new(vec![wall], 4);

    assert_eq!(
        rewire(&mut planner, tree.up, tree.d, 64),
        Err(Rejection::Cycle)
    );
    assert_eq!(
        rewire(&mut planner, tree.d, tree.up, 64),
        Err(Rejection::EdgeUnproven),
        "the edge (0, 1) -> (1, 2) crosses the wall"
    );
}

#[test]
fn steering_that_stops_short_rejects_the_rewire() {
    let (mut planner, tree) = hand_built(Vec::new(), &far_goal());
    // p -> v is about 0.354 long; this steering stops after 0.1.
    planner.problem.reach.steering = StraightLine::new(0.1);

    assert_eq!(
        rewire(&mut planner, tree.v, tree.p, 64),
        Err(Rejection::NotReached)
    );
}

#[test]
fn a_subtree_over_the_budget_is_skipped() {
    let (mut planner, tree) = hand_built(Vec::new(), &far_goal());

    assert_eq!(
        rewire(&mut planner, tree.v, tree.p, 0),
        Err(Rejection::OverBudget)
    );
    assert_eq!(
        rewire(&mut planner, tree.v, tree.p, 1),
        Err(Rejection::OverBudget)
    );
}

#[test]
fn a_committed_rewire_re_propagates_the_subtree() {
    let (mut planner, tree) = hand_built(Vec::new(), &far_goal());
    let mut budget = 2;

    assert_eq!(planner.try_rewire(tree.v, tree.p, &mut budget), Ok(()));
    assert_eq!(budget, 0, "one check per subtree node");
    assert_eq!(planner.nodes[tree.v].parent, Some(tree.p));
    assert!(!planner.children[tree.up].contains(&tree.v));
    assert!(planner.children[tree.p].contains(&tree.v));
    assert!((half_width(&planner, tree.v) - 0.45).abs() < 1e-9);
    assert!((half_width(&planner, tree.d) - 0.55).abs() < 1e-9);

    let path = crate::node::nominal_path(&planner.nodes, tree.d);
    assert_eq!(
        planner.costs[tree.d], path.cost,
        "costs are the nominal path length"
    );
    assert!((path.cost - (2.0f64.sqrt() + 1.0)).abs() < 1e-12);
}

#[test]
fn invalid_configs_are_rejected() {
    use super::Error;

    let problem = || Problem {
        reach: WithSteering::new(Grow, StraightLine::new(1.0)),
        obstacles: Obstacles::new(Vec::new(), 0),
        sampler: UniformBox::new([0.0, 0.0], [1.0, 1.0]),
        start: [0.0, 0.0],
        start_set: set(&square([0.0, 0.0], 0.05)),
        goal: PolytopeGoal::from_box(&far_goal(), 0),
    };
    let error = |config| Planner::new(problem(), config).err();
    let base = Config::new(1.0, 1.0);

    assert_eq!(
        error(Config {
            goal_bias: 1.5,
            ..base
        }),
        Some(Error::GoalBiasOutOfRange)
    );
    assert_eq!(
        error(Config { gamma: 0.0, ..base }),
        Some(Error::InvalidGamma)
    );
    assert_eq!(
        error(Config {
            gamma: f64::INFINITY,
            ..base
        }),
        Some(Error::InvalidGamma)
    );
    assert_eq!(
        error(Config {
            max_radius: 0.0,
            ..base
        }),
        Some(Error::InvalidMaxRadius)
    );
    assert_eq!(
        error(Config {
            max_radius: f64::NAN,
            ..base
        }),
        Some(Error::InvalidMaxRadius)
    );
    assert_eq!(
        error(Config {
            cost: Cost::WidthWeighted { weight: -0.1 },
            ..base
        }),
        Some(Error::InvalidCostWeight)
    );
    assert_eq!(
        error(Config {
            max_radius: f64::INFINITY,
            ..base
        }),
        None
    );
}

#[test]
fn a_weighted_rewire_that_makes_a_descendant_dearer_is_rejected() {
    // Width costs 0.3 per unit: v gets cheaper through p (2.134 < 2.24), but d's wider set
    // makes it dearer (3.464 > 3.45).
    let weighted = Cost::WidthWeighted { weight: 0.3 };
    let (mut planner, tree) = hand_built_with(Vec::new(), &far_goal(), weighted);

    assert_eq!(
        rewire(&mut planner, tree.v, tree.p, 64),
        Err(Rejection::CostIncreased)
    );
}

#[test]
fn a_weighted_rewire_whose_wider_set_is_not_cheaper_is_rejected() {
    // At 0.5 per unit, v through p costs 2.614, more than its 2.4 now, although the nominal
    // length alone would be shorter.
    let weighted = Cost::WidthWeighted { weight: 0.5 };
    let (mut planner, tree) = hand_built_with(Vec::new(), &far_goal(), weighted);

    assert!((planner.costs[tree.v] - 2.4).abs() < 1e-9);
    assert_eq!(
        rewire(&mut planner, tree.v, tree.p, 64),
        Err(Rejection::NotCheaper)
    );
}
