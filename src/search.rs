//! Reachability-guided search: A* over reachable-set nodes on a lattice of nominal states.
//!
//! Each node carries a nominal state and the reachable set the system can be in there. To
//! expand a node, every motion primitive (an offset from the nominal state) is steered
//! toward, the node's set is propagated along the edge, and the edge is kept only if every
//! set covering it is proven disjoint from every obstacle. Nodes are expanded in order of
//! `f = g + h`: `g` is the nominal path length, `h` a conservative lower bound on the
//! remaining length (see [`Config::goal_radius`]). A node reaches the goal when its whole
//! set is proven inside the goal region; the first such node popped is returned.
//!
//! ```mermaid
//! graph LR
//!     Pop["pop lowest f"] --> Goal{"set inside goal?"}
//!     Goal -->|yes| Done
//!     Goal -->|no| Prim["for each primitive: steer"]
//!     Prim --> Check["propagate and check cover"]
//!     Check -->|indeterminate or unsafe| Prim
//!     Check -->|safe| Dom{"dominated in its cell?"}
//!     Dom -->|yes| Prim
//!     Dom -->|no| Push["push with f = g + h"]
//!     Push --> Pop
//! ```
//!
//! # Deduplication
//!
//! Nominal states are binned into lattice cells of width [`Config::resolution`]. A new node
//! is dropped when a node already in its cell has a cost at most as high and an interval
//! hull inside the new node's interval hull. Conversely, a new node supersedes the older
//! nodes in its cell that it dominates, and they are skipped when popped. This is a heuristic dominance test: proving
//! one polynomial zonotope inside another is not available in polyzonotope 0.2, so the
//! hulls stand in for the sets. Pruning never affects safety, since every kept edge is
//! still proven, but it can drop a node whose set is thinner than its hull and so cost
//! completeness in tight passages.
//!
//! # Examples
//!
//! ```
//! use robust_planning::polyzonotope::{
//!     Interval, IntervalBox, Polytope, SparsePolyZonotope, Zonotope,
//! };
//! use robust_planning::sampling_planning::StraightLine;
//! use robust_planning::search::{Config, Problem, ReachSearch};
//! use robust_planning::{Obstacles, PolytopeGoal, Reach, WithSteering};
//!
//! /// Jumps to the setpoint along a straight line, ending within 0.1 of it.
//! struct Snap;
//!
//! impl Reach<2> for Snap {
//!     type Control = [f64; 2];
//!
//!     fn propagate(
//!         &self,
//!         initial: &SparsePolyZonotope,
//!         end: &[f64; 2],
//!         cover: &mut Vec<SparsePolyZonotope>,
//!     ) -> SparsePolyZonotope {
//!         let from = initial.interval_hull();
//!         let hull = (0..2).map(|i| {
//!             Interval::new(from.0[i].lo.min(end[i] - 0.1), from.0[i].hi.max(end[i] + 0.1))
//!         });
//!         let end_box = (0..2).map(|i| Interval::new(end[i] - 0.1, end[i] + 0.1));
//!         let set = |b: IntervalBox| SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&b));
//!
//!         cover.push(set(IntervalBox(hull.collect())));
//!         set(IntervalBox(end_box.collect()))
//!     }
//! }
//!
//! let square = |x: f64, y: f64, r: f64| {
//!     IntervalBox(vec![Interval::new(x - r, x + r), Interval::new(y - r, y + r)])
//! };
//! let primitives = vec![[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]];
//! let problem = Problem {
//!     reach: WithSteering::new(Snap, StraightLine::new(2.0)),
//!     // A wall at x in [1.5, 2.5] for y <= 1.5 forces a detour through y = 2.
//!     obstacles: Obstacles::new(vec![Polytope::from_box(&IntervalBox(vec![
//!         Interval::new(1.5, 2.5),
//!         Interval::new(-5.0, 1.5),
//!     ]))], 4),
//!     start: [0.0, 0.0],
//!     start_set: SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&square(0.0, 0.0, 0.1))),
//!     goal: PolytopeGoal::from_box(&square(4.0, 0.0, 0.3), 4),
//!     primitives,
//! };
//! let config = Config { goal_radius: 0.0, ..Config::default() };
//! let mut search = ReachSearch::new(problem, config).expect("valid start and config");
//! let path = search.solve().expect("the detour is proven safe");
//!
//! assert!((path.cost - 8.0).abs() < 1e-9, "up 2, across 4, down 2");
//! ```

use core::{cmp::Ordering, fmt};
use std::collections::{BinaryHeap, HashMap};

use polyzonotope::{Containment, IntervalBox, SparsePolyZonotope};
use sampling_planning::{Euclidean, Path, StateSpace};

use crate::{Goal, Node, Obstacles, PolytopeGoal, Reach, SafetyVerdict, Steer, check_edge, node};

/// Everything [`ReachSearch`] needs to know about the world.
///
/// The goal defaults to a [`PolytopeGoal`]; any [`Goal`] works.
#[derive(Debug, Clone)]
pub struct Problem<R, const N: usize, G = PolytopeGoal<N>> {
    /// The dynamics model that steers and propagates.
    pub reach: R,
    /// The obstacles and their split budget.
    pub obstacles: Obstacles,
    /// The nominal start state, which steering starts from.
    pub start: [f64; N],
    /// Every state the system may start in. Must be proven safe.
    pub start_set: SparsePolyZonotope,
    /// The goal region. A node reaches it when its set is proven inside.
    pub goal: G,
    /// The motion primitives: offsets from a node's nominal state that steering aims at.
    ///
    /// Grid vectors that the model reaches exactly (for example the 4- or 8-neighborhood at
    /// one step length) keep nominal states on a lattice, so deduplication can merge them.
    pub primitives: Vec<[f64; N]>,
}

/// Settings for [`ReachSearch`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    /// Width of the lattice cells nominal states are binned into for deduplication.
    /// Positive and finite.
    pub resolution: f64,
    /// Distance from [`Goal::center`] within which every goal node's nominal state lies.
    /// Non-negative; may be infinite.
    ///
    /// The heuristic is `h(x) = max(0, |x - center| - goal_radius)`, which is consistent
    /// for Euclidean edge costs. It is admissible, and the returned path optimal over the
    /// nodes deduplication keeps, only if every node whose set is proven inside the goal
    /// has its nominal state within `goal_radius` of the center. An infinite radius gives
    /// `h = 0`, a uniform-cost search that is always admissible but expands more nodes.
    /// The heuristic never affects safety.
    pub goal_radius: f64,
    /// The most nodes [`ReachSearch::solve`] expands before giving up.
    pub max_expansions: usize,
}

impl Default for Config {
    /// A resolution of `1e-6` (merging only rounding noise), an infinite goal radius
    /// (uniform-cost search) and at most 10 000 expansions.
    fn default() -> Self {
        Self {
            resolution: 1e-6,
            goal_radius: f64::INFINITY,
            max_expansions: 10_000,
        }
    }
}

/// Why a [`ReachSearch`] could not be created.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// The start set is not proven safe; carries the verdict.
    StartNotSafe(SafetyVerdict),
    /// [`Config::resolution`] is not positive and finite.
    ResolutionOutOfRange,
    /// [`Config::goal_radius`] is negative or NaN.
    GoalRadiusOutOfRange,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::StartNotSafe(verdict) => write!(f, "start set is not proven safe ({verdict:?})"),
            Self::ResolutionOutOfRange => f.write_str("resolution must be positive and finite"),
            Self::GoalRadiusOutOfRange => f.write_str("goal radius must be non-negative"),
        }
    }
}

impl core::error::Error for Error {}

/// An entry of the open list: a node id and its priority `f = g + h`.
///
/// Ordered so that [`BinaryHeap`], a max-heap, pops the lowest `f` first, and among equal
/// `f` the oldest node, which makes runs deterministic.
#[derive(Debug, Clone, Copy)]
struct Entry {
    /// `g + h` of the node. Never NaN.
    f: f64,
    /// The node id.
    id: usize,
}

impl Ord for Entry {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f.total_cmp(&self.f).then(other.id.cmp(&self.id))
    }
}

impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for Entry {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for Entry {}

/// A* over reachable-set nodes; see the [module docs](self).
///
/// Every edge in the search tree is proven safe over its whole duration, every
/// intermediate time step included, from the reachable set of its parent.
pub struct ReachSearch<R: Reach<N>, const N: usize, G = PolytopeGoal<N>> {
    /// The problem being solved.
    problem: Problem<R, N, G>,
    /// Settings.
    config: Config,
    /// Search-tree nodes by id; node `0` is the root.
    nodes: Vec<Node<R::Control, N>>,
    /// Nominal path length from the root, by node id.
    costs: Vec<f64>,
    /// Interval hull of each node's set, by node id; used for deduplication.
    hulls: Vec<IntervalBox>,
    /// Node ids by lattice cell of their nominal state.
    cells: HashMap<[i64; N], Vec<usize>>,
    /// Nodes generated but not expanded yet.
    open: BinaryHeap<Entry>,
    /// By node id: whether a later node in the same cell dominates it, so popping it is
    /// skipped.
    superseded: Vec<bool>,
    /// Nodes expanded so far.
    expansions: usize,
    /// The node whose set was proven inside the goal, once popped.
    goal_node: Option<usize>,
    /// Scratch buffer for the cover sets of the edge being checked.
    cover: Vec<SparsePolyZonotope>,
}

impl<R, const N: usize, G> ReachSearch<R, N, G>
where
    R: Reach<N> + Steer<N, R::Control>,
    G: Goal<N>,
{
    /// Checks the start set and the settings and pushes the root.
    ///
    /// # Errors
    ///
    /// - [`Error::ResolutionOutOfRange`] if the resolution is not positive and finite.
    /// - [`Error::GoalRadiusOutOfRange`] if the goal radius is negative or NaN.
    /// - [`Error::StartNotSafe`] if the start set is not proven disjoint from every obstacle.
    pub fn new(problem: Problem<R, N, G>, config: Config) -> Result<Self, Error> {
        if !(config.resolution.is_finite() && config.resolution > 0.0) {
            return Err(Error::ResolutionOutOfRange);
        }

        if config.goal_radius.is_nan() || config.goal_radius < 0.0 {
            return Err(Error::GoalRadiusOutOfRange);
        }

        let verdict = problem.obstacles.check_set(&problem.start_set);

        if !verdict.is_safe() {
            return Err(Error::StartNotSafe(verdict));
        }

        let root = Node {
            nominal: problem.start,
            set: problem.start_set.clone(),
            parent: None,
            control: None,
        };

        let mut search = Self {
            problem,
            config,
            nodes: Vec::new(),
            costs: Vec::new(),
            hulls: Vec::new(),
            cells: HashMap::new(),
            open: BinaryHeap::new(),
            superseded: Vec::new(),
            expansions: 0,
            goal_node: None,
            cover: Vec::new(),
        };
        let hull = root.set.interval_hull();
        search.push(root, 0.0, hull);

        Ok(search)
    }

    /// Searches until a goal node is popped, the open list runs empty or
    /// [`Config::max_expansions`] nodes are expanded.
    ///
    /// Returns the nominal path to the goal, or `None` if none was found. Calling it again
    /// after a budget stop continues with a fresh budget.
    ///
    /// # Panics
    ///
    /// Panics if the model's sets do not have the obstacles' dimension.
    pub fn solve(&mut self) -> Option<Path<[f64; N]>> {
        let budget = self.expansions + self.config.max_expansions;

        while self.goal_node.is_none() && self.expansions < budget {
            let Some(Entry { id, .. }) = self.open.pop() else {
                break;
            };

            if self.superseded[id] {
                continue;
            }

            if self.reaches_goal(id) {
                self.goal_node = Some(id);
            } else {
                self.expand(id);
            }
        }

        self.path()
    }

    /// All search-tree nodes by id; node `0` is the root.
    #[must_use]
    pub fn nodes(&self) -> &[Node<R::Control, N>] {
        &self.nodes
    }

    /// The nominal path length from the root to `node`.
    ///
    /// # Panics
    ///
    /// Panics if `node` is not a node id.
    #[must_use]
    pub fn cost(&self, node: usize) -> f64 {
        self.costs[node]
    }

    /// The node whose set was proven inside the goal, if one was popped.
    #[must_use]
    pub const fn goal_node(&self) -> Option<usize> {
        self.goal_node
    }

    /// The number of nodes expanded so far.
    #[must_use]
    pub const fn expansions(&self) -> usize {
        self.expansions
    }

    /// Whether every generated node has been expanded, popped as the goal or superseded
    /// by a dominating node.
    #[must_use]
    pub fn is_exhausted(&self) -> bool {
        self.open.iter().all(|entry| self.superseded[entry.id])
    }

    /// The problem being solved.
    #[must_use]
    pub const fn problem(&self) -> &Problem<R, N, G> {
        &self.problem
    }

    /// The node ids from the root to `node`, inclusive.
    ///
    /// # Panics
    ///
    /// Panics if `node` is not a node id.
    #[must_use]
    pub fn branch(&self, node: usize) -> Vec<usize> {
        node::branch(&self.nodes, node)
    }

    /// The nominal path to the goal node, or `None` if the goal was not reached yet.
    ///
    /// The cost is the Euclidean length of the nominal path.
    #[must_use]
    pub fn path(&self) -> Option<Path<[f64; N]>> {
        Some(node::nominal_path(&self.nodes, self.goal_node?))
    }

    /// Generates every proven-safe, non-dominated successor of node `id`.
    fn expand(&mut self, id: usize) {
        self.expansions += 1;

        for k in 0..self.problem.primitives.len() {
            if let Some((child, cost, hull)) = self.successor(id, k) {
                self.push(child, cost, hull);
            }
        }
    }

    /// The successor of node `id` along primitive `k` with its cost and interval hull, if
    /// the edge moves, is proven safe and its end node is not dominated.
    fn successor(
        &mut self,
        id: usize,
        k: usize,
    ) -> Option<(Node<R::Control, N>, f64, IntervalBox)> {
        let parent = &self.nodes[id];
        let offset = &self.problem.primitives[k];
        let target = core::array::from_fn(|i| parent.nominal[i] + offset[i]);
        let (control, nominal) = self.problem.reach.steer(&parent.nominal, &target);
        let length = Euclidean::<N>.distance(&parent.nominal, &nominal);

        // A zero-length edge could repeat forever at no cost.
        if length.is_nan() || length <= 0.0 {
            return None;
        }

        let (verdict, set) = check_edge(
            &self.problem.reach,
            &self.problem.obstacles,
            &parent.set,
            &control,
            &mut self.cover,
        );

        if !verdict.is_safe() {
            return None;
        }

        let cost = self.costs[id] + length;
        let hull = set.interval_hull();

        if self.is_dominated(&nominal, &hull, cost) {
            return None;
        }

        let child = Node {
            nominal,
            set,
            parent: Some(id),
            control: Some(control),
        };

        Some((child, cost, hull))
    }

    /// Whether a node in the cell of `nominal` costs at most `cost` and has a hull inside
    /// `hull`.
    fn is_dominated(&self, nominal: &[f64; N], hull: &IntervalBox, cost: f64) -> bool {
        self.cells.get(&self.cell(nominal)).is_some_and(|ids| {
            ids.iter()
                .any(|&e| self.costs[e] <= cost && box_within(&self.hulls[e], hull))
        })
    }

    /// Adds `node` with path cost `cost` and interval hull `hull` to the tree, its cell and
    /// the open list, and supersedes the nodes in its cell that it dominates.
    fn push(&mut self, node: Node<R::Control, N>, cost: f64, hull: IntervalBox) {
        let id = self.nodes.len();
        let f = cost + self.heuristic(&node.nominal);
        let cell = self.cells.entry(self.cell(&node.nominal)).or_default();

        for &e in cell.iter() {
            if cost <= self.costs[e] && box_within(&hull, &self.hulls[e]) {
                self.superseded[e] = true;
            }
        }

        cell.push(id);
        self.hulls.push(hull);
        self.superseded.push(false);
        self.costs.push(cost);
        self.nodes.push(node);
        self.open.push(Entry { f, id });
    }

    /// The lower bound `max(0, |x - center| - goal_radius)` on the remaining path length.
    fn heuristic(&self, nominal: &[f64; N]) -> f64 {
        let distance = Euclidean::<N>.distance(nominal, &self.problem.goal.center());

        (distance - self.config.goal_radius).max(0.0)
    }

    /// The lattice cell of `nominal`.
    fn cell(&self, nominal: &[f64; N]) -> [i64; N] {
        #[expect(
            clippy::cast_possible_truncation,
            reason = "cells only group nodes for deduplication; a saturated cell is harmless"
        )]
        core::array::from_fn(|i| (nominal[i] / self.config.resolution).round() as i64)
    }

    /// Whether the set of node `id` is proven inside the goal.
    fn reaches_goal(&self, id: usize) -> bool {
        self.problem.goal.containment(&self.nodes[id].set) == Containment::Inside
    }
}

/// Whether box `inner` lies inside box `outer`, bound by bound.
fn box_within(inner: &IntervalBox, outer: &IntervalBox) -> bool {
    inner.dim() == outer.dim()
        && inner
            .0
            .iter()
            .zip(&outer.0)
            .all(|(i, o)| o.lo <= i.lo && i.hi <= o.hi)
}

#[cfg(test)]
mod tests {
    use polyzonotope::Interval;

    use super::*;

    #[test]
    fn open_list_pops_lowest_f_then_oldest() {
        let mut open = BinaryHeap::new();

        for (f, id) in [(2.0, 0), (1.0, 1), (1.0, 2), (3.0, 3)] {
            open.push(Entry { f, id });
        }

        let order: Vec<usize> = core::iter::from_fn(|| open.pop().map(|e| e.id)).collect();
        assert_eq!(order, [1, 2, 0, 3]);
    }

    #[test]
    fn box_within_compares_every_bound() {
        let b = |lo: f64, hi: f64| IntervalBox(vec![Interval::new(lo, hi); 2]);

        assert!(box_within(&b(1.0, 2.0), &b(0.0, 3.0)));
        assert!(box_within(&b(1.0, 2.0), &b(1.0, 2.0)));
        assert!(!box_within(&b(0.0, 3.0), &b(1.0, 2.0)));
        assert!(!box_within(&b(0.5, 3.5), &b(0.0, 3.0)));
    }
}
