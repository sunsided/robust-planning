//! Reachability-guided search: A* over reachable-set nodes on a lattice of nominal states.
//!
//! Each node carries a nominal state and the reachable set the system can be in there. To
//! expand a node, every motion primitive (an offset from the nominal state) is steered
//! toward, the node's set is propagated along the edge, and the edge is kept only if every
//! set covering it is proven disjoint from every obstacle. Nodes are expanded in order of
//! `f = g + h`: `g` is the path cost (see [`Config::cost`]), `h` a conservative lower bound
//! on the remaining length (see [`Config::goal_radius`]). Successors whose nominal state
//! leaves [`Problem::bounds`] are dropped. A node reaches the goal when its whole
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
//! nodes in its cell that it dominates, and they are skipped when popped.
//!
//! By default ([`Dedup::Hull`]) this is a heuristic dominance test: the hulls stand in for
//! the sets. Pruning never affects safety, since every kept edge is still proven, but it can
//! drop a node whose set is thinner than its hull and so cost completeness in tight
//! passages. [`Dedup::Proven`] replaces it with a proof that one set lies inside the other
//! ([`decide_within`]), for zonotopes in up to four dimensions, at the
//! price of keeping many more nodes.
//!
//! # Examples
//!
//! ```
//! use robust_planning::polyzonotope::{
//!     Interval, IntervalBox, Polytope, SparsePolyZonotope, Zonotope,
//! };
//! use robust_planning::sampling_planning::StraightLine;
//! use robust_planning::search::{Config, GoalRadius, Problem, ReachSearch};
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
//!     ) -> Option<SparsePolyZonotope> {
//!         let from = initial.interval_hull();
//!         let hull = (0..2).map(|i| {
//!             Interval::new(from.0[i].lo.min(end[i] - 0.1), from.0[i].hi.max(end[i] + 0.1))
//!         });
//!         let end_box = (0..2).map(|i| Interval::new(end[i] - 0.1, end[i] + 0.1));
//!         let set = |b: IntervalBox| SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&b));
//!
//!         cover.push(set(IntervalBox(hull.collect())));
//!         Some(set(IntervalBox(end_box.collect())))
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
//!     bounds: None,
//! };
//! let config = Config { goal_radius: GoalRadius::Fixed(0.0), ..Config::default() };
//! let mut search = ReachSearch::new(problem, config).expect("valid start and config");
//! let path = search.solve().expect("the detour is proven safe");
//!
//! assert!((path.cost - 8.0).abs() < 1e-9, "up 2, across 4, down 2");
//! ```

use core::{cmp::Ordering, fmt};
use std::collections::{BinaryHeap, HashMap};

use polyzonotope::{Containment, Interval, IntervalBox, Polytope, SparsePolyZonotope};
use sampling_planning::{Euclidean, Path, StateSpace};

use crate::{
    Cost, Goal, Node, Obstacles, PolytopeGoal, Reach, SafetyVerdict, Steer, check_edge,
    containment::decide_within, node,
};

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
    /// The workspace: lower and upper corners of the box that every nominal state must lie
    /// in, or `None` for no bound.
    ///
    /// Successors outside it are dropped before their edge is checked, which keeps the
    /// lattice finite in open worlds without framing walls. It bounds nominal states only;
    /// to keep whole sets inside, add the outside as obstacles.
    pub bounds: Option<([f64; N], [f64; N])>,
}

/// Where the heuristic's goal radius comes from; see [`Config::goal_radius`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum GoalRadius {
    /// An infinite radius: `h = 0`, a uniform-cost search. The default.
    #[default]
    Infinite,
    /// This radius. Non-negative and not NaN.
    Fixed(f64),
    /// [`Goal::radius`] of the problem's goal, for example the half diagonal of a
    /// [`PolytopeGoal`] box.
    ///
    /// Admissible when each goal node's nominal state lies in its set, so that a set inside
    /// the goal puts the nominal state inside it as well. That holds for models whose set
    /// contains the nominal trajectory, like the affine test model.
    FromGoal,
}

/// Settings for [`ReachSearch`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    /// Width of the lattice cells nominal states are binned into for deduplication.
    /// Positive and finite.
    pub resolution: f64,
    /// Distance from [`Goal::center`] within which every goal node's nominal state lies.
    ///
    /// The heuristic is `h(x) = max(0, |x - center| - goal_radius)`, which is consistent
    /// for Euclidean edge costs. It is admissible, and the returned path optimal over the
    /// nodes deduplication keeps, only if every node whose set is proven inside the goal
    /// has its nominal state within `goal_radius` of the center. An infinite radius gives
    /// `h = 0`, a uniform-cost search that is always admissible but expands more nodes.
    /// The heuristic never affects safety.
    pub goal_radius: GoalRadius,
    /// The most nodes [`ReachSearch::solve`] expands before giving up.
    pub max_expansions: usize,
    /// How an edge is priced; `g` sums it. [`Cost::Length`] by default. Every option is at
    /// least the nominal length, so the heuristic stays admissible.
    pub cost: Cost,
    /// When a new node supersedes an older one, also skip every descendant the older node
    /// has generated so far. Off by default.
    ///
    /// The superseded node's subtree was grown from a set that a cheaper node with a set
    /// at most as wide (by the hull test) now covers, so the new node regrows it. Pruning
    /// only removes nodes from the open list; every kept edge stays proven, so it never
    /// affects safety. Like deduplication it can cost completeness, since dominance is a
    /// hull test.
    ///
    /// With the shipped heuristic, which is consistent, `f` never decreases along an edge.
    /// A node that is already expanded is then superseded only by a node of exactly equal
    /// cost, so this rarely prunes anything; it saves work only on such ties.
    pub prune_superseded_subtrees: bool,
    /// How one node's set is shown to lie inside another's for deduplication.
    /// [`Dedup::Hull`] by default.
    pub dedup: Dedup,
    /// Require whole sets, not only nominal states, to stay in [`Problem::bounds`]. Off by
    /// default; without bounds it has no effect.
    ///
    /// When on, the start set and every cover set and end set of an edge must be proven
    /// inside the bounds box ([`Containment::Inside`] within the obstacles' split budget).
    /// An edge that is not proven inside is dropped like an unsafe one. The proof is
    /// rigorous with the `sound` feature.
    pub bounds_contain_sets: bool,
}

/// The set-in-set test behind deduplication; see [`Config::dedup`].
///
/// Deduplication never affects safety, only which nodes are kept, so both are safe to use.
/// They differ in what "node A covers node B" means.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Dedup {
    /// Interval hulls: A's hull inside B's hull. Cheap and works in every dimension, but a
    /// heuristic: A's set can stick out of B's set where B is thinner than its hull. The
    /// default.
    #[default]
    Hull,
    /// A proof that A's set lies inside B's set inflated by `tolerance` along every axis,
    /// via [`decide_within`]. Where that test does not handle B's shape (not a zonotope,
    /// more than [`MAX_DIMENSION`](crate::containment::MAX_DIMENSION) dimensions or
    /// [`MAX_GENERATORS`](crate::containment::MAX_GENERATORS) generators, or a rank it
    /// cannot prove full), it falls back to the hull test.
    ///
    /// With `tolerance` 0, sets that only touch are never merged, which keeps many more
    /// nodes than [`Dedup::Hull`]. A small positive tolerance (`1e-9` merges rounding
    /// noise) merges them, and the dominance then holds up to `tolerance`. The hull test,
    /// widened by `tolerance`, runs first as a cheap filter.
    Proven {
        /// How far, along every axis, A may stick out of B. Finite and non-negative.
        tolerance: f64,
    },
}

impl Dedup {
    /// The tolerance of [`Dedup::proven`]: merges sets that differ by rounding noise.
    pub const PROVEN_TOLERANCE: f64 = 1e-9;

    /// [`Dedup::Proven`] with the default tolerance [`Dedup::PROVEN_TOLERANCE`] (`1e-9`).
    ///
    /// Dominance then holds only up to `1e-9` along every axis: a merged node's set may
    /// stick out of the covering set by that much. Like all deduplication this affects
    /// completeness, never safety. Use `Dedup::Proven { tolerance }` for another value,
    /// `0.0` for exact containment. The overall default stays [`Dedup::Hull`].
    ///
    /// # Examples
    ///
    /// ```
    /// use robust_planning::search::Dedup;
    ///
    /// assert_eq!(Dedup::proven(), Dedup::Proven { tolerance: 1e-9 });
    /// assert_eq!(Dedup::default(), Dedup::Hull);
    /// ```
    #[must_use]
    pub const fn proven() -> Self {
        Self::Proven {
            tolerance: Self::PROVEN_TOLERANCE,
        }
    }
}

impl Default for Config {
    /// A resolution of `1e-6` (merging only rounding noise), an infinite goal radius
    /// (uniform-cost search), at most 10 000 expansions, length costs, no subtree pruning
    /// and hull deduplication.
    fn default() -> Self {
        Self {
            resolution: 1e-6,
            goal_radius: GoalRadius::Infinite,
            max_expansions: 10_000,
            cost: Cost::Length,
            prune_superseded_subtrees: false,
            dedup: Dedup::Hull,
            bounds_contain_sets: false,
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
    /// [`Config::goal_radius`] is a negative or NaN [`GoalRadius::Fixed`].
    GoalRadiusOutOfRange,
    /// [`Config::goal_radius`] is [`GoalRadius::FromGoal`] but the goal knows no radius.
    GoalRadiusUnavailable,
    /// [`Problem::bounds`] has a NaN or inverted bound, or does not contain the start.
    InvalidBounds,
    /// The tolerance of [`Dedup::Proven`] is negative or not finite.
    InvalidDedupTolerance,
    /// The weight of [`Config::cost`] is negative or not finite.
    InvalidCostWeight,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::StartNotSafe(verdict) => write!(f, "start set is not proven safe ({verdict:?})"),
            Self::ResolutionOutOfRange => f.write_str("resolution must be positive and finite"),
            Self::GoalRadiusOutOfRange => f.write_str("goal radius must be non-negative"),
            Self::GoalRadiusUnavailable => f.write_str("the goal provides no radius"),
            Self::InvalidDedupTolerance => {
                f.write_str("dedup tolerance must be finite and non-negative")
            }
            Self::InvalidBounds => {
                f.write_str("bounds must be ordered, not NaN, and contain the start")
            }
            Self::InvalidCostWeight => f.write_str("cost weight must be finite and non-negative"),
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
    /// Cost from the root by node id; see [`Config::cost`].
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
    /// The resolved [`Config::goal_radius`]; non-negative, may be infinite.
    goal_radius: f64,
    /// The bounds box that whole sets must stay in, if [`Config::bounds_contain_sets`] is
    /// on and there are bounds.
    workspace: Option<Polytope>,
    /// Child ids by node id; kept for [`Config::prune_superseded_subtrees`].
    children: Vec<Vec<usize>>,
    /// Descendants skipped by [`Config::prune_superseded_subtrees`] so far.
    pruned: usize,
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
    /// - [`Error::GoalRadiusOutOfRange`] if a fixed goal radius is negative or NaN.
    /// - [`Error::GoalRadiusUnavailable`] if the radius should come from a goal that has
    ///   none.
    /// - [`Error::InvalidCostWeight`] if the cost weight is negative or not finite.
    /// - [`Error::InvalidBounds`] if the bounds are NaN, inverted or exclude the start, or,
    ///   with [`Config::bounds_contain_sets`], do not provably contain the start set.
    /// - [`Error::InvalidDedupTolerance`] if the proven-dedup tolerance is negative or not
    ///   finite.
    /// - [`Error::StartNotSafe`] if the start set is not proven disjoint from every obstacle.
    pub fn new(problem: Problem<R, N, G>, config: Config) -> Result<Self, Error> {
        let goal_radius = validate(&problem, &config)?;
        let workspace = workspace(&problem, &config);

        if workspace
            .as_ref()
            .is_some_and(|box_| !is_inside(&problem.start_set, box_, &problem.obstacles))
        {
            return Err(Error::InvalidBounds);
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
            goal_radius,
            workspace,
            children: Vec::new(),
            pruned: 0,
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

    /// The cost from the root to `node`: the nominal path length with [`Cost::Length`].
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

    /// The number of descendants skipped by [`Config::prune_superseded_subtrees`] so far.
    #[must_use]
    pub const fn pruned(&self) -> usize {
        self.pruned
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
    /// The cost is [`ReachSearch::cost`] of the goal node. With [`Cost::Length`] that is the
    /// Euclidean length of the nominal path, bit for bit.
    #[must_use]
    pub fn path(&self) -> Option<Path<[f64; N]>> {
        let goal = self.goal_node?;
        let states = node::nominal_path(&self.nodes, goal).states;

        Some(Path {
            states,
            cost: self.costs[goal],
        })
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
        if length.is_nan() || length <= 0.0 || !self.in_bounds(&nominal) {
            return None;
        }

        let (verdict, end) = check_edge(
            &self.problem.reach,
            &self.problem.obstacles,
            &parent.set,
            &control,
            &mut self.cover,
        );
        let set = end.filter(|_| verdict.is_safe())?;

        if !self.sets_in_workspace(&set) {
            return None;
        }

        let cost = self.costs[id] + self.config.cost.edge(length, &set);
        let hull = set.interval_hull();

        if self.is_dominated(&nominal, &set, &hull, cost) {
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

    /// Whether a node in the cell of `nominal` costs at most `cost` and covers the set
    /// `set` with interval hull `hull`, by [`Config::dedup`].
    fn is_dominated(
        &self,
        nominal: &[f64; N],
        set: &SparsePolyZonotope,
        hull: &IntervalBox,
        cost: f64,
    ) -> bool {
        self.cells.get(&self.cell(nominal)).is_some_and(|ids| {
            // Skip superseded nodes: one superseded by dominance is covered by its superseder,
            // which is checked in its place, and a pruned descendant must not drop the node
            // that regrows it.
            ids.iter().any(|&e| {
                !self.superseded[e]
                    && self.costs[e] <= cost
                    && self.covers((&self.nodes[e].set, &self.hulls[e]), (set, hull))
            })
        })
    }

    /// The nodes in the cell of `nominal` that a node with cost `cost`, set `set` and hull
    /// `hull` dominates.
    fn dominated_by(
        &self,
        nominal: &[f64; N],
        set: &SparsePolyZonotope,
        hull: &IntervalBox,
        cost: f64,
    ) -> Vec<usize> {
        self.cells
            .get(&self.cell(nominal))
            .map(|ids| {
                ids.iter()
                    .copied()
                    .filter(|&e| {
                        cost <= self.costs[e]
                            && self.covers((set, hull), (&self.nodes[e].set, &self.hulls[e]))
                    })
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Whether the set `inner` with its hull lies inside `outer`, by [`Config::dedup`].
    fn covers(
        &self,
        inner: (&SparsePolyZonotope, &IntervalBox),
        outer: (&SparsePolyZonotope, &IntervalBox),
    ) -> bool {
        match self.config.dedup {
            Dedup::Hull => box_within(inner.1, outer.1, 0.0),
            Dedup::Proven { tolerance } => {
                let splits = self.problem.obstacles.max_splits();

                box_within(inner.1, outer.1, tolerance)
                    && decide_within(inner.0, outer.0, tolerance, splits)
                        .unwrap_or_else(|| box_within(inner.1, outer.1, 0.0))
            }
        }
    }

    /// Adds `node` with path cost `cost` and interval hull `hull` to the tree, its cell and
    /// the open list, and supersedes the nodes in its cell that it dominates.
    fn push(&mut self, node: Node<R::Control, N>, cost: f64, hull: IntervalBox) {
        let id = self.nodes.len();
        let f = cost + self.heuristic(&node.nominal);
        let dominated = self.dominated_by(&node.nominal, &node.set, &hull, cost);
        self.cells
            .entry(self.cell(&node.nominal))
            .or_default()
            .push(id);

        if let Some(parent) = node.parent {
            self.children[parent].push(id);
        }

        self.hulls.push(hull);
        self.superseded.push(false);
        self.costs.push(cost);
        self.children.push(Vec::new());
        self.nodes.push(node);
        self.open.push(Entry { f, id });

        for e in dominated {
            self.supersede(e, id);
        }
    }

    /// Marks node `e` superseded by node `by` and, with
    /// [`Config::prune_superseded_subtrees`], every descendant of `e` as well.
    ///
    /// Descendants that are ancestors of `by` are kept, so `by`'s own branch is never
    /// pruned. With positive edge costs that case cannot arise, since `by` costs at most
    /// as much as `e`; the guard keeps it sound regardless.
    fn supersede(&mut self, e: usize, by: usize) {
        self.superseded[e] = true;

        if !self.config.prune_superseded_subtrees {
            return;
        }

        let ancestors = node::branch(&self.nodes, by);

        if ancestors.contains(&e) {
            return;
        }

        let mut stack = self.children[e].clone();

        while let Some(id) = stack.pop() {
            if !self.superseded[id] {
                self.superseded[id] = true;
                self.pruned += 1;
            }

            stack.extend_from_slice(&self.children[id]);
        }
    }

    /// Whether `end` and the cover sets of the edge just checked are proven inside the
    /// workspace box, if whole sets must stay there.
    fn sets_in_workspace(&self, end: &SparsePolyZonotope) -> bool {
        self.workspace.as_ref().is_none_or(|box_| {
            let obstacles = &self.problem.obstacles;

            is_inside(end, box_, obstacles)
                && self.cover.iter().all(|set| is_inside(set, box_, obstacles))
        })
    }

    /// Whether `nominal` lies in [`Problem::bounds`], if there are any.
    fn in_bounds(&self, nominal: &[f64; N]) -> bool {
        self.problem.bounds.is_none_or(|(lower, upper)| {
            (0..N).all(|i| lower[i] <= nominal[i] && nominal[i] <= upper[i])
        })
    }

    /// The lower bound `max(0, |x - center| - goal_radius)` on the remaining path length.
    fn heuristic(&self, nominal: &[f64; N]) -> f64 {
        let distance = Euclidean::<N>.distance(nominal, &self.problem.goal.center());

        (distance - self.goal_radius).max(0.0)
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

/// The bounds of `problem` as a polytope, if [`Config::bounds_contain_sets`] is on.
fn workspace<R, const N: usize, G>(
    problem: &Problem<R, N, G>,
    config: &Config,
) -> Option<Polytope> {
    let (lower, upper) = problem.bounds.filter(|_| config.bounds_contain_sets)?;
    let box_ = IntervalBox((0..N).map(|i| Interval::new(lower[i], upper[i])).collect());

    Some(Polytope::from_box(&box_))
}

/// Whether `set` is proven inside `box_` within the split budget of `obstacles`.
fn is_inside(set: &SparsePolyZonotope, box_: &Polytope, obstacles: &Obstacles) -> bool {
    set.containment(box_, obstacles.max_splits()) == Containment::Inside
}

/// Checks `config` and the bounds of `problem`, and resolves the goal radius.
fn validate<R, const N: usize, G: Goal<N>>(
    problem: &Problem<R, N, G>,
    config: &Config,
) -> Result<f64, Error> {
    if !(config.resolution.is_finite() && config.resolution > 0.0) {
        return Err(Error::ResolutionOutOfRange);
    }

    if !config.cost.is_valid() {
        return Err(Error::InvalidCostWeight);
    }

    if let Dedup::Proven { tolerance } = config.dedup
        && !(tolerance.is_finite() && tolerance >= 0.0)
    {
        return Err(Error::InvalidDedupTolerance);
    }

    if let Some((lower, upper)) = problem.bounds {
        let contains_start =
            (0..N).all(|i| lower[i] <= problem.start[i] && problem.start[i] <= upper[i]);

        if !contains_start {
            return Err(Error::InvalidBounds);
        }
    }

    match config.goal_radius {
        GoalRadius::Infinite => Ok(f64::INFINITY),
        GoalRadius::Fixed(radius) if radius.is_nan() || radius < 0.0 => {
            Err(Error::GoalRadiusOutOfRange)
        }
        GoalRadius::Fixed(radius) => Ok(radius),
        GoalRadius::FromGoal => problem.goal.radius().ok_or(Error::GoalRadiusUnavailable),
    }
}

/// Whether box `inner` lies inside box `outer` widened by `tolerance`, bound by bound.
fn box_within(inner: &IntervalBox, outer: &IntervalBox, tolerance: f64) -> bool {
    inner.dim() == outer.dim()
        && inner
            .0
            .iter()
            .zip(&outer.0)
            .all(|(i, o)| o.lo - tolerance <= i.lo && i.hi <= o.hi + tolerance)
}

#[cfg(test)]
mod tests {
    use polyzonotope::{Interval, Zonotope};
    use sampling_planning::StraightLine;

    use super::*;
    use crate::WithSteering;

    /// Jumps to the setpoint; the end set is the box of half width 0.1 around it.
    struct Snap;

    impl Reach<2> for Snap {
        type Control = [f64; 2];

        fn propagate(
            &self,
            _: &SparsePolyZonotope,
            end: &[f64; 2],
            cover: &mut Vec<SparsePolyZonotope>,
        ) -> Option<SparsePolyZonotope> {
            cover.push(square(*end, 0.1));
            Some(square(*end, 0.1))
        }
    }

    /// The box `center ± half_width` as a set.
    fn square(center: [f64; 2], half_width: f64) -> SparsePolyZonotope {
        let b = IntervalBox(
            center
                .iter()
                .map(|c| Interval::new(c - half_width, c + half_width))
                .collect(),
        );

        SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&b))
    }

    /// A search with no obstacles and no primitives; nodes are pushed by hand.
    fn empty_search(prune: bool) -> ReachSearch<WithSteering<Snap, StraightLine>, 2> {
        let problem = Problem {
            reach: WithSteering::new(Snap, StraightLine::new(1.0)),
            obstacles: Obstacles::new(Vec::new(), 0),
            start: [0.0, 0.0],
            start_set: square([0.0, 0.0], 0.1),
            goal: PolytopeGoal::from_box(&IntervalBox(vec![Interval::new(8.0, 9.0); 2]), 0),
            primitives: Vec::new(),
            bounds: None,
        };
        let config = Config {
            prune_superseded_subtrees: prune,
            ..Config::default()
        };

        ReachSearch::new(problem, config).expect("valid")
    }

    /// Pushes a node at `nominal` under `parent` with `cost` and a set of `half_width`.
    fn push(
        search: &mut ReachSearch<WithSteering<Snap, StraightLine>, 2>,
        parent: usize,
        nominal: [f64; 2],
        cost: f64,
        half_width: f64,
    ) -> usize {
        let set = square(nominal, half_width);
        let hull = set.interval_hull();
        let node = Node {
            nominal,
            set,
            parent: Some(parent),
            control: Some(nominal),
        };
        search.push(node, cost, hull);

        search.nodes.len() - 1
    }

    #[test]
    fn superseding_prunes_the_old_subtree_only_when_asked() {
        for prune in [false, true] {
            let mut search = empty_search(prune);
            let a = push(&mut search, 0, [1.0, 0.0], 1.0, 0.3);
            let b = push(&mut search, a, [2.0, 0.0], 2.0, 0.3);
            let c = push(&mut search, b, [3.0, 0.0], 3.0, 0.3);
            let other = push(&mut search, 0, [0.0, 1.0], 1.0, 0.3);
            let d = push(&mut search, 0, [1.0, 0.0], 0.5, 0.1);

            assert!(search.superseded[a], "d is cheaper and narrower than a");
            assert_eq!(search.superseded[b], prune);
            assert_eq!(search.superseded[c], prune);
            assert!(!search.superseded[other] && !search.superseded[d]);
            assert_eq!(search.pruned(), if prune { 2 } else { 0 });
        }
    }

    #[test]
    fn proven_dedup_keeps_a_node_the_hull_test_wrongly_drops() {
        // A box of half width 0.6 is not inside the diamond |x| + |y| <= 1, although its
        // hull is inside the diamond's hull [-1, 1]².
        let diamond = SparsePolyZonotope::from_zonotope(&Zonotope::new(
            nalgebra::DVector::from_column_slice(&[1.0, 0.0]),
            nalgebra::DMatrix::from_column_slice(2, 2, &[0.5, 0.5, 0.5, -0.5]),
        ));
        let diamond_hull = diamond.interval_hull();

        for (dedup, dropped) in [
            (Dedup::Hull, true),
            (Dedup::Proven { tolerance: 0.0 }, false),
        ] {
            let mut search = empty_search(false);
            search.config.dedup = dedup;
            push(&mut search, 0, [1.0, 0.0], 1.0, 0.6);

            assert_eq!(
                search.is_dominated(&[1.0, 0.0], &diamond, &diamond_hull, 2.0),
                dropped,
                "{dedup:?}"
            );
        }

        let mut search = empty_search(false);
        search.config.dedup = Dedup::Proven { tolerance: 0.0 };
        push(&mut search, 0, [1.0, 0.0], 1.0, 0.3);
        assert!(
            search.is_dominated(&[1.0, 0.0], &diamond, &diamond_hull, 2.0),
            "a box of half width 0.3 is proven inside"
        );
    }

    #[test]
    fn pruning_never_touches_the_branch_of_the_superseding_node() {
        let mut search = empty_search(true);
        let a = push(&mut search, 0, [1.0, 0.0], 1.0, 0.3);
        let b = push(&mut search, a, [2.0, 0.0], 1.0, 0.3);
        // A zero-cost loop back into a's cell, which positive edge costs rule out; the guard
        // keeps a's subtree, which holds the new node's own branch.
        let d = push(&mut search, b, [1.0, 0.0], 1.0, 0.1);

        assert!(search.superseded[a]);
        assert!(!search.superseded[b] && !search.superseded[d]);
        assert_eq!(search.pruned(), 0);
    }

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
        let box_within_exact =
            |inner: &IntervalBox, outer: &IntervalBox| box_within(inner, outer, 0.0);

        assert!(box_within_exact(&b(1.0, 2.0), &b(0.0, 3.0)));
        assert!(box_within_exact(&b(1.0, 2.0), &b(1.0, 2.0)));
        assert!(!box_within_exact(&b(0.0, 3.0), &b(1.0, 2.0)));
        assert!(!box_within_exact(&b(0.5, 3.5), &b(0.0, 3.0)));
        assert!(box_within(&b(0.5, 3.5), &b(0.0, 3.0), 0.5));
    }
}
