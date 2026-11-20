//! Robust RRT*: an RRT* whose nodes carry the reachable set they are reached with.
//!
//! Each iteration extends the tree like [`RobustRrt`](crate::RobustRrt), then improves it
//! like RRT* (Karaman and Frazzoli 2011):
//!
//! - **Choose parent.** Among the nodes within the connection radius of the new node, the
//!   cheapest one whose edge to the new node is proven safe becomes its parent. Candidates
//!   are tried in order of cost, so only edges that would lower the cost are checked.
//! - **Rewire.** Each neighbor that would be cheaper through the new node is moved under it,
//!   but only as a transaction. A new parent gives the neighbor a new reachable set, so the
//!   whole subtree below it is re-propagated from there and every edge in it re-proven with
//!   [`check_edge`]. The rewire commits only if every edge stays proven
//!   [`SafetyVerdict::Safe`] and every node of the subtree that reached the goal still does.
//!   Otherwise the tree stays exactly as it was.
//!
//! A choose-parent or rewire edge must arrive *exactly* at the node's nominal state, because
//! the stored controls of its descendants start there. A model whose steering stops short of
//! the target therefore only rewires within its reach; keep [`Config::max_radius`] at or
//! below the model's steering range.
//!
//! ```mermaid
//! graph LR
//!     Extend["extend nearest toward sample"] -->|safe| Choose["choose cheapest safe parent"]
//!     Choose --> Insert["insert node"]
//!     Insert --> Cand{"neighbor cheaper via new node?"}
//!     Cand -->|yes, within budget| Prove["re-propagate and re-prove subtree"]
//!     Prove -->|all safe, goal kept| Commit["commit rewire"]
//!     Prove -->|otherwise| Keep["keep old tree"]
//! ```
//!
//! # Optimality
//!
//! The cost is the nominal path length, as in [`RobustRrt`](crate::RobustRrt). The cost of
//! [`RobustRrtStar::path`] never increases: new nodes only add goal candidates, a commit
//! only lowers the costs of a subtree, and no commit drops a node from the goal.
//!
//! No convergence to an optimum is claimed. RRT*'s asymptotic optimality argument assumes
//! every cheaper rewire is taken; here a rewire can be rejected because the new branch makes
//! a set wider, skipped because of [`Config::rewire_budget`], or impossible because the
//! steering does not reach the node exactly.
//!
//! The argument carries over when all of these hold (see `docs/design.md` for the
//! statement): the budget is unbounded ([`Config::unbounded`]); steering reaches every
//! target within [`Config::max_radius`] exactly; whether an edge is proven safe, and the
//! set it ends in, do not depend on the path to its parent (for example a model whose sets
//! are a fixed tube around the nominal); and `gamma` is above the RRT* bound. Then no rewire
//! is ever skipped or rejected for a reason other than its own edge, and the tree is RRT*
//! on the graph of proven-safe edges. The optimum it approaches is the cheapest path made of
//! such edges, with some clearance in that graph; it is not the nominal optimum.
//!
//! # Examples
//!
//! ```
//! use rand_pcg::Pcg64Mcg;
//! use robust_planning::polyzonotope::{
//!     Interval, IntervalBox, Polytope, SparsePolyZonotope, Zonotope,
//! };
//! use robust_planning::robust_rrt::Problem;
//! use robust_planning::robust_rrt_star::{Config, RobustRrtStar};
//! use robust_planning::sampling_planning::{StraightLine, UniformBox, rand_core::SeedableRng};
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
//! let problem = Problem {
//!     reach: WithSteering::new(Snap, StraightLine::new(1.0)),
//!     obstacles: Obstacles::new(vec![Polytope::from_box(&square(2.0, 2.0, 0.5))], 4),
//!     sampler: UniformBox::new([0.0, 0.0], [4.0, 4.0]),
//!     start: [0.5, 0.5],
//!     start_set: SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&square(0.5, 0.5, 0.1))),
//!     goal: PolytopeGoal::from_box(&square(3.5, 3.5, 0.3), 4),
//! };
//! let mut planner = RobustRrtStar::new(problem, Config::new(4.0, 1.0)).expect("valid");
//! let mut rng = Pcg64Mcg::seed_from_u64(1);
//!
//! planner.run(200, &mut rng);
//! let first = planner.path().expect("a path around the block").cost;
//! planner.run(800, &mut rng);
//! let later = planner.path().expect("still solved").cost;
//!
//! assert!(later <= first, "the cost never increases");
//! assert!(later < 4.6, "close to the shortest detour: {later}");
//! ```

#[cfg(test)]
mod tests;

use core::fmt;

use polyzonotope::{Containment, SparsePolyZonotope};
use sampling_planning::{
    Euclidean, LinearScan, NearestNeighbors, Path, Sampler, StateSpace, rand_core::Rng, unit_f64,
};

pub use crate::robust_rrt::Problem;
use crate::{Goal, Node, PolytopeGoal, Reach, SafetyVerdict, Steer, check_edge, node};

/// Settings for [`RobustRrtStar`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    /// Probability in `[0, 1]` of extending toward the goal's [`Goal::center`] instead of
    /// a random sample.
    pub goal_bias: f64,
    /// The connection radius constant: with `n` nodes in `N` dimensions, nodes within
    /// `gamma * (ln n / n)^(1/N)` of a new node are candidate parents and rewiring targets.
    ///
    /// RRT* needs `gamma` above
    /// [`RrtStarConfig::gamma_lower_bound`](sampling_planning::trees::RrtStarConfig::gamma_lower_bound)
    /// for asymptotic optimality. Positive and finite.
    pub gamma: f64,
    /// Upper bound on the connection radius. Positive; may be infinite.
    ///
    /// Keep it at or below the distance the model's steering reaches in one edge: a
    /// candidate the steering does not reach exactly is skipped without a check.
    pub max_radius: f64,
    /// The most edge checks spent on rewiring in one iteration.
    ///
    /// A rewire costs one check per node of the moved subtree. A rewire whose subtree does
    /// not fit in what is left of the budget is skipped, so one iteration never runs more
    /// than this many rewiring checks. `0` disables rewiring.
    pub rewire_budget: usize,
}

impl Config {
    /// A config with the given radius constant and radius cap, a goal bias of 10 % and a
    /// rewiring budget of 64 edge checks per iteration.
    #[must_use]
    pub const fn new(gamma: f64, max_radius: f64) -> Self {
        Self {
            goal_bias: 0.1,
            gamma,
            max_radius,
            rewire_budget: 64,
        }
    }

    /// A config like [`Config::new`] but with an unbounded rewiring budget
    /// ([`usize::MAX`] edge checks per iteration), so no cheaper rewire is ever skipped.
    ///
    /// This is the mode in which the RRT* optimality argument can carry over; see the
    /// [module docs](self#optimality) for the remaining conditions. An iteration can then
    /// re-prove the whole tree.
    #[must_use]
    pub const fn unbounded(gamma: f64, max_radius: f64) -> Self {
        Self {
            rewire_budget: usize::MAX,
            ..Self::new(gamma, max_radius)
        }
    }
}

/// Why a [`RobustRrtStar`] could not be created.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// The start set is not proven safe; carries the verdict.
    StartNotSafe(SafetyVerdict),
    /// [`Config::goal_bias`] is not in `[0, 1]`.
    GoalBiasOutOfRange,
    /// [`Config::gamma`] is not positive and finite.
    InvalidGamma,
    /// [`Config::max_radius`] is not positive.
    InvalidMaxRadius,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::StartNotSafe(verdict) => write!(f, "start set is not proven safe ({verdict:?})"),
            Self::GoalBiasOutOfRange => f.write_str("goal bias must be in [0, 1]"),
            Self::InvalidGamma => f.write_str("gamma must be positive and finite"),
            Self::InvalidMaxRadius => f.write_str("max radius must be positive"),
        }
    }
}

impl core::error::Error for Error {}

/// Counts of rewiring outcomes since the planner was created.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Stats {
    /// Rewires that were proven and committed.
    pub rewires: usize,
    /// Rewires that were tried and rejected: the edge to the new parent or an edge in the
    /// subtree was not proven safe, a goal node would have lost the goal, or the steering
    /// did not reach the node exactly.
    pub rejected_rewires: usize,
    /// Rewires skipped because their subtree did not fit in the remaining
    /// [`Config::rewire_budget`].
    pub skipped_rewires: usize,
}

/// Why a rewire was not committed. The tree is unchanged in every case.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Rejection {
    /// The subtree has more nodes than the remaining budget allows checks.
    OverBudget,
    /// The new parent lies in the subtree of the node, so the rewire would close a cycle.
    Cycle,
    /// Steering from the new parent does not end exactly at the node's nominal state.
    NotReached,
    /// The edge from the new parent to the node is not proven safe.
    EdgeUnproven,
    /// An edge below the node is not proven safe from its re-propagated parent set.
    DescendantUnproven,
    /// A node of the subtree that reached the goal would no longer be proven inside it.
    GoalLost,
}

/// A proven edge into a node that is not in the tree yet.
struct Edge<C> {
    /// The parent node id.
    parent: usize,
    /// The control from the parent.
    control: C,
    /// The end set, which becomes the new node's set.
    set: SparsePolyZonotope,
}

/// The Robust RRT* planner.
///
/// Every edge in the tree is proven safe over its whole duration, from the reachable set
/// its parent has *now*: rewires re-prove the subtrees they move. Runs are reproducible:
/// all randomness comes from the caller's RNG.
pub struct RobustRrtStar<R: Reach<N>, Sa, const N: usize, G = PolytopeGoal<N>> {
    /// The problem being solved.
    problem: Problem<R, Sa, N, G>,
    /// Settings; validated by [`RobustRrtStar::new`].
    config: Config,
    /// Tree nodes by id; node `0` is the root. Ids never change.
    nodes: Vec<Node<R::Control, N>>,
    /// Nominal path length from the root to each node, summed from the root down.
    costs: Vec<f64>,
    /// Child ids of each node, in no particular order.
    children: Vec<Vec<usize>>,
    /// Whether each node's set is proven inside the goal.
    in_goal: Vec<bool>,
    /// Index over the nominal states of all nodes, using node ids.
    nn: LinearScan<[f64; N]>,
    /// Rewiring outcomes so far.
    stats: Stats,
    /// Scratch buffer for the cover sets of the edge being checked.
    cover: Vec<SparsePolyZonotope>,
    /// Scratch buffer for neighbor queries.
    near: Vec<usize>,
}

impl<R, Sa, const N: usize, G> RobustRrtStar<R, Sa, N, G>
where
    R: Reach<N> + Steer<N, R::Control>,
    Sa: Sampler<[f64; N]>,
    G: Goal<N>,
{
    /// Checks the config and the start set and plants the root.
    ///
    /// # Errors
    ///
    /// - [`Error::GoalBiasOutOfRange`] if the goal bias is not in `[0, 1]`.
    /// - [`Error::InvalidGamma`] if `gamma` is not positive and finite.
    /// - [`Error::InvalidMaxRadius`] if `max_radius` is not positive.
    /// - [`Error::StartNotSafe`] if the start set is not proven disjoint from every obstacle.
    pub fn new(problem: Problem<R, Sa, N, G>, config: Config) -> Result<Self, Error> {
        validate(&config)?;
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
        let mut planner = Self {
            problem,
            config,
            nodes: Vec::new(),
            costs: Vec::new(),
            children: Vec::new(),
            in_goal: Vec::new(),
            nn: LinearScan::default(),
            stats: Stats::default(),
            cover: Vec::new(),
            near: Vec::new(),
        };
        planner.push(root, 0.0);

        Ok(planner)
    }

    /// Runs one iteration: extend, choose a parent, insert, rewire. Returns whether a node
    /// was added.
    ///
    /// # Panics
    ///
    /// Panics if the model's sets do not have the obstacles' dimension.
    pub fn step<Rg: Rng + ?Sized>(&mut self, rng: &mut Rg) -> bool {
        let target = self.draw(rng);
        let Some((fallback, nominal)) = self.extend(&target) else {
            return false;
        };

        let mut near = core::mem::take(&mut self.near);
        self.nn
            .within_radius(&Euclidean::<N>, &nominal, self.radius(), &mut near);
        let edge = self.choose_parent(fallback, &nominal, &near);
        let id = self.insert(edge, nominal);
        self.rewire_neighbors(id, &near);
        self.near = near;

        true
    }

    /// Runs `iterations` iterations.
    ///
    /// Unlike [`RobustRrt::solve`](crate::RobustRrt::solve), it does not stop at the first
    /// goal node: later iterations keep lowering the cost.
    ///
    /// # Panics
    ///
    /// Panics if the model's sets do not have the obstacles' dimension.
    pub fn run<Rg: Rng + ?Sized>(&mut self, iterations: usize, rng: &mut Rg) {
        for _ in 0..iterations {
            self.step(rng);
        }
    }

    /// All tree nodes by id; node `0` is the root.
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

    /// The cheapest node whose set is proven inside the goal region, if any. Ties go to the
    /// oldest node.
    #[must_use]
    pub fn goal_node(&self) -> Option<usize> {
        (0..self.nodes.len())
            .filter(|&id| self.in_goal[id])
            .min_by(|&a, &b| self.costs[a].total_cmp(&self.costs[b]))
    }

    /// Rewiring outcomes since the planner was created.
    #[must_use]
    pub const fn stats(&self) -> Stats {
        self.stats
    }

    /// The problem being solved.
    #[must_use]
    pub const fn problem(&self) -> &Problem<R, Sa, N, G> {
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

    /// The nominal path to [`RobustRrtStar::goal_node`], or `None` if the goal was not
    /// reached yet.
    ///
    /// The cost is the Euclidean length of the nominal path; it equals
    /// [`RobustRrtStar::cost`] of the goal node.
    #[must_use]
    pub fn path(&self) -> Option<Path<[f64; N]>> {
        Some(node::nominal_path(&self.nodes, self.goal_node()?))
    }

    /// Draws the goal center with the configured probability, otherwise a sampler state.
    fn draw<Rg: Rng + ?Sized>(&mut self, rng: &mut Rg) -> [f64; N] {
        if unit_f64(rng) < self.config.goal_bias {
            self.problem.goal.center()
        } else {
            self.problem.sampler.sample(rng)
        }
    }

    /// Steers the nearest node toward `target`. Returns the edge and its nominal end state if
    /// the edge moves and is proven safe.
    ///
    /// A zero-length edge is skipped, as in [`ReachSearch`](crate::ReachSearch): once a node
    /// sits at the goal center, every goal-biased draw would otherwise add a duplicate there.
    fn extend(&mut self, target: &[f64; N]) -> Option<(Edge<R::Control>, [f64; N])> {
        let parent = self
            .nn
            .nearest(&Euclidean::<N>, target)
            .expect("the tree always holds the root");
        let (control, nominal) = self
            .problem
            .reach
            .steer(&self.nodes[parent].nominal, target);
        let length = Euclidean::<N>.distance(&self.nodes[parent].nominal, &nominal);

        if length.is_nan() || length <= 0.0 {
            return None;
        }

        let set = prove(
            &self.problem,
            &mut self.cover,
            &self.nodes[parent].set,
            &control,
        )?;

        Some((
            Edge {
                parent,
                control,
                set,
            },
            nominal,
        ))
    }

    /// The connection radius for the next node: `min(gamma * (ln n / n)^(1/N), max_radius)`,
    /// with `n` counting the new node.
    fn radius(&self) -> f64 {
        let n = count_to_f64(self.nodes.len() + 1);
        let d = count_to_f64(N.max(1));

        (self.config.gamma * (n.ln() / n).powf(1.0 / d)).min(self.config.max_radius)
    }

    /// The cheapest parent for a node at `nominal` among `fallback.parent` and `near`.
    ///
    /// `fallback` is already proven. Other candidates are tried in order of cost, and only
    /// if they are cheaper; the first one with a proven edge wins.
    fn choose_parent(
        &mut self,
        fallback: Edge<R::Control>,
        nominal: &[f64; N],
        near: &[usize],
    ) -> Edge<R::Control> {
        let bound = self.cost_via(fallback.parent, nominal);
        let mut candidates: Vec<(f64, usize)> = near
            .iter()
            .map(|&id| (self.cost_via(id, nominal), id))
            .filter(|&(cost, id)| cost < bound && id != fallback.parent)
            .collect();
        candidates.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));

        candidates
            .into_iter()
            .find_map(|(_, id)| self.connect(id, nominal))
            .unwrap_or(fallback)
    }

    /// Tries to move every node in `near` under the new node `id` when that is cheaper,
    /// within the iteration's rewiring budget.
    fn rewire_neighbors(&mut self, id: usize, near: &[usize]) {
        let mut budget = self.config.rewire_budget;

        for &node in near {
            if self.cost_via(id, &self.nodes[node].nominal) >= self.costs[node] {
                continue;
            }

            match self.try_rewire(node, id, &mut budget) {
                Ok(()) => self.stats.rewires += 1,
                Err(Rejection::OverBudget) => self.stats.skipped_rewires += 1,
                Err(_) => self.stats.rejected_rewires += 1,
            }
        }
    }

    /// Moves `node` and its subtree under `parent` if every edge stays proven safe and no
    /// goal node loses the goal; otherwise leaves the tree unchanged.
    ///
    /// Charges one unit of `budget` per edge checked.
    fn try_rewire(
        &mut self,
        node: usize,
        parent: usize,
        budget: &mut usize,
    ) -> Result<(), Rejection> {
        let subtree = self.subtree(node, *budget).ok_or(Rejection::OverBudget)?;

        if subtree.iter().any(|&(id, _)| id == parent) {
            return Err(Rejection::Cycle);
        }

        let control = self
            .exact_control(parent, &self.nodes[node].nominal)
            .ok_or(Rejection::NotReached)?;
        let sets = self.prove_subtree(parent, &control, &subtree, budget)?;
        let in_goal = self.goal_flags(&subtree, &sets)?;
        self.commit(parent, control, &subtree, sets, &in_goal);

        Ok(())
    }

    /// The subtree of `node` in breadth-first order, each entry with the index of its
    /// parent's entry (`0` for the node itself), or `None` if it has more than `limit`
    /// nodes.
    fn subtree(&self, node: usize, limit: usize) -> Option<Vec<(usize, usize)>> {
        let mut entries = vec![(node, 0)];
        let mut next = 0;

        while next < entries.len() {
            let id = entries[next].0;
            entries.extend(self.children[id].iter().map(|&child| (child, next)));

            if entries.len() > limit {
                return None;
            }

            next += 1;
        }

        Some(entries)
    }

    /// The new set of every node in `subtree`, propagated from `parent` with `control` for
    /// the subtree root and the stored controls below it, if every edge is proven safe.
    fn prove_subtree(
        &mut self,
        parent: usize,
        control: &R::Control,
        subtree: &[(usize, usize)],
        budget: &mut usize,
    ) -> Result<Vec<SparsePolyZonotope>, Rejection> {
        let mut sets: Vec<SparsePolyZonotope> = Vec::with_capacity(subtree.len());
        let root_set = prove(
            &self.problem,
            &mut self.cover,
            &self.nodes[parent].set,
            control,
        );
        *budget = budget.saturating_sub(1);
        sets.push(root_set.ok_or(Rejection::EdgeUnproven)?);

        for &(id, parent_entry) in &subtree[1..] {
            let control = self.nodes[id]
                .control
                .as_ref()
                .expect("non-root nodes have one");
            let set = prove(&self.problem, &mut self.cover, &sets[parent_entry], control);
            *budget = budget.saturating_sub(1);
            sets.push(set.ok_or(Rejection::DescendantUnproven)?);
        }

        Ok(sets)
    }

    /// Whether each node of `subtree` is proven inside the goal with its new set, or
    /// [`Rejection::GoalLost`] if a node that reached the goal no longer does.
    fn goal_flags(
        &self,
        subtree: &[(usize, usize)],
        sets: &[SparsePolyZonotope],
    ) -> Result<Vec<bool>, Rejection> {
        subtree
            .iter()
            .zip(sets)
            .map(|(&(id, _), set)| {
                let inside = self.reaches_goal(set);

                if self.in_goal[id] && !inside {
                    Err(Rejection::GoalLost)
                } else {
                    Ok(inside)
                }
            })
            .collect()
    }

    /// Moves the root of `subtree` under `parent` and stores the proven sets, goal flags and
    /// costs of the whole subtree.
    fn commit(
        &mut self,
        parent: usize,
        control: R::Control,
        subtree: &[(usize, usize)],
        sets: Vec<SparsePolyZonotope>,
        in_goal: &[bool],
    ) {
        let node = subtree[0].0;
        let old = self.nodes[node].parent.expect("the root is never rewired");
        self.children[old].retain(|&child| child != node);
        self.children[parent].push(node);
        self.nodes[node].parent = Some(parent);
        self.nodes[node].control = Some(control);

        for ((&(id, _), set), &inside) in subtree.iter().zip(sets).zip(in_goal) {
            let parent = self.nodes[id].parent.expect("non-root nodes have one");
            self.costs[id] = self.cost_via(parent, &self.nodes[id].nominal);
            self.nodes[id].set = set;
            self.in_goal[id] = inside;
        }
    }

    /// A proven edge from node `from` that ends exactly at `nominal`, if there is one.
    fn connect(&mut self, from: usize, nominal: &[f64; N]) -> Option<Edge<R::Control>> {
        let control = self.exact_control(from, nominal)?;
        let set = prove(
            &self.problem,
            &mut self.cover,
            &self.nodes[from].set,
            &control,
        )?;

        Some(Edge {
            parent: from,
            control,
            set,
        })
    }

    /// The control that steers node `from` exactly to `nominal`, or `None` if steering
    /// ends anywhere else.
    fn exact_control(&self, from: usize, nominal: &[f64; N]) -> Option<R::Control> {
        let (control, end) = self.problem.reach.steer(&self.nodes[from].nominal, nominal);

        #[expect(
            clippy::float_cmp,
            reason = "the stored controls of the node's children start exactly at `nominal`"
        )]
        let reached = end == *nominal;

        reached.then_some(control)
    }

    /// Adds a node at `nominal` with the proven `edge` and returns its id.
    fn insert(&mut self, edge: Edge<R::Control>, nominal: [f64; N]) -> usize {
        let cost = self.cost_via(edge.parent, &nominal);
        let id = self.push(
            Node {
                nominal,
                set: edge.set,
                parent: Some(edge.parent),
                control: Some(edge.control),
            },
            cost,
        );
        self.children[edge.parent].push(id);

        id
    }

    /// Appends `node` with `cost` to every per-node table and the index; returns its id.
    fn push(&mut self, node: Node<R::Control, N>, cost: f64) -> usize {
        let id = self.nodes.len();
        self.in_goal.push(self.reaches_goal(&node.set));
        self.costs.push(cost);
        self.children.push(Vec::new());
        NearestNeighbors::<Euclidean<N>>::insert(&mut self.nn, id, node.nominal);
        self.nodes.push(node);

        id
    }

    /// The cost of reaching `nominal` through node `via`.
    fn cost_via(&self, via: usize, nominal: &[f64; N]) -> f64 {
        self.costs[via] + Euclidean::<N>.distance(&self.nodes[via].nominal, nominal)
    }

    /// Whether `set` is proven inside the goal region.
    fn reaches_goal(&self, set: &SparsePolyZonotope) -> bool {
        self.problem.goal.containment(set) == Containment::Inside
    }
}

/// The end set of the edge from `initial` under `control`, if the edge is proven safe.
///
/// Takes the problem and the cover buffer apart from the planner, so that `initial` can be
/// borrowed from its nodes.
fn prove<R: Reach<N>, Sa, const N: usize, G>(
    problem: &Problem<R, Sa, N, G>,
    cover: &mut Vec<SparsePolyZonotope>,
    initial: &SparsePolyZonotope,
    control: &R::Control,
) -> Option<SparsePolyZonotope> {
    let (verdict, end) = check_edge(&problem.reach, &problem.obstacles, initial, control, cover);

    verdict.is_safe().then_some(end)
}

/// Checks the ranges of every [`Config`] field.
fn validate(config: &Config) -> Result<(), Error> {
    if !(0.0..=1.0).contains(&config.goal_bias) {
        return Err(Error::GoalBiasOutOfRange);
    }

    if !(config.gamma.is_finite() && config.gamma > 0.0) {
        return Err(Error::InvalidGamma);
    }

    if config.max_radius.is_nan() || config.max_radius <= 0.0 {
        return Err(Error::InvalidMaxRadius);
    }

    Ok(())
}

/// Converts a count to `f64`, saturating at `u32::MAX` (far beyond any tree size).
fn count_to_f64(count: usize) -> f64 {
    f64::from(u32::try_from(count).unwrap_or(u32::MAX))
}
