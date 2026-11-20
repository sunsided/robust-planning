//! Robust-RRT: an RRT whose nodes carry the reachable set they are reached with.
//!
//! Each iteration steers the nearest node toward a goal-biased sample, propagates that node's
//! reachable set along the edge and keeps the edge only if every set covering it is proven
//! disjoint from every obstacle. Uncertainty therefore builds up along a branch. A node
//! reaches the goal only when its whole set is proven inside the goal region.
//!
//! ```mermaid
//! graph LR
//!     Sample["sample (goal-biased)"] --> Nearest
//!     Nearest --> Steer["steer: control"]
//!     Steer --> Propagate["propagate set"]
//!     Propagate --> Check["check cover"]
//!     Check -->|safe| Add["add node"]
//!     Check -->|indeterminate| Sample
//!     Add --> Goal["set inside goal?"]
//! ```

use core::fmt;

use polyzonotope::{Containment, Polytope, SparsePolyZonotope};
use sampling_planning::{
    Euclidean, LinearScan, NearestNeighbors, Path, Sampler, StateSpace, rand_core::Rng, unit_f64,
};

use crate::{Obstacles, Reach, SafetyVerdict, Steer, reach::check_edge};

/// Everything [`RobustRrt`] needs to know about the world.
#[derive(Debug, Clone)]
pub struct Problem<R, Sa, const N: usize> {
    /// The dynamics model that steers and propagates.
    pub reach: R,
    /// The obstacles and their split budget.
    pub obstacles: Obstacles,
    /// Draws the states the tree grows toward.
    pub sampler: Sa,
    /// The nominal start state, which steering starts from.
    pub start: [f64; N],
    /// Every state the system may start in. Must be proven safe.
    pub start_set: SparsePolyZonotope,
    /// The goal region. A node reaches it when its set is proven inside.
    pub goal: Polytope,
    /// The state drawn for goal-biased samples, usually inside [`Problem::goal`].
    pub goal_center: [f64; N],
}

/// Settings for [`RobustRrt`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    /// Probability in `[0, 1]` of extending toward [`Problem::goal_center`] instead of a
    /// random sample.
    pub goal_bias: f64,
    /// Split budget for proving a node's set inside the goal region.
    pub goal_splits: usize,
}

impl Default for Config {
    /// A goal bias of 10 % and 8 goal splits.
    fn default() -> Self {
        Self {
            goal_bias: 0.1,
            goal_splits: 8,
        }
    }
}

/// A tree node.
#[derive(Debug, Clone)]
pub struct Node<C, const N: usize> {
    /// The nominal state the node's control drives toward.
    pub nominal: [f64; N],
    /// Every state the system can be in at this node.
    pub set: SparsePolyZonotope,
    /// The parent node id; `None` only for the root.
    pub parent: Option<usize>,
    /// The control from the parent to this node; `None` only for the root.
    pub control: Option<C>,
}

/// Why a [`RobustRrt`] could not be created.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The start set is not proven safe; carries the verdict.
    StartNotSafe(SafetyVerdict),
    /// [`Config::goal_bias`] is not in `[0, 1]`.
    GoalBiasOutOfRange,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::StartNotSafe(verdict) => write!(f, "start set is not proven safe ({verdict:?})"),
            Self::GoalBiasOutOfRange => f.write_str("goal bias must be in [0, 1]"),
        }
    }
}

impl core::error::Error for Error {}

/// The Robust-RRT planner.
///
/// Every edge in the tree is proven safe over its whole duration, every intermediate time
/// step included, from the reachable set of its parent. Runs are reproducible: all randomness
/// comes from the caller's RNG.
pub struct RobustRrt<R: Reach<N>, Sa, const N: usize> {
    /// The problem being solved.
    problem: Problem<R, Sa, N>,
    /// Settings.
    config: Config,
    /// Tree nodes by id; node `0` is the root.
    nodes: Vec<Node<R::Control, N>>,
    /// Index over the nominal states of all nodes, using node ids.
    nn: LinearScan<[f64; N]>,
    /// The first node whose set is proven inside the goal region.
    goal_node: Option<usize>,
    /// Scratch buffer for the cover sets of the edge being checked.
    cover: Vec<SparsePolyZonotope>,
}

impl<R: Reach<N> + Steer<N, R::Control>, Sa: Sampler<[f64; N]>, const N: usize>
    RobustRrt<R, Sa, N>
{
    /// Checks the start set and plants the root.
    ///
    /// # Errors
    ///
    /// - [`Error::StartNotSafe`] if the start set is not proven disjoint from every obstacle.
    /// - [`Error::GoalBiasOutOfRange`] if the goal bias is not in `[0, 1]`.
    pub fn new(problem: Problem<R, Sa, N>, config: Config) -> Result<Self, Error> {
        if !(0.0..=1.0).contains(&config.goal_bias) {
            return Err(Error::GoalBiasOutOfRange);
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

        let mut planner = Self {
            problem,
            config,
            nodes: Vec::new(),
            nn: LinearScan::default(),
            goal_node: None,
            cover: Vec::new(),
        };
        planner.add(root);

        Ok(planner)
    }

    /// Runs one iteration. Returns whether a node was added.
    ///
    /// # Panics
    ///
    /// Panics if the model's sets do not have the obstacles' dimension.
    pub fn step<G: Rng + ?Sized>(&mut self, rng: &mut G) -> bool {
        let target = self.draw(rng);
        let nearest = self
            .nn
            .nearest(&Euclidean::<N>, &target)
            .expect("the tree always holds the root");
        let parent = &self.nodes[nearest];
        let (control, nominal) = self.problem.reach.steer(&parent.nominal, &target);
        let (verdict, set) = check_edge(
            &self.problem.reach,
            &self.problem.obstacles,
            &parent.set,
            &control,
            &mut self.cover,
        );

        if !verdict.is_safe() {
            return false;
        }

        self.add(Node {
            nominal,
            set,
            parent: Some(nearest),
            control: Some(control),
        });

        true
    }

    /// Runs until the goal is reached or at most `max_iterations` iterations.
    ///
    /// Returns the path to the goal, or `None` if none was found within the budget.
    pub fn solve<G: Rng + ?Sized>(
        &mut self,
        max_iterations: usize,
        rng: &mut G,
    ) -> Option<Path<[f64; N]>> {
        for _ in 0..max_iterations {
            if self.goal_node.is_some() {
                break;
            }

            self.step(rng);
        }

        self.path()
    }

    /// All tree nodes by id; node `0` is the root.
    #[must_use]
    pub fn nodes(&self) -> &[Node<R::Control, N>] {
        &self.nodes
    }

    /// The first node whose set was proven inside the goal region, if any.
    #[must_use]
    pub const fn goal_node(&self) -> Option<usize> {
        self.goal_node
    }

    /// The problem being solved.
    #[must_use]
    pub const fn problem(&self) -> &Problem<R, Sa, N> {
        &self.problem
    }

    /// The node ids from the root to `node`, inclusive.
    ///
    /// # Panics
    ///
    /// Panics if `node` is not a node id.
    #[must_use]
    pub fn branch(&self, node: usize) -> Vec<usize> {
        let mut ids = vec![node];
        let mut cursor = node;

        while let Some(parent) = self.nodes[cursor].parent {
            ids.push(parent);
            cursor = parent;
        }

        ids.reverse();
        ids
    }

    /// The nominal path to the goal node, or `None` if the goal was not reached yet.
    ///
    /// The cost is the Euclidean length of the nominal path.
    #[must_use]
    pub fn path(&self) -> Option<Path<[f64; N]>> {
        let states: Vec<[f64; N]> = self
            .branch(self.goal_node?)
            .into_iter()
            .map(|id| self.nodes[id].nominal)
            .collect();
        let cost = states
            .windows(2)
            .map(|pair| Euclidean::<N>.distance(&pair[0], &pair[1]))
            .sum();

        Some(Path { states, cost })
    }

    /// Draws the goal center with the configured probability, otherwise a sampler state.
    fn draw<G: Rng + ?Sized>(&mut self, rng: &mut G) -> [f64; N] {
        if unit_f64(rng) < self.config.goal_bias {
            self.problem.goal_center
        } else {
            self.problem.sampler.sample(rng)
        }
    }

    /// Adds `node` to the tree and records it if it is the first to reach the goal.
    fn add(&mut self, node: Node<R::Control, N>) {
        let id = self.nodes.len();

        if self.goal_node.is_none() && self.reaches_goal(&node.set) {
            self.goal_node = Some(id);
        }

        NearestNeighbors::<Euclidean<N>>::insert(&mut self.nn, id, node.nominal);
        self.nodes.push(node);
    }

    /// Whether `set` is proven inside the goal region.
    fn reaches_goal(&self, set: &SparsePolyZonotope) -> bool {
        set.containment(&self.problem.goal, self.config.goal_splits) == Containment::Inside
    }
}
