//! # Robust planning
//!
//! Planning over reachable sets under uncertainty: Robust-RRT and reachability-guided search.
//!
//! Every check in this crate is conservative. A check that cannot prove safety answers
//! [`SafetyVerdict::Indeterminate`], never [`SafetyVerdict::Safe`], and planners reject
//! anything that is not proven safe.
//!
//! ## Building blocks
//!
//! - [`Reach`]: propagates a reachable set along one edge under one control. The crate defines
//!   the trait; implementations come from the user (and later from `polyzonotope-reach`).
//!   Its optional [`Reach::certify_collision`] hook lets a model whose sets are exact turn an
//!   unproven edge into [`SafetyVerdict::Unsafe`] with a [`Witness`]; [`check_edge`] runs
//!   one edge check.
//! - [`Steer`]: picks the control of one edge. Implement it on the model, or pair a model
//!   with a steering function through [`WithSteering`]; `sampling-planning`'s
//!   [`StraightLine`](sampling_planning::StraightLine) steers models whose control is a
//!   setpoint.
//! - [`Cost`]: how planners price an edge: the nominal length, or the length plus a weight
//!   times the width of the edge's end set (a robustness-weighted cost).
//! - [`Obstacles`]: polytope obstacles, checked against reachable sets with optional
//!   refinement by splitting.
//! - [`Goal`]: a goal region tested against whole reachable sets. [`PolytopeGoal`] is a
//!   convex polytope in H-representation; it answers
//!   [`Containment::Inside`](polyzonotope::Containment::Inside) only when the set is proven
//!   inside.
//! - [`proven_within`]: proves one planar set inside a planar zonotope, through
//!   polyzonotope's set-in-polytope test on an inner H-representation.
//! - [`ReachChecker`]: a [`sampling_planning::ValidityChecker`] that runs a [`Reach`] model
//!   along each motion, so `sampling-planning`'s planners only accept proven-safe edges.
//! - [`RobustRrt`]: an RRT whose nodes carry the reachable set they are reached with.
//! - [`RobustRrtStar`]: the RRT* variant of [`RobustRrt`]. Choose-parent and rewiring accept
//!   only proven-safe edges, and a rewire re-propagates and re-proves the whole moved
//!   subtree before it commits.
//! - [`ReachSearch`]: A* over reachable-set nodes along a fixed set of motion primitives,
//!   expanding only proven-safe edges, with deduplication on a lattice of nominal states.
//!
//! ## The `sound` feature
//!
//! Without `sound`, polyzonotope computes in plain `f64`, so [`SafetyVerdict::Safe`] means
//! "safe up to floating-point rounding". Only with `sound` (outward rounding) is it a proof.
//! See `docs/design.md`.
//!
//! ## Status
//!
//! Pre-release. Milestone 1 (edge checks and Robust-RRT in Euclidean spaces) works; the API
//! will change.
//!
//! Repository: <https://github.com/sunsided/robust-planning>

#![forbid(unsafe_code)]

mod checker;
mod containment;
mod cost;
mod goal;
mod node;
mod obstacles;
mod reach;
pub mod robust_rrt;
pub mod robust_rrt_star;
pub mod search;
mod steer;
mod verdict;

pub use checker::ReachChecker;
pub use containment::proven_within;
pub use cost::Cost;
pub use goal::{Goal, PolytopeGoal};
pub use node::Node;
pub use obstacles::Obstacles;
pub use reach::{Reach, check_edge};
pub use robust_rrt::RobustRrt;
pub use robust_rrt_star::RobustRrtStar;
pub use search::ReachSearch;
pub use steer::{Steer, WithSteering};
pub use verdict::{SafetyVerdict, Witness};

pub use polyzonotope;
pub use sampling_planning;
