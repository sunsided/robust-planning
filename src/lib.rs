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
//! - [`Obstacles`]: polytope obstacles, checked against reachable sets with optional
//!   refinement by splitting.
//! - [`ReachChecker`]: a [`sampling_planning::ValidityChecker`] that runs a [`Reach`] model
//!   along each motion, so `sampling-planning`'s planners only accept proven-safe edges.
//! - [`RobustRrt`]: an RRT whose nodes carry the reachable set they are reached with.
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
mod obstacles;
mod reach;
pub mod robust_rrt;
mod verdict;

pub use checker::ReachChecker;
pub use obstacles::Obstacles;
pub use reach::Reach;
pub use robust_rrt::RobustRrt;
pub use verdict::SafetyVerdict;

pub use polyzonotope;
pub use sampling_planning;
