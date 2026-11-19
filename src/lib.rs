//! # Robust planning
//!
//! Planning over reachable sets under uncertainty: Robust-RRT and reachability-guided search.
//!
//! ## Planned scope
//!
//! - Reachability-guided tree search
//! - Robust-RRT
//! - Safety-constrained trajectory planning
//! - Uses polyzonotope-reach for reachable sets
//!
//! ## Status
//!
//! Pre-release. Version 0.0.0 reserves the name; the API is not started yet and will change.
//!
//! Repository: <https://github.com/sunsided/robust-planning>

#![no_std]

/// The result of a conservative safety check.
///
/// Over-approximations can only prove safety. A check that cannot prove it
/// returns [`SafetyVerdict::Indeterminate`], never [`SafetyVerdict::Safe`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SafetyVerdict {
    /// The reachable set provably avoids every unsafe region.
    Safe,
    /// Some reachable state provably lies in an unsafe region.
    Unsafe,
    /// Neither could be proven; refine or reject.
    Indeterminate,
}
