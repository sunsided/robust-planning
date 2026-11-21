//! [`Reach`](crate::Reach) models on `polyzonotope-reach`: validated reachable sets of
//! continuous-time dynamics. Needs the `reach` feature.
//!
//! [`Linear`] propagates `x' = A x + B u`, [`Nonlinear`] any
//! [`Dynamics`](polyzonotope_reach::Dynamics) `x' = f(x, u)`. Both take one input set per
//! time step as the control of an edge: the system may follow any integrable input signal
//! with values in that set during the step, so nominal inputs, input errors and
//! disturbances (as extra input columns) all go into it.
//!
//! Each time step yields two sets. The time-interval set holds every state during the step
//! and becomes one cover set of the edge. The time-point set holds every state at the end
//! of the step and starts the next one; the last becomes the edge's end set.
//!
//! ```mermaid
//! graph LR
//!     Start["initial set"] --> Step["step under input set k"]
//!     Step -->|time interval| Cover["cover set k"]
//!     Step -->|time point| Next{"more input sets?"}
//!     Next -->|yes| Step
//!     Next -->|no| End["end set"]
//!     Step -->|refused| None["None: Indeterminate"]
//! ```
//!
//! Steering is separate: pair a model with a function that picks the input sets through
//! [`WithSteering`](crate::WithSteering).

mod linear;
mod nonlinear;

pub use linear::Linear;
pub use nonlinear::Nonlinear;

use polyzonotope::{SparsePolyZonotope, Zonotope};
use polyzonotope_reach::Step;

/// Chains one `step` per input set from `initial`, in order.
///
/// Pushes each step's time-interval set to `cover` and starts the next step from its
/// time-point set. Returns the last time-point set, or `initial` itself for no input sets
/// (with nothing pushed, which [`check_edge`](crate::check_edge) answers `Indeterminate`).
/// Returns `None` as soon as `step` refuses one, leaving the sets of the steps before it in
/// `cover`.
fn chain(
    initial: &SparsePolyZonotope,
    inputs: &[Zonotope],
    cover: &mut Vec<SparsePolyZonotope>,
    mut step: impl FnMut(&SparsePolyZonotope, &Zonotope) -> Option<Step>,
) -> Option<SparsePolyZonotope> {
    let mut set = initial.clone();

    for input in inputs {
        let Step {
            time_point,
            time_interval,
            ..
        } = step(&set, input)?;
        cover.push(time_interval);
        set = time_point;
    }

    Some(set)
}
