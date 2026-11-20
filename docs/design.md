# robust-planning design

Planning over reachable sets under uncertainty: reachability-guided tree search, Robust-RRT,
and later safety-constrained trajectory planning.

## Core rule

Computation is conservative. A check that cannot prove safety returns
`SafetyVerdict::Indeterminate`, never `Safe`. Planners treat `Indeterminate` as a rejection by
default. A planner may only accept a state, edge or goal that has been proven.

## Problem model

States are points in `ℝᴺ`. The current API uses `[f64; N]`, which matches `sampling-planning`'s
`Euclidean<N>`. The system is uncertain in three ways:

| Source | Set | Where it enters |
| --- | --- | --- |
| State uncertainty | `X₀`, the start set | `robust_rrt::Problem::start_set`, or the tube box of `ReachChecker` |
| Control uncertainty | `U`: the nominal input `u` plus an actuation error `E` | in the model: `B (u ⊕ E)` |
| Disturbance | `W` | in the model: `⊕ W` per time step |

All sets are polyzonotope `SparsePolyZonotope`s. Zonotopes and boxes convert exactly.

- **Obstacles** are convex `Polytope`s, for example `Polytope::from_box`, kept in
  `Obstacles` together with a split budget. Non-convex obstacles are unions of polytopes.
- **The goal region** sits behind the `Goal<N>` trait, the set counterpart of
  `sampling-planning`'s `GoalRegion`: `center()` is drawn for goal-biased samples, and
  `containment(set)` answers polyzonotope's `Containment`. A node reaches the goal only
  when its whole reachable set is proven inside it (`Containment::Inside`); `NotInside`
  and `Unknown` count alike as not reached. A node whose set merely overlaps the goal does
  not count. `PolytopeGoal` is the shipped implementation: a convex polytope in
  H-representation plus a center and a split budget, and the default goal type of
  `robust_rrt::Problem`. Its constructor rejects facets with non-finite entries or the
  wrong dimension, because polyzonotope 0.2's `containment` never refutes a NaN facet and
  would answer `Inside` for a set far outside it.
- **Dynamics** sit behind the `Reach` trait (see below). `Reach::propagate` returns the
  sets that cover the whole edge under one control, plus the end set.
- **Steering** sits behind the separate `Steer<N, C>` trait. For one edge, `Steer::steer`
  chooses a control of type `C` and a nominal end state. Planners require
  `R: Reach<N> + Steer<N, R::Control>`. A model that steers itself implements both traits.
  A model that only propagates is paired with a steering function through
  `WithSteering::new(model, steering)`, which implements both by delegation. `Steer` is
  implemented for `sampling_planning::StraightLine` with the end point as the control, for
  models whose control is a setpoint. Steering is never trusted for safety: every control
  is propagated and checked, so a bad steering function costs progress, not soundness.

  Alternatives considered: a separate `steering` field on `Problem` and `ReachChecker`
  would mirror `sampling-planning`, but would force every caller to name a steering value
  even when the model steers itself. A blanket `impl Steer for R where R: Reach<Control =
  [f64; N]>` would clash (E0119) with every setpoint model that wants its own steering.

The model the tests and the example use is the affine system

```text
x_{k+1} = A x_k + B (u_k + e_k) + w_k,   e_k ∈ E, w_k ∈ W
X_{k+1} = reduce(A X_k ⊕ B (u_k ⊕ E) ⊕ W)
```

It uses `linear_map`, `minkowski_sum` and Girard `reduce` per step. Between time steps the
system is assumed to move in a straight line. The cover set of step `k` is then the box
around `X_k ∪ X_{k+1}`: it contains every segment from a point of `X_k` to a point of
`X_{k+1}`, so a step longer than a thin wall cannot jump over it. This model lives only in
`tests/it/affine.rs` and is shared with the example through `#[path]`. **The real
implementation will come from `polyzonotope-reach`** (owned by the polyzonotope session,
workspace member `crates/polyzonotope-reach`). That crate will provide validated
time-point and time-interval sets, including for nonlinear dynamics. The local `Reach`
trait is the seam that crate will plug into.

## Safety verdicts

`SafetyVerdict { Safe, Unsafe(Witness), Indeterminate }`:

- `Safe`: the reachable set provably avoids every unsafe region.
- `Unsafe`: some *actually reachable* state provably lies in an unsafe region.
- `Indeterminate`: neither has been proven, so refine or reject.

### From polyzonotope

| polyzonotope answer | Verdict | Why |
| --- | --- | --- |
| `Intersection::Disjoint` | `Safe` (for that obstacle) | Sound outer bounds separate the set from the polytope |
| `Intersection::Intersects` | `Indeterminate` | Its witness is a point of the *enclosure*, evaluated in plain `f64`. It is neither certified nor known to be reachable |
| `Intersection::Unknown` | `Indeterminate` | The split budget ran out |
| `Containment::Inside` (goal) | goal reached | Sound bounds put every point inside |
| `Containment::NotInside` / `Unknown` (goal) | not reached | |

`Intersects` would map to `Unsafe` only if it were proven. That needs two things: a
certified witness, and a set that is exact or under-approximated, so that the witness is a
real reachable state. polyzonotope 0.2 provides neither, so `from_intersection` never
produces `Unsafe`.

### Certified collisions: `Unsafe(Witness)`

`SafetyVerdict::Unsafe` carries a `Witness { point, set, obstacle }`: the colliding state,
the index of the cover set it lies in and the index of the obstacle. Without a witness
there is no `Unsafe`, only `Indeterminate`. The verdict is therefore `Clone` and
`PartialEq`, no longer `Copy`, `Eq` or `Hash`.

The witness comes from a trait hook,
`Reach::certify_collision(initial, control, set, obstacle)`, which
defaults to `None`. Only the model knows whether its cover sets are exact or
under-approximated, so the hook lives with the model. `check_edge` calls it only for edges
that are not proven safe, in time order, and keeps the first point that passes a plain-`f64`
sanity check (inside the obstacle and inside the set's interval hull). That check guards
against hook bugs; it is not a proof. `WithSteering` forwards the hook.

A cargo feature was rejected: a feature that changes the shape of `Unsafe` is not additive,
so enabling it in one crate of a build would break matches in another. Once polyzonotope
offers a certified witness, a model with exact sets can implement the hook by calling it.

Both `Unsafe` and `Indeterminate` are rejections, so a wrong witness can only cost a
diagnostic, never soundness: `Safe` is decided before the hook runs.

Verdicts over several sets and obstacles combine with `SafetyVerdict::and`: `Unsafe`
dominates `Indeterminate`, which dominates `Safe`. `Obstacles::check_set` and
`check_sets` stop at the first set that is not proven safe, since that already decides the
rejection.

### To sampling-planning

| Verdict | `sampling_planning::Validity` |
| --- | --- |
| `Safe` | `Valid` |
| `Unsafe` | `Invalid` |
| `Indeterminate` | `Unknown` |

`sampling-planning`'s planners accept only `Valid`, so `Unknown` is rejected like
`Invalid`. That matches the core rule without extra configuration.

## The `sound` feature

`sound` forwards to `polyzonotope/sound`, which rounds outward with inari. Without it,
polyzonotope computes in plain `f64`.

- Claims of rigor are made **only with `sound`**. Without it, `Safe` means "safe up to
  floating-point rounding". Docs, examples and reports must say which mode produced a
  result.
- All set arithmetic goes through polyzonotope operations, so that `sound` covers it. The
  test model builds `B (u ⊕ E)` as a `linear_map` of the zonotope with center `u` and
  generators `E`, not as an `f64` product `B·u`.
  It builds hull boxes from outward-rounded `interval_hull`s and only takes `min` and `max`
  of them, which is exact.
- Nominal states and controls are plain `f64`. They label nodes and choose inputs. Safety
  never depends on them: only the sets are checked.
- inari needs x86-64 with AVX2 and FMA (Haswell or newer) or AArch64.
  `.cargo/config.toml` sets `-C target-cpu=haswell` for x86-64, as polyzonotope does.
  Every gate runs both with and without `sound`.

## Edge checking for sampling-planning: `ReachChecker`

`sampling-planning` hands a `ValidityChecker` only `(from, to)`, two bare states with no
payload. The adapter therefore assumes a fixed uncertainty *tube*: at every node the real
state lies in the box `state ± half_widths`. A motion is `Valid` only if all of these hold:

1. `Steer::steer(from, to)` ends exactly at `to`. Otherwise the result is `Unknown`, since
   the next tube would be centered at the wrong state.
2. Every cover set, propagated from the box at `from`, is proven disjoint from every
   obstacle. That covers every intermediate time step and the motion between them.
3. The end set is proven inside the box at `to` (`Containment::Inside`). The tube
   assumption therefore holds again at the next node, and chained edges stay sound. For a
   contracting model this holds once the tube is at least the invariant error box.

`check_state` proves the box at a state disjoint from every obstacle. Uncertainty that
grows along a branch does not fit this interface. That case is what `RobustRrt` is for.

## Robust-RRT

`RobustRrt` is a minimal local tree. It reuses `Sampler`, `UniformBox`, `LinearScan`,
`Euclidean`, `unit_f64` and `Path` from `sampling-planning`. It cannot reuse
`trees::Rrt`, because nodes must carry data the RRT does not store (see Cross-crate needs).
Each node stores:

- a nominal state;
- its reachable set;
- its parent;
- the control from the parent.

One iteration:

1. Draw a goal-biased sample and find the nearest node by nominal state.
2. `steer` from the node's nominal state toward the sample, to get a control and a nominal
   end state.
3. `propagate` the node's set under that control, to get the cover sets and the end set.
4. Accept the edge only if `Obstacles::check_sets(cover)` is `Safe`. The cover spans the
   whole edge, every intermediate time step included. An empty cover is `Indeterminate`.
5. Add the node with the end set. It reaches the goal if `Goal::containment` proves the
   end set inside the goal.

Uncertainty builds up along a branch. Sets wider than a passage cannot get through it,
even when a point robot can. `solve` stops at the first goal node. All randomness comes
from the caller's RNG, so a seed fixes the run (the tree's nominals, parents, controls and
sets). `FactorId`s are process-global, so set *ids* differ between runs, but their bounds
do not.

## Refinement by splitting before rejecting

A set is only rejected after refinement has been tried. `Obstacles` passes its split budget
to `SparsePolyZonotope::intersection`. That call discards a piece when a single facet
separates it. Otherwise it splits the piece along its heaviest factor, up to the budget.
Sets that need a split are typical near obstacle corners: for example, a rotated square
near a box corner is `Unknown` at budget 0 and `Disjoint` at budget 1. Budget `0` disables
refinement. `ReachChecker` uses the same budget to prove end sets inside the tube, and
`PolytopeGoal::max_splits` bounds the goal proof.

Planned refinements:

- split the *edge* in time (shorter steps), not only the set;
- bisect the initial set and propagate the pieces separately;
- escalate the budget only for edges that come back `Indeterminate`.

## Cross-crate needs

Nothing here changes another repo. These are requests to the owning sessions.

**sampling-planning**

- **Per-node payload.** `ValidityChecker::check_motion` sees only `(from, to)`, and
  `Tree`/`Growth` are private. A tree planner that stores a payload per node (here the
  reachable set) and passes the parent's payload to the edge check would let `RobustRrt`
  reuse `trees::Rrt` and `RrtStar`, instead of a local tree.
- **Steering that returns controls.** `Steering::steer` returns only a state. Kinodynamic
  and robust planners need it to return the control (an input sequence) that produced the
  state, and need the edge check to receive that control. This crate's `Steer` trait is
  the local stand-in.
- **Node and edge enumeration** on `Rrt`/`RrtStar`, so that callers and tests can audit
  every accepted edge.
- **Non-ball goal regions.** `GoalRegion` is a metric ball around a center. Robust goals
  need a "set inside region" test, ideally a goal trait. This crate's `Goal` trait is the
  local stand-in.
- **Cost over sets** (later): RRT* rewiring changes a node's parent and therefore its
  reachable set. Rewiring must re-propagate and re-check the subtree.

**polyzonotope**

- **A certified witness** for `Intersection::Intersects`, so that an exact or
  under-approximated set can prove `Unsafe` through `Reach::certify_collision`: the factor
  values of a point of the set itself, plus an outward-rounded enclosure of its image that
  is proven inside every facet of the polytope.
- **Under-approximations**, or an exactness flag, for the same reason.
- **NaN facets in `containment`.** A facet with a NaN offset never refutes containment, so
  a set far outside it is reported `Inside`. `PolytopeGoal` guards against this locally.
- **Convex hull or time-interval enclosures** of two consecutive sets. Today the test model
  covers inter-sample motion with an axis-aligned box, which is coarse for rotated sets.

**polyzonotope-reach**

- The real `Reach` implementation: validated time-point *and* time-interval reachable sets
  for affine and nonlinear dynamics under `X₀`, `U` and `W`. robust-planning will switch
  from its local trait to that crate's API, or adapt it, once it exists.

## Testing strategy

- **Steering, goals and witnesses:** a steering that aims into a wall still yields only
  proven-disjoint edges; `ReachChecker` is `Unknown` when steering stops short; goals that
  answer `Unknown` or `NotInside` are never reached; `PolytopeGoal` rejects NaN facets and
  never contains a set one ULP outside; `Unsafe` appears only with a hook witness that
  passes the sanity check.
- **Unit tests:**
  - the verdict mapping from `Intersection`;
  - the `and` lattice and the `Validity` mapping;
  - splitting that turns `Unknown` into `Disjoint`.
- **Edge audit:** for every accepted edge in solved trees, re-propagate from the parent's
  set and assert that every cover set is `Disjoint` from every obstacle, calling
  polyzonotope directly rather than through `Obstacles`.
- **Monte-Carlo:** simulate the affine system along tree branches under random start
  states, input errors and disturbances, with a quarter of the draws at the corners of
  their boxes. Assert three things:
  - no state, and no point between two states, lies in an obstacle;
  - each state lies in the interval hull of its node's set;
  - the trees are non-trivial.
- **Unknown is rejected:** a stub model whose cover needs a split. At budget 0
  `ReachChecker` answers `Unknown` and `RobustRrt` adds no node. With splits, the same
  motion is `Valid`.
- **Reproducibility:** the same seed gives the same tree.
- **Narrow passage:** a wall with a 0.2 gap. A point RRT gets through. `RobustRrt`
  (settled set width 0.3) gets no node past the wall and finds no path. `sampling-planning`'s
  `Rrt` with `ReachChecker` (tube width 0.4) finds no path; `Rrt` exposes no nodes to check
  further. The same planners pass a 1.0 gap, so the rejection is due to the set width.
- All gates run with and without `sound`, so outward rounding must not break any test
  margin. Case counts stay modest.

Later: property tests over random affine systems; cross-checks of the sets against CORA
through polyzonotope's harness; fuzzing of the checker inputs.

## Milestones

1. **Done in this draft.** `SafetyVerdict` semantics, `Obstacles`, the `Reach` trait, the
   `ReachChecker` adapter for `sampling-planning`, and `RobustRrt` with a reachable set
   per node. Tests use an affine model; there is a 2D example.
2. Switch to `polyzonotope-reach`. Adopt time-interval sets and drop the box hull.
   Nonlinear dynamics.
3. Refinement beyond set splitting: time splitting, initial-set bisection, and budget
   escalation.
4. Robust RRT* or a reachability-guided best-first search, with re-propagation on
   rewiring. Reuse `sampling-planning` trees once they carry per-node payloads.
5. Safety-constrained trajectory planning: optimize nominal controls under proven set
   constraints. Fail-safe maneuvers.
6. Optional `temporal-control` adapter for timed plans (spaceflight track).

## References

These are cited from memory. Check venues and years before formal citation.

- R. Pepy, M. Kieffer, E. Walter. *Reliable robust path planning with application to
  mobile robots.* Int. J. Applied Mathematics and Computer Science 19(3), 2009. Box-RRT:
  RRT with interval-analysis edge checks.
- B. Luders, S. Karaman, E. Frazzoli, J. How. *Bounds on tracking error using closed-loop
  rapidly-exploring random trees.* American Control Conference, 2010. Robust CL-RRT,
  tracking-error tubes.
- B. Luders, M. Kothari, J. How. *Chance constrained RRT for probabilistic robustness to
  environmental uncertainty.* AIAA GNC, 2010.
- M. Althoff, O. Stursberg, M. Buss. *Reachability analysis of nonlinear systems with
  uncertain parameters using conservative linearization.* IEEE CDC, 2008.
- M. Althoff, J. M. Dolan. *Online verification of automated road vehicles using
  reachability analysis.* IEEE Transactions on Robotics 30(4), 2014.
- C. Pek, M. Althoff. *Fail-safe motion planning for online verification of autonomous
  vehicles using convex optimization.* IEEE Transactions on Robotics 37(3), 2021.
- N. Kochdumper, M. Althoff. *Sparse polynomial zonotopes: a novel set representation for
  reachability analysis.* IEEE Transactions on Automatic Control 66(9), 2021.
- A. Girard. *Reachability of uncertain linear systems using zonotopes.* HSCC, 2005.
- S. M. LaValle. *Rapidly-exploring random trees: a new tool for path planning.* TR 98-11,
  Iowa State University, 1998.
