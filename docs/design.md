# robust-planning design

Planning over reachable sets under uncertainty: reachability-guided search, Robust-RRT and
Robust RRT*.

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
`tests/it/affine.rs` and is shared with the example through `#[path]`. Users implement
`Reach` for their own dynamics. `polyzonotope-reach` (a member of the polyzonotope workspace)
computes validated time-point and time-interval sets, including for nonlinear dynamics; the
`Reach` trait is the seam a model built on it plugs into.

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

## Reachability-guided search: `ReachSearch`

`search::ReachSearch` is A* over reachable-set nodes. Instead of sampling, it expands every
node along a fixed list of motion primitives (`search::Problem::primitives`), offsets from
the node's nominal state that `Steer` aims at. Each edge is propagated from the parent's
set and kept only if `check_edge` proves it `Safe`, exactly as in `RobustRrt`. A node
reaches the goal when it is *popped* with its set proven inside the goal, which is the
A* optimality condition.

- **Cost** `g` sums `Config::cost` over the edges: the nominal length by default, or the
  robustness-weighted cost (below). Zero-length edges are skipped, since they could repeat
  at no cost.
- **Heuristic** `h(x) = max(0, |x - goal.center()| - goal_radius)`. It is consistent for
  Euclidean edge costs. It is admissible if every node whose set is proven inside the goal
  has its nominal state within `goal_radius` of the center. `search::GoalRadius` picks the
  radius: `Infinite` (the default, `h = 0`, uniform cost), `Fixed(r)`, or `FromGoal`,
  which takes `Goal::radius()`. `PolytopeGoal` answers it with the distance from its
  center to the farthest corner of the box its axis-aligned facets bound (`None` if an
  axis is unbounded that way), widened by a few ULPs. `FromGoal` is admissible when each
  goal node's nominal state lies in its set, as for the affine model. The heuristic orders expansion
  only, so it never affects safety.
- **Deduplication.** Nominal states are binned into cells of width `resolution` (default
  `1e-6`, which merges rounding noise only). A new node is dropped if a node in its cell
  costs at most as much and has an interval hull inside the new node's hull. A new node
  that dominates older nodes in its cell supersedes them, and they are skipped when popped.
  `Config::dedup` picks the set-in-set test. `Dedup::Hull` (the default) compares
  interval hulls, a heuristic: pruning never affects safety, but it can cost completeness
  in passages where a set is much thinner than its hull. `Dedup::Proven { tolerance }`
  proves `A ⊆ B ⊕ [-tolerance, tolerance]ⁿ` instead (next section), and falls back to the
  hull test where the proof does not handle `B`'s shape. A* expansions on the slalom
  (release, `sound`), all with the same optimum 22.49:

  | Test | Expansions |
  | --- | --- |
  | `Hull` | 350 |
  | `Proven { tolerance: 0 }` | 3388 |
  | `Proven { tolerance: 1e-12 }` (also `1e-9`, `1e-6`) | 234 |

  Without tolerance, sets that differ only by rounding noise touch and are never merged.
  Any tolerance from `1e-12` up merges them, and then the proof merges *more* than the
  hull test, whose exact bounds reject a hull that sticks out by one ULP. The dominance is
  then only true up to the tolerance, which, like all deduplication, affects completeness
  and never safety. `Dedup::proven()` picks `1e-9` (`Dedup::PROVEN_TOLERANCE`), the value
  chosen for the proven mode; an explicit `Dedup::Proven { tolerance }` still takes any
  other. The overall default stays `Dedup::Hull`, the cheap test that works in every
  dimension.
- **Workspace bounds.** `search::Problem::bounds` is an optional box for nominal states.
  Successors outside it are dropped before their edge is checked, so an open world needs
  no framing walls to stay finite. With `Config::bounds_contain_sets` (opt-in) whole sets
  must stay inside too: the start set and every cover and end set of an edge must be
  proven inside the box (`Containment::Inside` within the obstacles' split budget), and an
  edge that is not is dropped like an unsafe one. `new` rejects NaN or inverted bounds,
  bounds that exclude the start, and, with set bounds, a start set not proven inside.
- **Subtree pruning (opt-in).** With `Config::prune_superseded_subtrees`, a node that a
  new node supersedes also takes every descendant it has generated off the open list.
  This is sound: pruning only removes open nodes, every kept edge stays proven, and the
  goal branch is rebuilt from parents that are never removed. A guard keeps the
  superseding node's own branch, although positive edge costs already rule that case
  out. It is rarely effective: the heuristic is consistent, so `f` never decreases along
  an edge, and an expanded node can only be superseded by one of *equal* cost. On the
  slalom it prunes nothing and gives the same path. Unit tests on hand-pushed nodes cover
  the pruning itself.
- **Budget.** `max_expansions` bounds one `solve` call. Calling it again continues the
  search.
- Ties in `f` pop the oldest node first, so runs are deterministic.

The search has no external heuristic-search dependency. It uses `std`'s `BinaryHeap` and
`HashMap`.

## Robust RRT*: `RobustRrtStar`

`robust_rrt_star::RobustRrtStar` is the RRT* variant of `RobustRrt`. It takes the same
`robust_rrt::Problem` and keeps its own local tree: per node the `Node` (nominal state,
reachable set, parent, control), plus its cost, its children and whether its set is proven
inside the goal. `sampling-planning` is unchanged; the radius formula is repeated locally,
because that crate's `radius` module is private.

One iteration:

1. Extend the nearest node toward a goal-biased sample and prove the edge, exactly as
   `RobustRrt` does. If it is not proven safe, the iteration ends.
2. **Choose parent.** The neighbors within `min(gamma (ln n / n)^(1/N), max_radius)` of the
   new nominal state that would make it cheaper are tried in order of cost. The first one
   whose edge is proven `Safe` becomes the parent; otherwise the nearest node stays.
3. Insert the node.
4. **Rewire.** For every neighbor that would be cheaper through the new node, try a
   transaction (below), within `Config::rewire_budget` edge checks per iteration.

### The rewire transaction

A new parent gives the moved node a new reachable set, and every descendant's set depends
on it. The transaction therefore:

1. collects the node's subtree in breadth-first order, so every parent comes before its
   children. If the subtree has more nodes than the remaining budget, the rewire is
   *skipped* before any check, so one iteration never exceeds the budget. Budget `0`
   disables rewiring;
2. rejects the rewire if the new parent lies in the subtree (a cycle);
3. steers from the new parent to the node and rejects unless the steering ends *exactly* at
   the node's nominal state. The descendants keep their stored controls, which start
   there. A model that stops short of the target therefore rewires only within its reach,
   so `max_radius` should not exceed the model's steering range;
4. propagates the new parent's set along that edge, then every descendant from its
   parent's *new* set under its stored control, proving each edge with `check_edge`. The
   first edge that is not `Safe` rejects the rewire;
5. rejects the rewire if a node of the subtree that was proven inside the goal is not
   proven inside with its new set. Nodes may gain the goal, never lose it;
6. only then commits: parent, control, sets, goal flags and costs of the whole subtree.

The new sets live in a scratch vector until the commit, so a rejection leaves the tree
exactly as it was. Costs are recomputed parent-first as `cost(parent) + |edge|`, the same
left fold as the path's cost, so `cost(goal node)` equals `path().cost` bit for bit.

Rejecting is the only option considered. Pruning the descendants that become unproven
would also be sound, but it can drop goal nodes and so break the non-increasing cost, and
it throws away tree that took checks to build.

### Cost and optimality

The cost is `Config::cost`, the nominal path length by default. The best cost never
increases: new nodes only add goal candidates, a commit only lowers the costs of one
subtree, and no commit drops a goal node.
No convergence to an optimum is claimed. RRT*'s asymptotic optimality argument assumes
every cheaper rewire is taken; rejected rewires (a branch with a wider set), budget skips
and the exact-arrival rule all break it.

**When the argument carries over.** Let `E` be the set of nominal edges `(x, y)` with
`|x - y| <= max_radius`, and let an edge be *proven* when `check_edge` returns `Safe` for it.
RRT*'s proof (Karaman and Frazzoli 2011, Theorem 38) uses the obstacles only through the
predicate "edge `(x, y)` is collision-free", which must depend on `x` and `y` alone, and
uses every cheaper rewire. Both hold, and Robust RRT* is RRT* with "collision-free"
replaced by "proven", if:

1. **Unbounded budget.** `Config::unbounded` (`rewire_budget = usize::MAX`), so no rewire
   is skipped.
2. **Exact steering.** `steer(x, y)` ends exactly at `y` for every `|x - y| <= max_radius`,
   so no rewire is rejected as `NotReached` and choose-parent sees every neighbor.
3. **Path-independent proofs.** Whether `(x, y)` is proven depends on `x` and `y` only,
   and the end set at `y` depends on `y` alone, not on the parent or the branch that
   leads to it. This holds for a model whose set at a node is a fixed function of its
   nominal state (a tracking tube, as `ReachChecker` assumes), and fails for sets that
   grow along a branch. With it, a moved node and every re-propagated descendant keep
   their sets, so no rewire is rejected as `DescendantUnproven` or `GoalLost`.
4. **The RRT* side conditions:** `gamma` above `RrtStarConfig::gamma_lower_bound` for the
   sampled volume, a sampler with a density bounded below on the free space, and the
   cost `Cost::Length`. `Cost::WidthWeighted` charges per edge, so the same path cut into
   more edges costs more and the cost is not bounded by a multiple of the length; the
   RRT* argument approximates the optimum with ever shorter edges, so it does not carry
   over to that cost.
5. **A robust optimum in the proven graph.** There is an optimal path whose edges stay
   proven under small perturbations of their end points (clearance in the proven sense,
   not only in the nominal one), and whose end node's set is proven inside the goal under
   the same perturbations.

Under 1 to 5 the best cost converges almost surely to the cost of that robust optimum.
It is the optimum over proven edges, which is never below the nominal optimum and is
strictly above it wherever the sets are wider than the clearance. If 3 fails, the tree
still approaches the optimum of the rewires that commit, but no limit is claimed; the
`Grow` unit-test model shows a cheaper branch that cannot be taken because its set is
wider. The open-world test runs the unbounded mode, sees no skipped or rejected rewires
and ends within 5 % of the straight-line cost.

`RobustRrtStar::run` does not stop at the first goal node, since later iterations keep
lowering the cost. `stats()` counts committed, rejected and skipped rewires.

## Robustness-weighted cost: `Cost`

`Cost` prices one edge, for `RobustRrtStar` (`robust_rrt_star::Config::cost`) and
`ReachSearch` (`search::Config::cost`):

- `Cost::Length`, the default: the nominal edge length. Costs summed from the root equal
  the nominal path length bit for bit, so earlier results are unchanged.
- `Cost::WidthWeighted { weight }`: the length plus `weight` times the width of the edge's
  end set, the largest side of its interval hull. A path whose sets stay narrow keeps
  more margin to the obstacles that bounded the proof, and costs less. `weight` is finite
  and non-negative; the constructors reject anything else.

Both costs are additive and never below the nominal length, so the A* heuristic stays
admissible and consistent, and RRT*'s length bound `cost(parent) + |edge|` stays a lower
bound. With the weighted cost an edge's price depends on its parent's set, so:

- choose-parent tries candidates by that lower bound and proves each until the bound
  reaches the best proven cost;
- a rewire recomputes the costs of the whole moved subtree from the new sets, and is
  rejected as `NotCheaper` if the node itself does not get cheaper, or as `CostIncreased`
  if any node below it gets more expensive. Costs therefore still only go down, and the
  best cost still never increases.

The width is a hull measure, not a proof of anything; it only orders paths. Using the
mean width or the volume instead would be equally additive. The largest side was chosen
because it is dimension-independent in scale and cheap.

## Proven set-in-set containment: `decide_within`

polyzonotope 0.2 proves a set inside a polytope (`containment`) but not inside another
set. Proving `A ⊆ B` through an *outer* polytope of `B` would be unsound: `A` inside a
superset of `B` says nothing about `B`. What is needed is a polytope *inside* `B`.

A zonotope `B = c + Σ βⱼ gⱼ` of rank `n` in `n` dimensions is exactly the intersection of
the facet pairs `±ĉᵀ (x - c) <= Σⱼ |ĉᵀ gⱼ|`, where `ĉ` runs over the generalised cross
products of every `n - 1` generators (entry `i` is `(-1)^i` times the minor without row
`i`). Every such pair contains `B`, and the true facets are among them.

- **Plane.** `ĉ = (-g_y, g_x)` is exact (a swap and a negation). The support pairs along
  each generator are added too, which bounds a rank-1 zonotope (a segment).
- **3 and 4 dimensions.** The minors are expanded by Leibniz with the error bound
  `γ_{k + k! - 1} Σ |products|` (Higham §3.1), so each computed normal `ĉ` is within a
  componentwise `ε` of the exact one. Rank `n` must be proven first (some `n` generators
  with `|det| >` its error bound): below full rank the pairs only cut a slab around the
  flat zonotope, and the shape answers "not decided".
- **Offsets.** For a point `x` of `A` with `|xᵢ - cᵢ| <= rᵢ` (from `A`'s interval hull),
  the computed facet implies the exact one if `ĉᵀ x <= ĉᵀ c + Σⱼ |ĉᵀ gⱼ| - Σᵢ εᵢ (Σⱼ
  |gⱼᵢ| + rᵢ)`. That right-hand side is evaluated in `f64` and lowered by
  `2 γ_{n+m+2}` times the magnitudes of its terms, the slack rounded up, and an absolute
  term for underflow. So the polytope `P` depends on `A`, and `A ⊆ P` implies `A ⊆ B`.
- **Proof.** First a cheap rigorous test: `A`'s interval hull inside `P`, each facet's
  maximum over the box rounded up. Otherwise `A.containment(P) == Inside` from
  polyzonotope, which is rigorous with `sound`.
- **Inflation.** With a tolerance `δ > 0`, the `n` generators `δ eᵢ` are appended to `B`,
  which proves `A ⊆ B ⊕ [-δ, δ]ⁿ` exactly as above.

`decide_within(inner, outer, tolerance, max_splits)` answers `None` when the shape is not
handled, and then claims nothing:

- `B` is not a zonotope in sparse form (a dependent monomial that is not one factor to
  the power one, or a factor in two monomials);
- the dimension is 1 or above `MAX_DIMENSION = 4` (Leibniz cost grows with `n!`);
- there are more than `MAX_GENERATORS = 12` non-zero generators, inflation included
  (the facet count grows like `m^(n-1)`);
- an entry or an offset is not finite, or the rank is not proven full.

Otherwise it answers `Some(true)` (proven) or `Some(false)` (not proven). `proven_within`
is the boolean shorthand without tolerance. Representations with equal entries (ids
aside) describe the same set and are accepted directly. Sets that merely touch are not
proven, because the rounded-down offsets make the boundary strict.

Cross-checks: in the plane the exact and the general (cofactor) facets give the same
answer on 300 random zonotope and box pairs, more than 20 of them proven. In 3D, every
sampled point (corners and random points) of each proven box lies in the zonotope, over
200 random pairs with more than 10 proven. The flat 3D zonotope and 5D are not decided.
- Containment of the *true* sets does not make the propagation monotone (Girard reduction
  is not), so even a proven dominance can cost completeness; it never costs safety.

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
  reachable set, so rewiring must re-propagate and re-check the subtree. `RobustRrtStar`
  does this in its local tree; payload-carrying trees in `sampling-planning` would let it
  reuse `RrtStar` instead, and a public radius helper would remove the local copy.

**polyzonotope**

- **Set-in-set containment**, at least zonotope-in-zonotope in every dimension and for
  more generators, so that `Dedup::Proven` needs no hull fallback (today `decide_within`
  handles zonotopes in 2 to 4 dimensions with at most 12 generators).
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
  for affine and nonlinear dynamics under `X₀`, `U` and `W`. robust-planning will adapt
  its local trait to that crate's API.

## Testing strategy

- **Steering, goals and witnesses:** a steering that aims into a wall still yields only
  proven-disjoint edges; `ReachChecker` is `Unknown` when steering stops short; goals that
  answer `Unknown` or `NotInside` are never reached; `PolytopeGoal` rejects NaN facets and
  never contains a set one ULP outside; `Unsafe` appears only with a hook witness that
  passes the sanity check.
- **Search:** lattice optima in an open world, hand-checked for straight and diagonal
  goals and identical with A* and uniform cost. A* and uniform cost agree on the slalom,
  and A* expands no more nodes. Every edge of the search tree is audited. Monte-Carlo runs
  along the found path. A passage narrower than the set exhausts the search. No kept node
  is dominated by an older one in its cell. A budget stop can be resumed.
- **Robust RRT*:** hand-built trees with a model whose set grows by 0.1 per edge, so a
  cheaper branch with more edges gives a wider set. A rewire is rejected when a descendant
  becomes unproven, when a goal node would lose the goal, when the edge to the new parent is
  unproven, on a cycle and over budget, and each rejection leaves parents, controls, sets,
  costs, children and goal flags unchanged. A committed rewire re-propagates the subtree
  and its costs match the nominal path. On the slalom: every final edge is re-propagated
  from its parent's *current* set, the stored set matches, and every cover set is checked
  against polyzonotope directly; Monte-Carlo runs along the returned path and other
  branches; the best cost never increases over 1500 iterations and ends at or below
  `RobustRrt`'s for seeds 0, 1 and 2 (an empirical check, not a theorem); same seed, same
  tree; no zero-length edge. A stub steering that stops short is rejected as
  `NotReached`. The unbounded mode in an open world skips and rejects nothing and ends
  within 5 % of the straight line.
- **Weighted cost:** `NotCheaper` and `CostIncreased` rejections on the hand-built tree;
  weighted A* equals weighted uniform cost on the slalom, weight 0 equals length, and
  the path cost matches a recomputation; weighted Robust RRT* never increases its cost
  and every stored cost equals its parent's plus the edge.
- **Search options:** a bounded unframed search exhausts inside its bounds; invalid
  bounds are rejected; `GoalRadius::FromGoal` matches uniform cost with fewer
  expansions and errors on a goal without a radius; `PolytopeGoal::radius` bounds the box.
  Pruning: unit tests on hand-pushed nodes (with and without the option, and the ancestor
  guard); on the slalom it gives the same path, every edge audited, Monte-Carlo along it.
- **Proven containment:** a box whose hull fits a diamond's hull but whose corners stick
  out is not proven inside, smaller boxes are; touching and differently represented
  copies are not, equal representations are; segments, shared factors, 5D, flat 3D,
  too many generators and non-finite inputs are left undecided; planar and cofactor
  facets agree in 2D; sampled points of proven 3D boxes lie in the zonotope; a tolerance
  merges touching copies only up to itself; Leibniz determinants and their bounds.
  `Dedup::Proven` keeps a node the hull test wrongly drops and finds the lattice optimum
  in an open world; an ignored release test reports the slalom expansions.
- **Set bounds:** with `bounds_contain_sets`, every cover and end set is proven inside the
  box (checked with polyzonotope directly), fewer nodes survive than with nominal bounds,
  and a start set that sticks out is rejected only in that mode.
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

## Not yet implemented

- A `Reach` model on `polyzonotope-reach`, with time-interval sets in place of the box hull,
  and nonlinear dynamics.
- Refinement beyond set splitting: time splitting, initial-set bisection, and budget
  escalation.
- Reusing `sampling-planning` trees once they carry per-node payloads; proven containment
  above four dimensions or twelve generators.
- Safety-constrained trajectory planning: optimize nominal controls under proven set
  constraints. Fail-safe maneuvers.
- An optional `temporal-control` adapter for timed plans.

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
