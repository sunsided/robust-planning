# robust-planning

[![crates.io](https://img.shields.io/crates/v/robust-planning.svg)](https://crates.io/crates/robust-planning)
[![docs.rs](https://img.shields.io/docsrs/robust-planning)](https://docs.rs/robust-planning)
[![CI](https://github.com/sunsided/robust-planning/actions/workflows/ci.yml/badge.svg)](https://github.com/sunsided/robust-planning/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/sunsided/robust-planning/graph/badge.svg)](https://codecov.io/gh/sunsided/robust-planning)
[![MSRV](https://img.shields.io/crates/msrv/robust-planning)](https://crates.io/crates/robust-planning)
[![License](https://img.shields.io/crates/l/robust-planning)](https://github.com/sunsided/robust-planning#license)
[![unsafe forbidden](https://img.shields.io/badge/unsafe-forbidden-success.svg)](https://github.com/rust-secure-code/safety-dance/)

Motion planning that proves its edges safe: planners whose nodes carry reachable sets
(sparse polynomial zonotopes from
[polyzonotope](https://github.com/sunsided/polyzonotope)) instead of single states, so a path is
accepted only when every state the system can reach under bounded disturbances and initial
uncertainty stays clear of the obstacles.

The crate has Robust-RRT, Robust RRT* and a reachability-guided A* over motion primitives,
plus an adapter that lets the planners of
[sampling-planning](https://github.com/sunsided/sampling-planning) check their edges the same
way. Every check is conservative: a check that cannot prove safety returns `Indeterminate`,
never `Safe`, and planners reject it. With the `sound` feature, polyzonotope rounds outward and
a `Safe` verdict becomes a proof rather than "safe up to floating-point rounding".

## Who this is for

- Engineers working on autonomous vehicles, drones or spacecraft who have a dynamics model
  with bounded noise and need a path that is safe for the whole tube around it, not only for
  the nominal trajectory.
- Researchers in set-based verification who want to put their reachability analysis behind
  RRT, RRT* or a lattice search without writing the planner first.
- Anyone who already plans with sampling-planning and wants its RRT, PRM or BIT* to accept
  only proven-safe edges in Euclidean spaces (`ReachChecker`).
- You bring the `Reach` model, i.e. the propagation of a set along one edge; the crate ships
  the planners, obstacles, goals and containment tests around it.
- When the dynamics are deterministic and a collision check on single states is enough,
  [sampling-planning](https://github.com/sunsided/sampling-planning) alone is simpler and much
  faster; for uncertainty in the action outcomes of a discrete model (MDPs, POMDPs),
  [stochastic-search](https://github.com/sunsided/stochastic-search) is the better fit.

## Installation

```toml
[dependencies]
robust-planning = "0.2"
```

Enable the `sound` feature for rigorous verdicts. It needs x86-64 Haswell or newer (see
[`.cargo/config.toml`](https://github.com/sunsided/robust-planning/blob/main/.cargo/config.toml)) or AArch64.
Enable `reach` for the ready-made dynamics models on polyzonotope-reach (`models`).

## Features

- Robust-RRT: an RRT whose nodes carry reachable sets (`RobustRrt`).
- Robust RRT*: choose-parent and rewiring over proven-safe edges only; a rewire
  re-propagates and re-proves the whole moved subtree and commits only if all of it stays
  proven safe and no goal node loses the goal, within a per-iteration budget (`RobustRrtStar`).
- Reachability-guided search: A* over reachable-set nodes along motion primitives,
  with lattice deduplication, optional workspace bounds on nominal states or whole sets,
  a goal radius derived from the goal, opt-in pruning of superseded subtrees, and opt-in
  proven set-in-set deduplication for zonotopes in up to four dimensions (`ReachSearch`,
  `decide_within`).
- A robustness-weighted cost (`Cost::WidthWeighted`) for `RobustRrtStar` and `ReachSearch`.
- Edge checks for `sampling-planning` planners in Euclidean spaces (`ReachChecker`).
- Pluggable steering (`Steer`, `WithSteering`), goals (`Goal`, `PolytopeGoal`) and
  certified collision witnesses (`Reach::certify_collision`).
- Polytope obstacles checked against reachable sets, with optional refinement by splitting
  (`Obstacles`).
- Dynamics models on polyzonotope-reach, behind the `reach` feature: validated
  continuous-time reachable sets of linear (`models::Linear`) and nonlinear
  (`models::Nonlinear`) systems. Their cover sets enclose the motion between time steps.

Not yet implemented: safety-constrained trajectory planning (optimizing nominal controls under
proven set constraints).

## Usage

See the [crate documentation](https://docs.rs/robust-planning) and
[docs/design.md](https://github.com/sunsided/robust-planning/blob/main/docs/design.md) for the
safety semantics. The example plans through a 2D world with an affine model, and the benchmarks
time the search and Robust RRT*:

```sh
cargo run --example robust_rrt_2d
cargo bench --bench search
cargo bench --bench robust_rrt_star
```

## Minimum supported Rust version

Rust 1.89 (edition 2024). The MSRV applies to the library; tests and benchmarks may need a
newer toolchain. It may be raised in a minor release when a dependency or a language
feature calls for it.

## Contributing

See [CONTRIBUTING.md](https://github.com/sunsided/robust-planning/blob/main/CONTRIBUTING.md).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](https://github.com/sunsided/robust-planning/blob/main/LICENSE-APACHE))
- MIT license ([LICENSE-MIT](https://github.com/sunsided/robust-planning/blob/main/LICENSE-MIT))

at your option.

The example in [`examples/`](https://github.com/sunsided/robust-planning/tree/main/examples) is public domain
([CC0-1.0](https://github.com/sunsided/robust-planning/blob/main/LICENSES/CC0-1.0.txt)), so you can copy it into anything.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion
in this work by you, as defined in the Apache-2.0 license, shall be dual licensed as above,
without any additional terms or conditions.
