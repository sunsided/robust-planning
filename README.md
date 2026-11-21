# robust-planning

[![crates.io](https://img.shields.io/crates/v/robust-planning.svg)](https://crates.io/crates/robust-planning)
[![docs.rs](https://img.shields.io/docsrs/robust-planning)](https://docs.rs/robust-planning)
[![CI](https://github.com/sunsided/robust-planning/actions/workflows/ci.yml/badge.svg)](https://github.com/sunsided/robust-planning/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/sunsided/robust-planning/graph/badge.svg)](https://codecov.io/gh/sunsided/robust-planning)
[![MSRV](https://img.shields.io/crates/msrv/robust-planning)](https://crates.io/crates/robust-planning)
[![License](https://img.shields.io/crates/l/robust-planning)](https://github.com/sunsided/robust-planning#license)
[![unsafe forbidden](https://img.shields.io/badge/unsafe-forbidden-success.svg)](https://github.com/rust-secure-code/safety-dance/)

Planning over reachable sets under uncertainty: Robust-RRT and reachability-guided search.

## Scope

- [x] Robust-RRT: an RRT whose nodes carry reachable sets (`RobustRrt`)
- [x] Reachability-guided search: A* over reachable-set nodes along motion primitives,
  with lattice deduplication, optional workspace bounds on nominal states or whole sets,
  a goal radius derived from the goal, opt-in pruning of superseded subtrees, and opt-in
  proven set-in-set deduplication for zonotopes in up to four dimensions (`ReachSearch`,
  `decide_within`)
- [x] A robustness-weighted cost (`Cost::WidthWeighted`) for `RobustRrtStar` and `ReachSearch`
- [x] Edge checks for `sampling-planning` planners (`ReachChecker`)
- [x] Pluggable steering (`Steer`, `WithSteering`), goals (`Goal`, `PolytopeGoal`) and
  certified collision witnesses (`Reach::certify_collision`)
- [x] Robust RRT*: choose-parent and rewiring over proven-safe edges only; a rewire
  re-propagates and re-proves the whole moved subtree and commits only if all of it stays
  proven safe and no goal node loses the goal, within a per-iteration budget (`RobustRrtStar`)
- [ ] Safety-constrained trajectory planning
- [ ] polyzonotope-reach for reachable sets (a test-only affine model stands in)

Every check is conservative: a check that cannot prove safety returns `Indeterminate`, never
`Safe`, and planners reject it. Proofs are rigorous only with the `sound` feature, which
needs x86-64 Haswell or newer (see
[`.cargo/config.toml`](https://github.com/sunsided/robust-planning/blob/main/.cargo/config.toml)) or AArch64. See
[docs/design.md](https://github.com/sunsided/robust-planning/blob/main/docs/design.md).

## Status

Pre-release, version 0.0.0. Robust-RRT, Robust RRT*, the reachability-guided search and the
`sampling-planning` adapter work. Run `cargo run --example robust_rrt_2d`,
`cargo bench --bench search` and `cargo bench --bench robust_rrt_star`. The API will change.

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
