# robust-planning

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
needs x86-64 Haswell or newer (see `.cargo/config.toml`) or AArch64. See
[docs/design.md](docs/design.md).

## Status

Pre-release, version 0.0.0. Robust-RRT, Robust RRT*, the reachability-guided search and the
`sampling-planning` adapter work. Run `cargo run --example robust_rrt_2d`,
`cargo bench --bench search` and `cargo bench --bench robust_rrt_star`. The API will change.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
