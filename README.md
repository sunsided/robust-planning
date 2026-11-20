# robust-planning

Planning over reachable sets under uncertainty: Robust-RRT and reachability-guided search.

## Scope

- Reachability-guided tree search
- Robust-RRT
- Safety-constrained trajectory planning
- Uses polyzonotope-reach for reachable sets

Every check is conservative: a check that cannot prove safety returns `Indeterminate`, never
`Safe`, and planners reject it. Proofs are rigorous only with the `sound` feature, which
needs x86-64 Haswell or newer (see `.cargo/config.toml`) or AArch64. See
[docs/design.md](docs/design.md).

## Status

Pre-release, version 0.0.0. Milestone 1 works: a `ValidityChecker` adapter for
`sampling-planning` and a Robust-RRT whose nodes carry reachable sets. Run
`cargo run --example robust_rrt_2d`. Steering (`Steer`, `WithSteering`), goals (`Goal`,
`PolytopeGoal`) and certified collision witnesses (`Reach::certify_collision`) are
pluggable. The API will change.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
