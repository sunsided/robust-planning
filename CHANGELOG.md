# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0](https://github.com/sunsided/robust-planning/releases/tag/v0.1.0) - 2026-11-21

### Added

- Add dormant release automation with release-plz
- Add CONTRIBUTING.md
- Add git hooks and Copilot review instructions
- Add README badges, MSRV, contributing and license sections
- Add property tests for verdicts, edge costs and proven containment
- Add a fuzz target for proven zonotope containment
- Add REUSE licensing metadata
- Add cargo-deny, Dependabot and typos configuration
- Add the Codecov configuration
- Add the Taskfile and CI workflows
- Add workspace bounds, a goal-derived radius and subtree pruning to ReachSearch
- Add a robustness-weighted edge cost to Robust RRT* and ReachSearch
- Add Robust RRT* with transactional subtree re-propagation
- Add ReachSearch, an A* over reachable-set nodes
- Add a Goal trait over reachable sets and a polytope goal
- Add conservative reachable-set safety checks and Robust-RRT
- Add polyzonotope and sampling-planning dependencies

### Changed

- Set the version to 0.1.0 for the first release
- Update polyzonotope to 0.4
- Depend on sampling-planning from crates.io
- Describe the released scope and audience in the README
- Call the sibling checkout action by its local path
- Point release-plz at its config from the workspace root
- Extend the lint table and clippy.toml
- Render mermaid diagrams in the rustdoc output
- Give proven dedup a default tolerance of 1e-9
- Prove containment in zonotopes up to 4D, add a dedup tolerance and set bounds
- Prove set-in-set containment for planar zonotopes; opt-in proven dedup
- State when the RRT* optimality argument carries over; add unbounded mode
- Skip zero-length extensions in Robust RRT* and test short steering
- Claim no convergence for Robust RRT*, only a non-increasing cost
- Document Robust RRT* in the README and the design notes
- Document ReachSearch and update the README scope and status
- Pass the edge to the collision hook and document the new seams
- Carry a certified witness in SafetyVerdict::Unsafe via a Reach hook
- Split steering out of Reach into its own Steer trait
- Tighten the test model and move nalgebra to dev-dependencies
- Document the design and milestone 1
- Test Robust-RRT with an affine model and add a 2D example
- Reserve the crate name with a 0.0.0 placeholder

### Fixed

- Fix the optimality conditions, guard proven_within, bench proven dedup
