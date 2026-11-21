---
applyTo: "**/*.md,docs/**"
---

# Documentation

- README files are rendered on crates.io and docs.rs. Every link to a repo file and every
  image uses an absolute URL (`https://github.com/...`, `https://raw.githubusercontent.com/...`).
  Only in-page anchors may be relative.
- Mermaid diagrams (```` ```mermaid ````) must be valid (`task lint:mermaid` renders them)
  and must match the code. Flag diagrams that the change makes stale. Keep each diagram under
  about 15 nodes. The module docs of `robust_rrt`, `robust_rrt_star`, `search` and
  `containment` carry diagrams.
- Code blocks are tagged with their language. The README has no Rust blocks and is not
  doctested; if it gains a Rust example, include it through a private `#[cfg(doctest)]`
  module in `src/lib.rs` so it compiles. The crate-level example lives in `src/lib.rs`.
- `docs/design.md` records the design and milestones. Update it when a planner, trait or
  check is added or changes shape, and keep the README "Scope" list in step.
- Keep badges, the License section and the MSRV statement consistent with `Cargo.toml`.
