---
applyTo: "**/*.rs"
---

# Rust code

- Every module (`//!`), type, field, variant, function, constant and macro has a doc
  comment, private and `pub(crate)` items included. Docs state intent, invariants, units and
  error/panic conditions (`# Errors`, `# Panics`). They do not restate the name.
- No `mod.rs`: modules with children are `foo.rs` plus `foo/`. Integration tests live under
  `tests/it/`; property tests in `tests/it/properties.rs`.
- No separator comments (`// -----`, `// =====`). If a file needs sections, split it into
  submodules.
- Leave a blank line after the closing `}` of a block statement when another statement
  follows, and between logical phases of a function. No lumped code.
- One level of abstraction per function. Planner loops are short drivers that call
  documented step functions. If a block needs a "what" comment, extract it into a function
  named after that comment.
- `clippy::pedantic` clean. Suppression only with a scoped
  `#[expect(lint, reason = "...")]`. Never `#[allow]`, and never crate-level suppression.
- No `unsafe`: the crate root has `#![forbid(unsafe_code)]`.
- Every check is conservative. A check that cannot prove safety returns
  `SafetyVerdict::Indeterminate`, never `Safe`, and planners accept only `Safe`.
  `Unsafe` needs a certified `Witness` from `Reach::certify_collision`, never a point of an
  over-approximation. Reject changes that weaken this.
- Rounding goes outward or offsets go down: proofs are rigorous with the `sound` feature
  and up to floating-point rounding without it. Document which one a new check gives.
- `proven_within` / `decide_within` may answer `None` or `Some(false)` freely, but a
  `Some(true)` must be a proof; the fuzz target and the property tests check that.
- Randomness comes only from a caller-provided RNG. A seeded run must be fully
  reproducible: no hidden RNGs, no iteration over hash maps.
- Public API changes go with a matching update to the README "Scope" list and
  `docs/design.md`.
- Mermaid diagrams in doc comments must match the code they describe. Update them when the
  structure changes.
