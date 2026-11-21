---
applyTo: ".github/workflows/**,Taskfile.dist.yaml,**/Taskfile.dist.yaml,.pre-commit-config.yaml"
---

# CI, Taskfile and hooks stay in sync

- The `ci` task in `Taskfile.dist.yaml` mirrors the jobs in `.github/workflows/ci.yml`. A
  change that adds, removes or renames a CI job, or changes its flags, must make the same
  change to `task ci` and the task it calls, and the other way round. Flag any drift in review.
- Local hooks in `.pre-commit-config.yaml` call Taskfile tasks. They must not duplicate
  commands.
- Every `uses:` is pinned to a full 40-character commit SHA, with a `# vX.Y.Z` comment. Reject
  tag or branch refs.
- `dtolnay/rust-toolchain` is pinned to a `master` commit (`# master`) and always gets an
  explicit `toolchain:` input. Its `stable`/`nightly` branches are not tags, and Dependabot
  cannot follow them: it re-pinned the `nightly` use to the `stable` commit.
- The workflow sets `permissions: {}` at the top, and each job sets the least privilege it
  needs. Every `actions/checkout` sets `persist-credentials: false`.
- No `pull_request_target`. No `${{ github.event.* }}` or other untrusted expressions
  directly in `run:`. Pass them through `env:`.
- Do not rename `publish-crates-io.yml` or its `crates-io` environment. crates.io trusted publishing is
  bound to both. Deploy jobs run in their own tag-restricted environment, and `needs:` a
  verification job.
- CI jobs: `branch`, `fmt`, `clippy`, `style`, `doc`, `test` (Linux only), `coverage`,
  `deny`, `msrv`, `semver`, `fuzz`, plus `actionlint` and `zizmor` in `actions-lint.yml`.
- `sampling-planning` is an unreleased path dependency. Every job that builds checks the
  repository out into `robust-planning/` and the private sibling next to it through
  `$/.github/actions/checkout-siblings` (token `SIBLING_REPOS_TOKEN`), and sets
  `defaults.run.working-directory: robust-planning`. `uses:` steps that read the crate need
  an explicit path input (manifest path, cache workspace, Codecov working directory).
  Remove the sibling checkout once the dependency moves to a crates.io release.
- `.github/actionlint.yaml` ignores only actionlint's complaint about the `$/` syntax. Do
  not widen it.
- `cargo fmt` runs without `--all`: `--all` also formats path dependencies and would
  rewrite the sibling checkout.
- The `sound` feature (inari) needs `-C target-cpu=haswell` on x86-64. `.cargo/config.toml`
  sets it for commands run inside the repository; jobs whose action runs cargo from the
  workspace root (semver, release-plz) set `RUSTFLAGS` themselves.
- `msrv` checks the library only (`cargo hack check --rust-version --all-features`, no
  `--all-targets`): dev-dependencies may need a newer Rust. Do not add `--all-targets` back.
- `fuzz/` is a separate nightly workspace (target `decide_within`). Its tasks
  live in `fuzz/Taskfile.dist.yaml`, included as `fuzz:`; CI's `fuzz` job mirrors
  `task fuzz:build` and `task fuzz:lint`.
- No `Cargo.lock` is tracked (root or `fuzz/`): this is a library, and local builds in the
  planning hub patch published siblings to their working trees (the hub's
  `.cargo/config.toml`), which rewrites the lock on every build. CI resolves fresh from
  crates.io, and no step uses `--locked`.
- `scripts/check-branch.sh` is the single definition of allowed branch names. It is used by
  the Taskfile, the hook and the CI `branch` job.
- New jobs that should gate merges must also be added to branch protection. Mention that in
  the PR.
