# Changelog

## 1.1.0 - 2026-06-11

- Added automatic FK coordinate normalization to improve numerical stability for small or translated inputs while returning results in the caller's original coordinate frames.
- Added typed Python solve exceptions and C++ wrapper error-code access so callers can distinguish infeasible, no-solution, and no-stable-solution failures without parsing messages.

## 1.0.0 - 2026-06-10

- First public release of `vvcm-rs`.
- Rust API for VVCM forward kinematics with domain types including `Point2`, `Point3`, `RobotFormation`, `SheetShape`, and `FkSolution`.
- Stable-solution search with taut-cable-set enumeration, candidate solving, and stable-branch filtering.
- Simulation wrappers for velocity-driven updates and manually supplied robot formations.
- Python bindings published as `vvcm-rs` / `vvcm_rs`, with typed package metadata.
- C ABI and C++17 wrapper headers for native consumers.
- Distribution through crates.io, PyPI, GitHub Releases, and vcpkg overlays.
