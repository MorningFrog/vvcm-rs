# Changelog

## 1.1.0 - Unreleased

## 1.0.0 - 2026-06-10

- First public release of `vvcm-rs`.
- Rust API for VVCM forward kinematics with domain types including `Point2`, `Point3`, `RobotFormation`, `SheetShape`, and `FkSolution`.
- Stable-solution search with taut-cable-set enumeration, candidate solving, and stable-branch filtering.
- Simulation wrappers for velocity-driven updates and manually supplied robot formations.
- Python bindings published as `vvcm-rs` / `vvcm_rs`, with typed package metadata.
- C ABI and C++17 wrapper headers for native consumers.
- Distribution through crates.io, PyPI, GitHub Releases, and vcpkg overlays.
