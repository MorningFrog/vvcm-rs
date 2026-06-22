# Changelog

## 2.0.0 - 2026-06-22

- Breaking: replaced the wrapper-heavy public geometry model with `nalgebra`-first Rust aliases and slice-based formation inputs. `RobotFormation` and `SheetShape` are removed, and `VvcmFk::new` now derives the robot count from the sheet.
- Breaking: changed Python bindings to a NumPy-first API. Formation, sheet, velocity, and point inputs must be C-contiguous `float32` arrays, and FK results are exposed as `FkSolution` objects instead of flattened buffers.
- Breaking: changed C++, C, and JavaScript/TypeScript exports to row-major matrix-view or typed-array inputs with Rust-like per-solution FK result objects and per-index C query functions.
- Performance: reduced conversion and allocation overhead by exposing the internal numeric representation directly where possible.

## 1.3.0 - 2026-06-20

- Added taut-cable `lambda_values` to FK solutions across Rust, Python, C++, C, and JavaScript/TypeScript exports. Rust `FkSolution` and C `VvcmRsFkSolution` now include an additional public field, so old headers should not be mixed with new native libraries.
- Updated Python development setup instructions to use a `uv` virtual environment.

## 1.2.0 - 2026-06-12

- Added WebAssembly bindings for frontend usage with `wasm-bindgen`, covering forward kinematics, velocity-driven simulation, and manual simulation wrappers.
- Added npm packaging for both `@morningfrog/vvcm-rs` and `vvcm-rs`, including hand-written TypeScript declarations.

## 1.1.0 - 2026-06-11

- Added automatic FK coordinate normalization to improve numerical stability for small or translated inputs while returning results in the caller's original coordinate frames.
- Added typed Python solve exceptions and C++ wrapper error-code access so callers can distinguish infeasible, no-solution, and no-stable-solution failures without parsing messages.

## 1.0.0 - 2026-06-10

- First public release of `vvcm-rs`.
- Rust API for VVCM forward kinematics with domain types including `Point2`, `Point3`, `RobotFormation`, `SheetShape`, and `FkSolution`.
- Stable-solution search with taut-cable-set enumeration, candidate solving, and stable-branch filtering.
- Simulation wrappers for velocity-driven updates and manually supplied robot formations.
- Python bindings published as `vvcm-rs` / `vvcm_rs`, with typed package metadata.
- C++17 wrapper headers and C ABI for native consumers.
- Distribution through crates.io, PyPI, GitHub Releases, and vcpkg overlays.
