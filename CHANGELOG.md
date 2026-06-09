# Changelog

## Unreleased

- No unreleased changes.

## 1.0.0 - 2026-06-09

- Initial public release of `vvcm-rs`, a Rust implementation of the Virtual
  Variable Cables Model (VVCM) forward-kinematics workflow for multi-robot
  transportation with a deformable sheet.
- Published packages are available from crates.io, PyPI, GitHub Releases, and
  vcpkg.
- Rust API provides domain types such as `Point2`, `Point3`, `RobotFormation`,
  `SheetShape`, `FkSolution`, `VvcmFk`, `VvcmSimulation`, and
  `VvcmManualSimulation`.
- Forward-kinematics solving supports taut-cable-set enumeration, candidate
  state solving, stable-branch marking, and local filtering of stable
  solutions.
- Simulation APIs support velocity-driven robot-formation updates and manual
  stable-solution queries for externally supplied formations.
- Python package `vvcm-rs` exposes the `vvcm_rs` module with typed PyO3
  bindings, `py.typed`, and `.pyi` type information.
- C and C++ consumers can use the raw C ABI in `include/vvcm_rs.h`, the C++17
  RAII wrapper in `include/vvcm_rs.hpp`, and native shared or static library
  artifacts.
- vcpkg packaging supports CMake consumption through the `vvcm_rs::vvcm_rs`
  target, with the repo-local overlay port available for source builds.
- Examples include a basic forward-kinematics run and a 20-robot FK timing
  benchmark fixture.
- Validation coverage includes Rust smoke tests, README sample coverage, Python
  binding tests, and a Cargo-driven C++ export smoke test.
- The library emits a runtime warning for inputs that appear too small for the
  millimeter-scale fixtures used by the bundled examples and tests.
- Release automation validates version consistency, Rust checks, Python
  distributions, vcpkg packaging, crates.io publishing, PyPI Trusted Publishing,
  and GitHub release assets.
