# Changelog

## Unreleased

- Implemented the Rust `VvcmFk` forward kinematics core.
- Added regression coverage for the README forward-kinematics sample.
- Added a runtime warning for inputs that look too small for millimeter units.
- Added a release-friendly FK timing example for a 20-robot benchmark fixture.
- Completed `VvcmSimulation` and `VvcmManualSimulation` behavior.
- Optimized the `VvcmFk` hot path by reusing precomputed point data and removing unused per-candidate work.
- Documented the public Rust API and key internal FK/simulation steps.
- Added typed PyO3 Python bindings, package metadata, and Python regression tests.
- Added a C ABI, header-only C++ wrapper, native `staticlib` output, and a
  Cargo-driven C++ smoke test.
