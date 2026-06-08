# Changelog

## Unreleased

- Implemented the Rust `VvcmFk` forward kinematics core.
- Added regression coverage for the C++ README forward-kinematics sample.
- Added a runtime warning for inputs that look too small for millimeter units.
- Added a release-friendly FK timing example based on the C++ 20-robot fixture.
- Completed `VvcmSimulation` and `VvcmManualSimulation` behavior.
- Optimized the `VvcmFk` hot path by reusing precomputed point data and removing unused per-candidate work.
