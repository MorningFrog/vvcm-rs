# vvcm-rs

Rust implementation scaffold for the Virtual Variable Cables Model (VVCM)
forward kinematics algorithm for multi-robot transportation with a deformable
sheet.

This crate is intended to become a Rust implementation of the existing C++ VVCM
library. The current state is a project scaffold: public Rust APIs, module
boundaries, examples, and smoke tests are in place, while the numerical forward
kinematics core still needs to be ported.

## Current Status

- Public API uses Rust domain types such as `Point2`, `Point3`,
  `RobotFormation`, `SheetShape`, and `FkSolution`.
- FK results are stored as one list of `FkSolution` values; each solution has a
  `stable` flag instead of mirroring the C++ implementation's separated stable
  and all-solution arrays.
- `nalgebra` is kept as an internal numerical backend, not as the main public
  interface.
- `VvcmFk::update_stable_solutions` currently returns
  `VvcmError::NotImplemented`.
- Simulation wrappers are scaffolded, but VVCM numerical updates are not yet
  implemented.

## Module Overview

- `fk`: forward kinematics engine state and stable-solution entry point.
- `simulation`: velocity-driven simulation wrapper.
- `manual_simulation`: wrapper for querying a new stable solution from an
  externally provided robot formation.
- `types`: public domain types used by the Rust API.
- `error`: crate error type.

## Quick Start

Run the smoke tests:

```shell
cargo test
```

Run the basic forward-kinematics example:

```shell
cargo run --example basic_fk
```

Example API shape:

```rust
use vvcm_rs::{Point2, RobotFormation, SheetShape, VvcmFk};

let formation = RobotFormation::new(vec![
    Point2::new(213.7, 122.7),
    Point2::new(804.6, 37.2),
    Point2::new(904.0, 550.0),
    Point2::new(439.3, 715.9),
])?;

let sheet = SheetShape::new(vec![
    Point2::new(-316.1, -421.9),
    Point2::new(803.4, -384.1),
    Point2::new(746.1, 712.8),
    Point2::new(-367.3, 664.2),
])?;

let mut fk = VvcmFk::new(4, 1000.0, sheet)?;
let solutions = fk.update_stable_solutions(formation)?;
# Ok::<(), vvcm_rs::VvcmError>(())
```

The default length unit follows the C++ implementation examples: millimeters.

## Porting Roadmap

1. Port the VVCM forward kinematics core, including taut-cable enumeration,
   constrained quadratic solve, polygon feasibility, and stability filtering.
2. Add numerical regression tests against the C++ examples and README sample.
3. Complete `VvcmSimulation` and `VvcmManualSimulation` behavior.
4. Expand documentation once the numerical API is implemented.

## Citation

If you use the forward kinematics algorithm, please cite:

```bibtex
@article{ma2026stable,
  title = {Stable Kinematics for Multi-Robot Collaborative Transporting System with a Deformable Sheet},
  author = {Ma, Wenyao and Hu, Jiawei and Li, Jiamao and Yi, Jingang and Xiong, Zhenhua},
  year = 2026,
  journal = {IEEE Transactions on Robotics},
  volume = {42},
  pages = {837-853},
  doi = {10.1109/TRO.2026.3653870}
}
```

For the original VVCM model, please cite:

```bibtex
@article{hu2022multirobot,
  title = {Multi-Robot Object Transport Motion Planning With a Deformable Sheet},
  author = {Hu, Jiawei and Liu, Wenhang and Zhang, Heng and Yi, Jingang and Xiong, Zhenhua},
  year = 2022,
  journal = {IEEE Robotics and Automation Letters},
  volume = {7},
  number = {4},
  pages = {9350--9357}
}
```
