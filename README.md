# vvcm-rs

Rust implementation of the Virtual Variable Cables Model (VVCM) forward
kinematics algorithm for multi-robot transportation with a deformable sheet.

`vvcm-rs` is an independent Rust crate for evaluating VVCM forward kinematics
and simulation workflows. The forward kinematics core and simulation wrappers
are implemented in Rust.

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

## Current Status

- Public API uses Rust domain types such as `Point2`, `Point3`,
  `RobotFormation`, `SheetShape`, and `FkSolution`.
- FK results are stored as one list of `FkSolution` values; each solution has a
  `stable` flag for filtering locally stable branches.
- `nalgebra` is kept as an internal numerical backend, not as the main public
  interface.
- `VvcmFk::update_stable_solutions` enumerates taut cable sets, solves candidate
  forward-kinematics states, and marks stable solutions.
- `VvcmSimulation` integrates robot velocities over a fixed time step and keeps
  the closest stable FK branch.
- `VvcmManualSimulation` returns the closest stable FK branch for externally
  supplied robot formations.
- Python bindings are available through the `vvcm_rs` package. The wheel ships
  typed Python package files (`py.typed` and `__init__.pyi`) so editors and type
  checkers can inspect the exported classes.

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

Run the 20-robot FK timing benchmark example in release mode:

```shell
cargo run --release --example fk_timing
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
for solution in solutions.stable() {
    println!("{:?}", solution.po);
}
# Ok::<(), vvcm_rs::VvcmError>(())
```

## Python Usage

Build and install the Python extension in a virtual environment:

```shell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip maturin pytest numpy
maturin develop
python -m pytest tests/python
```

The Python package name on PyPI is `vvcm-rs`, and the import name is
`vvcm_rs`. Coordinate collections accept `Point2` values, ordinary
`list`/`tuple` rows, or sequence-like two-column arrays such as NumPy `N x 2`
arrays.

```python
from vvcm_rs import VvcmFk

formation = [
    (213.7, 122.7),
    (804.6, 37.2),
    (904.0, 550.0),
    (439.3, 715.9),
]
sheet = [
    (-316.1, -421.9),
    (803.4, -384.1),
    (746.1, 712.8),
    (-367.3, 664.2),
]

fk = VvcmFk(4, 1000.0, sheet)
solutions = fk.update_stable_solutions(formation)

for solution in solutions.stable():
    print(solution.po.as_tuple(), solution.vo.as_tuple(), solution.taut_cables)
```

The bundled examples and regression fixtures use millimeters. If `VvcmFk` sees
values that look very small for millimeter-scale data, such as meter-scale
coordinates, it emits a warning to `stderr`. Convert meter inputs to millimeters
before solving, for example by multiplying lengths by `1000.0`.

## Roadmap

1. Broaden numerical regression tests across additional robot/sheet
   configurations.
2. Expand documentation for algorithm details and expected numeric tolerances.
