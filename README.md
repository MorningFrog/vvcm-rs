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
- C and C++ exports are available through `include/vvcm_rs.h` and the
  header-only C++ wrapper `include/vvcm_rs.hpp`. The crate builds native
  `cdylib` and `staticlib` artifacts for external linking.
- Python bindings are available through the `vvcm_rs` package. The wheel ships
  typed Python package files (`py.typed` and `__init__.pyi`) so editors and type
  checkers can inspect the exported classes.
- Distribution metadata is available for crates.io, PyPI, and the repo-local
  vcpkg overlay port under `vcpkg/ports/vvcm-rs`. The manual GitHub Actions
  `Release` workflow publishes the Rust crate, Python distributions, and a
  vcpkg-ready source archive. Its `dry-run` input defaults to `true` for
  validation-only runs.

## Module Overview

- `fk`: forward kinematics engine state and stable-solution entry point.
- `simulation`: velocity-driven simulation wrapper.
- `manual_simulation`: wrapper for querying a new stable solution from an
  externally provided robot formation.
- `types`: public domain types used by the Rust API.
- `ffi`: C ABI implementation behind the C/C++ headers.
- `error`: crate error type.

## Installation

Use the Rust crate from crates.io:

```shell
cargo add vvcm-rs
```

Install the Python package from PyPI:

```shell
python -m pip install vvcm-rs
```

Use the C/C++ package through the repo-local vcpkg overlay port. The overlay
builds the native Rust library with Cargo, so Rust must be installed on the
machine running vcpkg:

```shell
vcpkg install vvcm-rs --overlay-ports=<path-to-vvcm-rs>/vcpkg/ports
```

Then consume the installed CMake package:

```cmake
find_package(vvcm-rs CONFIG REQUIRED)
target_link_libraries(app PRIVATE vvcm_rs::vvcm_rs)
```

## Quick Start

Run the smoke tests. The C++ export smoke test requires a C++17 compiler:

```shell
cargo test
```

Run only the C++ export smoke test:

```shell
cargo test --test cpp_export_smoke
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

## C++ Usage

Build the native library artifacts:

```shell
cargo build --lib
```

On Windows this produces `target/debug/vvcm_rs.dll`,
`target/debug/vvcm_rs.dll.lib`, and `target/debug/vvcm_rs.lib`. On Linux and
macOS, link against the generated `libvvcm_rs` shared or static library under
the corresponding target profile directory.

Include `include/vvcm_rs.hpp` for the C++17 RAII wrapper, or
`include/vvcm_rs.h` for the raw C ABI. The C++ wrapper accepts ordinary
`std::vector<vvcm_rs::Point2>` coordinate lists, exposes `VvcmFk`,
`VvcmSimulation`, and `VvcmManualSimulation`, and throws `vvcm_rs::Error` when
the Rust solver reports an error.

```cpp
#include "vvcm_rs.hpp"

#include <iostream>
#include <vector>

int main() {
    using namespace vvcm_rs;

    std::vector<Point2> formation = {
        Point2(213.7f, 122.7f),
        Point2(804.6f, 37.2f),
        Point2(904.0f, 550.0f),
        Point2(439.3f, 715.9f),
    };
    std::vector<Point2> sheet = {
        Point2(-316.1f, -421.9f),
        Point2(803.4f, -384.1f),
        Point2(746.1f, 712.8f),
        Point2(-367.3f, 664.2f),
    };

    VvcmFk fk(4, 1000.0f, sheet);
    FkSolutions solutions = fk.update_stable_solutions(formation);

    for (const auto &solution : solutions.stable()) {
        std::cout << solution.po.x << " "
                  << solution.po.y << " "
                  << solution.po.z << "\n";
    }
}
```

## Python Usage

Install the published package from PyPI:

```shell
python -m pip install vvcm-rs
```

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
