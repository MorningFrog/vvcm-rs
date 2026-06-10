# vvcm-rs

Rust implementation of the Virtual Variable Cables Model (VVCM) forward kinematics algorithm for multi-robot transportation with a deformable sheet.

`vvcm-rs` is an independent Rust crate for evaluating VVCM forward kinematics and simulation workflows. The forward kinematics core and simulation wrappers are implemented in Rust.

If you plan to modify the codebase, read [CONTRIBUTING.md](CONTRIBUTING.md) first for workflow, structure, and release expectations.

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

## Published Release

Version `1.0.0` (2026-06-10) is the first public release of `vvcm-rs`.

- Rust VVCM forward-kinematics API with `Point2`, `Point3`, `RobotFormation`, `SheetShape`, and `FkSolution`.
- Stable-solution search with taut-cable enumeration, candidate solving, and stable-branch filtering.
- Velocity-driven and manual simulation wrappers.
- Python bindings published as `vvcm-rs` / `vvcm_rs` with typed package metadata.
- C ABI and C++17 wrapper headers for native consumers.
- Distribution through crates.io, PyPI, GitHub Releases, and vcpkg overlays.

## Module Overview

- `fk`: forward kinematics engine state and stable-solution entry point.
- `simulation`: velocity-driven simulation wrapper.
- `manual_simulation`: wrapper for querying a new stable solution from an externally provided robot formation.
- `types`: public domain types used by the Rust API.
- `ffi`: C ABI implementation behind the C/C++ headers.
- `error`: crate error type.

## Installation

For source-based installation or local development, read [CONTRIBUTING.md](CONTRIBUTING.md) first.

### Rust

Use the crate from crates.io:

```shell
cargo add vvcm-rs
```

### Python

Install the package from PyPI:

```shell
python -m pip install vvcm-rs
```

### C and C++

Install the prebuilt package from the GitHub release archive:

```shell
vcpkg install vvcm-rs --overlay-ports=<path-to-unzipped-release>/ports --triplet <platform-triplet>
```

The prebuilt overlay ships native packages for Windows x64, Linux x64, and macOS arm64. It does not require Rust. Use the triplet that matches your platform, such as `x64-windows`, `x64-linux`, or `arm64-osx`.

If you want to build from the repository source instead, use the repo-local overlay port. That overlay builds the native Rust library with Cargo, so Rust must be installed on the machine running vcpkg. Python is only needed when you build the Python extension feature:

```shell
vcpkg install vvcm-rs --overlay-ports=<path-to-vvcm-rs>/vcpkg/ports
```

Then consume the installed CMake package:

```cmake
find_package(vvcm-rs CONFIG REQUIRED)
target_link_libraries(app PRIVATE vvcm_rs::vvcm_rs)
```

## Usage

The language-specific snippets below assume installation is already complete. Choose the section that matches your project.

### Rust Usage

After adding `vvcm-rs` from crates.io, the Rust API looks like this:

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

### C++ Usage

After installing the vcpkg package or a release archive, consume the installed CMake package and headers directly. The package exports the raw C ABI in `vvcm_rs.h` and the C++17 RAII wrapper in `vvcm_rs.hpp`.

```cmake
find_package(vvcm-rs CONFIG REQUIRED)
target_link_libraries(app PRIVATE vvcm_rs::vvcm_rs)
```

```cpp
#include <vvcm_rs.hpp>

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

### Python Usage

After installing `vvcm-rs` from PyPI, import it as `vvcm_rs`. Coordinate collections accept `Point2` values, ordinary `list`/`tuple` rows, or sequence-like two-column arrays such as NumPy `N x 2` arrays.

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

The bundled examples and regression fixtures use millimeters. If `VvcmFk` sees values that look very small for millimeter-scale data, such as meter-scale coordinates, it emits a warning to `stderr`. Convert meter inputs to millimeters before solving, for example by multiplying lengths by `1000.0`.
