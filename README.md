# vvcm-rs

Rust implementation for kinematics of multi-robot transporting systems with a deformable sheet using the Virtual Variable Cables Model (VVCM).

`vvcm-rs` is implemented in Rust, but it is not limited to Rust projects. The same VVCM forward-kinematics and simulation library is available to **Rust**, **JavaScript/TypeScript**, **C++**, and **Python** users through the native Rust API, WebAssembly npm packages, C ABI/C++17 wrapper headers, and Python bindings.

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

## Features

This package includes `vvcm-rs` for Rust, JavaScript/TypeScript, Python, and C/C++ users with:

- A Rust VVCM forward-kinematics API built around `Point2`, `Point3`, `RobotFormation`, `SheetShape`, and `FkSolution`.
- Stable-solution search with taut-cable enumeration, candidate solving, and stable-branch filtering.
- Velocity-driven and manual simulation wrappers.
- WebAssembly bindings published to npm as `@morningfrog/vvcm-rs` and the unscoped mirror `vvcm-rs`, with hand-written TypeScript declarations.
- Python bindings published as `vvcm-rs` / `vvcm_rs` with typed package metadata.
- C ABI and C++17 wrapper headers for native consumers.
- Distribution through crates.io, npm, PyPI, GitHub Releases, and vcpkg overlays.

## Module Overview

- `fk`: forward kinematics engine state and stable-solution entry point.
- `simulation`: velocity-driven simulation wrapper.
- `manual_simulation`: wrapper for querying a new stable solution from an externally provided robot formation.
- `types`: public domain types used by the Rust API.
- `ffi`: C ABI implementation behind the C/C++ headers.
- `wasm`: WebAssembly bindings compiled with the `wasm` feature for npm packages.
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

Prebuilt PyPI wheels are published for CPython 3.10 through 3.14 on Windows x64, Linux x64, and macOS arm64. Python 3.9 and other platforms may fall back to building from the source distribution, which requires a local Rust toolchain and Python build tooling.

### JavaScript and TypeScript

Install the WebAssembly package from npm:

```shell
npm install @morningfrog/vvcm-rs
```

The unscoped mirror package is also published for users who prefer the shorter install name:

```shell
npm install vvcm-rs
```

The npm packages target modern bundlers such as Vite, Webpack, and Rollup. They include `index.d.ts` TypeScript declarations and expose ready-to-use named exports from the package entry point.

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

The sample outputs below round floating-point values to three decimals; small platform differences are normal.

### Rust Usage

After adding `vvcm-rs` from crates.io, the Rust API looks like this:

```rust
use vvcm_rs::{Point2, RobotFormation, SheetShape, VvcmError, VvcmFk};

fn main() -> Result<(), VvcmError> {
    // Robot formation: each Point2 is a robot node position on the world-coordinate XY plane, in millimeters.
    let formation = RobotFormation::new(vec![
        Point2::new(213.7, 122.7),
        Point2::new(804.6, 37.2),
        Point2::new(904.0, 550.0),
        Point2::new(439.3, 715.9),
    ])?;

    // Unfolded sheet: each Point2 is a vertex in the sheet's local coordinate frame, in millimeters.
    let sheet = SheetShape::new(vec![
        Point2::new(-316.1, -421.9),
        Point2::new(803.4, -384.1),
        Point2::new(746.1, 712.8),
        Point2::new(-367.3, 664.2),
    ])?;

    // Create the FK solver for four robots with a 1000 mm hold height.
    let mut fk = VvcmFk::new(4, 1000.0, sheet)?;

    // Ask the solver to enumerate every candidate equilibrium for this formation.
    let solutions = fk.update_stable_solutions(formation)?;

    // Report the total branch count and the subset that is stable.
    println!("all solutions: {}", solutions.all_count());
    println!("stable solutions: {}", solutions.stable_count());

    // Print each stable branch with object pose, virtual object point, and taut cables.
    for (index, solution) in solutions.stable().enumerate() {
        println!(
            "#{index}: Po=({:.3}, {:.3}, {:.3}), Vo=({:.3}, {:.3}), taut={:?}",
            solution.po.x,
            solution.po.y,
            solution.po.z,
            solution.vo.x,
            solution.vo.y,
            solution.taut_cables,
        );
    }

    Ok(())
}
```

Expected output:

```text
all solutions: 3
stable solutions: 2
#0: Po=(568.841, 324.728, 336.736), Vo=(238.633, 125.028), taut=[0, 1, 2]
#1: Po=(557.919, 341.232, 337.247), Vo=(208.794, 152.532), taut=[0, 2, 3]
```

### JavaScript and TypeScript Usage

After installing `@morningfrog/vvcm-rs` or `vvcm-rs` from npm, import the WebAssembly module and use the same solver concepts as the native APIs. Coordinate inputs accept `[x, y]` tuples or `{ x, y }` objects.

```ts
import { VvcmFk } from "@morningfrog/vvcm-rs";

// Robot formation: each tuple is a robot node position on the world-coordinate XY plane, in millimeters.
const formation = [
  [213.7, 122.7],
  [804.6, 37.2],
  [904.0, 550.0],
  [439.3, 715.9],
] as const;

// Unfolded sheet: each tuple is a vertex in the sheet's local coordinate frame, in millimeters.
const sheet = [
  [-316.1, -421.9],
  [803.4, -384.1],
  [746.1, 712.8],
  [-367.3, 664.2],
] as const;

// Create the FK solver for four robots with a 1000 mm hold height.
const fk = new VvcmFk(4, 1000, sheet);

// Solve all candidate equilibria for the current formation.
const solutions = fk.updateStableSolutions(formation);

// Report the total branch count and the subset that is stable.
console.log(`all solutions: ${solutions.allCount}`);
console.log(`stable solutions: ${solutions.stableCount}`);

// Print each stable branch with object pose, virtual object point, and taut cables.
solutions.solutions
  .filter((solution) => solution.stable)
  .forEach((solution, index) => {
    const po = solution.po;
    const vo = solution.vo;
    console.log(
      `#${index}: Po=(${po.x.toFixed(3)}, ${po.y.toFixed(3)}, ${po.z.toFixed(3)}), ` +
        `Vo=(${vo.x.toFixed(3)}, ${vo.y.toFixed(3)}), taut=${JSON.stringify(solution.tautCables)}`,
    );
  });

fk.free();
```

Expected output:

```text
all solutions: 3
stable solutions: 2
#0: Po=(568.841, 324.728, 336.736), Vo=(238.633, 125.028), taut=[0,1,2]
#1: Po=(557.919, 341.232, 337.247), Vo=(208.794, 152.532), taut=[0,2,3]
```

### C++ Usage

After installing the vcpkg package or a release archive, consume the installed CMake package and headers directly. The package exports the raw C ABI in `vvcm_rs.h` and the C++17 RAII wrapper in `vvcm_rs.hpp`.

```cmake
find_package(vvcm-rs CONFIG REQUIRED)
target_link_libraries(app PRIVATE vvcm_rs::vvcm_rs)
```

```cpp
#include <vvcm_rs.hpp>

#include <cstddef>
#include <iomanip>
#include <iostream>
#include <vector>

int main() {
    using namespace vvcm_rs;

    // Robot formation: each Point2 is a robot node position on the world-coordinate XY plane, in millimeters.
    const std::vector<Point2> formation = {
        Point2(213.7f, 122.7f),
        Point2(804.6f, 37.2f),
        Point2(904.0f, 550.0f),
        Point2(439.3f, 715.9f),
    };

    // Unfolded sheet: each Point2 is a vertex in the sheet's local coordinate frame, in millimeters.
    const std::vector<Point2> sheet = {
        Point2(-316.1f, -421.9f),
        Point2(803.4f, -384.1f),
        Point2(746.1f, 712.8f),
        Point2(-367.3f, 664.2f),
    };

    // Build the solver for four robots and a 1000 mm hold height.
    VvcmFk fk(4, 1000.0f, sheet);
    // Solve all candidate equilibria for the current formation.
    FkSolutions solutions = fk.update_stable_solutions(formation);

    // Report the total branch count and the subset that is stable.
    std::cout << "all solutions: " << solutions.all_count() << "\n";
    std::cout << "stable solutions: " << solutions.stable_count() << "\n";

    // Print each stable branch with object pose, virtual object point, and taut cables.
    std::cout << std::fixed << std::setprecision(3);
    const std::vector<FkSolution> stable = solutions.stable();
    for (std::size_t index = 0; index < stable.size(); ++index) {
        const auto &solution = stable[index];
        std::cout << "#" << index << ": Po=("
                  << solution.po.x << ", "
                  << solution.po.y << ", "
                  << solution.po.z << "), Vo=("
                  << solution.vo.x << ", "
                  << solution.vo.y << "), taut=[";
        for (std::size_t taut_index = 0; taut_index < solution.taut_cables.size(); ++taut_index) {
            if (taut_index > 0) {
                std::cout << ", ";
            }
            std::cout << solution.taut_cables[taut_index];
        }
        std::cout << "]\n";
    }
}
```

Expected output:

```text
all solutions: 3
stable solutions: 2
#0: Po=(568.841, 324.728, 336.736), Vo=(238.633, 125.028), taut=[0, 1, 2]
#1: Po=(557.919, 341.232, 337.247), Vo=(208.794, 152.532), taut=[0, 2, 3]
```

### Python Usage

After installing `vvcm-rs` from PyPI, import it as `vvcm_rs`. Coordinate collections accept `Point2` values, ordinary `list`/`tuple` rows, or sequence-like two-column arrays such as NumPy `N x 2` arrays.

```python
from vvcm_rs import VvcmFk

# Robot formation: each tuple is a robot node position on the world-coordinate XY plane, in millimeters.
formation = [
    (213.7, 122.7),
    (804.6, 37.2),
    (904.0, 550.0),
    (439.3, 715.9),
]
# Unfolded sheet: each tuple is a vertex in the sheet's local coordinate frame, in millimeters.
sheet = [
    (-316.1, -421.9),
    (803.4, -384.1),
    (746.1, 712.8),
    (-367.3, 664.2),
]

# Create the solver for four robots and a 1000 mm hold height.
fk = VvcmFk(4, 1000.0, sheet)
# Solve all candidate equilibria for the current formation.
solutions = fk.update_stable_solutions(formation)

# Report the total branch count and the subset that is stable.
print(f"all solutions: {solutions.all_count()}")
print(f"stable solutions: {solutions.stable_count()}")

# Print each stable branch with object pose, virtual object point, and taut cables.
for index, solution in enumerate(solutions.stable()):
    print(
        f"#{index}: Po=({solution.po.x:.3f}, {solution.po.y:.3f}, {solution.po.z:.3f}), "
        f"Vo=({solution.vo.x:.3f}, {solution.vo.y:.3f}), taut={solution.taut_cables}"
    )
```

Expected output:

```text
all solutions: 3
stable solutions: 2
#0: Po=(568.841, 324.728, 336.736), Vo=(238.633, 125.028), taut=[0, 1, 2]
#1: Po=(557.919, 341.232, 337.247), Vo=(208.794, 152.532), taut=[0, 2, 3]
```

Length units are not encoded in the API. Use one consistent unit for formation coordinates, sheet coordinates, and hold height; `VvcmFk` normalizes coordinates internally for numerical stability and maps returned object positions and virtual object points back to the original coordinate frames.

## Error Handling

Forward-kinematics and simulation solves report failures through each language's normal error channel. Error messages are intended for human diagnostics; branch on Rust enum variants, JavaScript error codes, Python exception classes, or C/C++ error codes when program logic needs to distinguish failure modes.

The snippets below show one simple handling pattern for each language: catch the failure, print the message, and branch on the typed error when you need a specific recovery path.

### Rust

```rust
match fk.update_stable_solutions(formation) {
    Ok(solutions) => {
        println!("stable solutions: {}", solutions.stable_count());
    }
    Err(vvcm_rs::VvcmError::InfeasibleFormation) => {
        eprintln!("formation is infeasible");
    }
    Err(error) => {
        eprintln!("vvcm-rs solve failed: {error}");
    }
}
```

The main solve errors in Rust are:

- `VvcmError::DimensionMismatch` for input size mismatches during construction or solve setup.
- `VvcmError::InfeasibleFormation` when the robot formation cannot be realized by the sheet geometry.
- `VvcmError::NoSolution` when no candidate branch can be constructed.
- `VvcmError::NoStableSolution` when candidate branches exist but none are stable.
- `VvcmError` remains the common error type, so `Err(error)` still catches any of them and `Err(vvcm_rs::VvcmError::InfeasibleFormation)` can catch one case specifically.

### Python

```python
from vvcm_rs import InfeasibleFormationError, VvcmError

try:
    solutions = fk.update_stable_solutions(formation)
except InfeasibleFormationError as error:
    print(f"formation is infeasible: {error}")
except VvcmError as error:
    print(f"vvcm-rs solve failed: {error}")
else:
    print(f"stable solutions: {solutions.stable_count()}")
```

The main solve errors in Python are:

- `DimensionMismatchError` for input size mismatches during construction or solve setup.
- `InfeasibleFormationError` when the robot formation cannot be realized by the sheet geometry.
- `NoSolutionError` when no candidate branch can be constructed.
- `NoStableSolutionError` when candidate branches exist but none are stable.
- `VvcmError` remains the common base class, so `except VvcmError as error` still catches any of them and `except InfeasibleFormationError as error` can catch one case specifically.

### JavaScript and TypeScript

```ts
import { VvcmFk, type VvcmError } from "@morningfrog/vvcm-rs";

function isVvcmError(error: unknown): error is VvcmError {
  return error instanceof Error && error.name === "VvcmError" && "code" in error;
}

try {
  const solutions = fk.updateStableSolutions(formation);
  console.log(`stable solutions: ${solutions.stableCount}`);
} catch (error) {
  if (isVvcmError(error) && error.code === "INFEASIBLE_FORMATION") {
    console.error("formation is infeasible");
  } else {
    console.error("vvcm-rs solve failed:", error);
  }
}
```

The main solve errors in JavaScript and TypeScript are:

- `DIMENSION_MISMATCH` for input size mismatches during construction or solve setup.
- `INFEASIBLE_FORMATION` when the robot formation cannot be realized by the sheet geometry.
- `NO_SOLUTION` when no candidate branch can be constructed.
- `NO_STABLE_SOLUTION` when candidate branches exist but none are stable.
- `INVALID_ARGUMENT` when a JavaScript value cannot be parsed as the expected point, formation, or sheet input shape.

### C

```c
VvcmRsErrorCode code = vvcm_rs_fk_update_stable_solutions(
    fk,
    formation_points,
    formation_point_count);
if (code != VVCM_RS_ERROR_OK) {
    fprintf(stderr, "vvcm-rs failed: %s\n", vvcm_rs_last_error_message());
    if (code == VVCM_RS_ERROR_INFEASIBLE_FORMATION) {
        fprintf(stderr, "formation is infeasible\n");
    }
}
```

The main solve errors in C are:

- `VVCM_RS_ERROR_DIMENSION_MISMATCH` for input size mismatches during construction or solve setup.
- `VVCM_RS_ERROR_INFEASIBLE_FORMATION` when the robot formation cannot be realized by the sheet geometry.
- `VVCM_RS_ERROR_NO_SOLUTION` when no candidate branch can be constructed.
- `VVCM_RS_ERROR_NO_STABLE_SOLUTION` when candidate branches exist but none are stable.
- `vvcm_rs_last_error_message()` returns the human-readable message for the most recent failure on the current thread, while `vvcm_rs_error_message(code)` returns the generic message for a given code.

### C++

```cpp
try {
    vvcm_rs::FkSolutions solutions = fk.update_stable_solutions(formation);
    std::cout << "stable solutions: " << solutions.stable_count() << "\n";
} catch (const vvcm_rs::Error &error) {
    std::cerr << "vvcm-rs failed: " << error.what()
              << " (code " << error.code() << ")\n";
    if (error.code() == VVCM_RS_ERROR_INFEASIBLE_FORMATION) {
        std::cerr << "formation is infeasible\n";
    }
}
```

The main solve errors in C++ are:

- `VVCM_RS_ERROR_DIMENSION_MISMATCH` for input size mismatches during construction or solve setup.
- `VVCM_RS_ERROR_INFEASIBLE_FORMATION` when the robot formation cannot be realized by the sheet geometry.
- `VVCM_RS_ERROR_NO_SOLUTION` when no candidate branch can be constructed.
- `VVCM_RS_ERROR_NO_STABLE_SOLUTION` when candidate branches exist but none are stable.
- `vvcm_rs::Error` keeps the originating code, so `catch (const vvcm_rs::Error &error)` still handles all failures and `error.code()` lets you branch on one case specifically.
