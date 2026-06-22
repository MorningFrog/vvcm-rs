# vvcm-rs

Rust implementation for kinematics of multi-robot transporting systems with a deformable sheet using the Virtual Variable Cables Model (VVCM).

`vvcm-rs` is implemented in Rust, but it is not limited to Rust projects. The same VVCM forward-kinematics and simulation library is available to **Rust**, **Python**, **C++**, **C**, and **JavaScript/TypeScript** users through the native Rust API, Python bindings, C++17 wrapper headers, C ABI, and WebAssembly npm packages.

If you plan to modify the codebase, read [CONTRIBUTING.md](CONTRIBUTING.md) first for workflow, structure, and release expectations.

**Open the visual test bench at [vvcm-web](https://morningfrog.github.io/vvcm-web/).**

[![VVCM Visual Test Bench live preview](https://api.microlink.io/?url=https%3A%2F%2Fmorningfrog.github.io%2Fvvcm-web%2F&screenshot=true&viewport.width=1920&viewport.height=1080&embed=screenshot.url)](https://morningfrog.github.io/vvcm-web/)

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

This package includes `vvcm-rs` for Rust, Python, C++, C, and JavaScript/TypeScript users with:

- A Rust VVCM forward-kinematics API that exposes `nalgebra` `Point2`/`Point3` aliases directly and accepts slice-based formation inputs.
- Python bindings published as `vvcm-rs` / `vvcm_rs` with NumPy-first typed package metadata.
- C++17 wrapper headers and a C ABI for native consumers using row-major matrix views.
- WebAssembly bindings published to npm as `@morningfrog/vvcm-rs` and the unscoped mirror `vvcm-rs`, using `Float32Array` inputs and Rust-like solution objects.
- Stable-solution search with taut-cable enumeration, candidate solving, and stable-branch filtering.
- Velocity-driven and manual simulation wrappers.
- Distribution through crates.io, npm, PyPI, GitHub Releases, and vcpkg overlays.

## Module Overview

- `fk`: forward kinematics engine state and stable-solution entry point.
- `simulation`: velocity-driven simulation wrapper.
- `manual_simulation`: wrapper for querying a new stable solution from an externally provided robot formation.
- `types`: public `nalgebra` aliases, conversion helpers, and FK solution collections.
- `ffi`: C++ wrapper and C ABI implementation behind the native headers.
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

Prebuilt PyPI wheels are published for CPython 3.10 through 3.14 on Windows x64, Linux x64, and macOS arm64. The Python API depends on NumPy and uses C-contiguous `float32` arrays for formation, sheet, velocity, and result buffers. Python 3.9 and other platforms may fall back to building from the source distribution, which requires a local Rust toolchain and Python build tooling.

### C++ and C

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

## Usage

The language-specific snippets below assume installation is already complete. Choose the section that matches your project.

The sample outputs below round floating-point values to three decimals; small platform differences are normal.

### Rust Usage

After adding `vvcm-rs` from crates.io, the Rust API looks like this:

```rust
use vvcm_rs::{Point2, VvcmError, VvcmFk};

fn main() -> Result<(), VvcmError> {
    // Robot formation: each Point2 is a robot node position on the
    // world-coordinate XY plane, in millimeters.
    let formation = vec![
        Point2::new(213.7, 122.7),
        Point2::new(804.6, 37.2),
        Point2::new(904.0, 550.0),
        Point2::new(439.3, 715.9),
    ];

    // Unfolded sheet: each Point2 is a vertex in the sheet's local
    // coordinate frame, in millimeters.
    let sheet = vec![
        Point2::new(-316.1, -421.9),
        Point2::new(803.4, -384.1),
        Point2::new(746.1, 712.8),
        Point2::new(-367.3, 664.2),
    ];

    // Create the FK solver with a 1000 mm hold height.
    // The robot count is inferred from sheet.len().
    let mut fk = VvcmFk::new(1000.0, sheet)?;

    // Ask the solver to enumerate every candidate equilibrium for this formation.
    let solutions = fk.update_stable_solutions(&formation)?;

    // Report the total branch count and the subset that is stable.
    println!("all solutions: {}", solutions.all_count());
    println!("stable solutions: {}", solutions.stable_count());

    // Print each stable branch. lambda_values[i] belongs to taut_cables[i]
    // on the same solution.
    for (index, solution) in solutions.stable().enumerate() {
        let lambda_values = solution
            .lambda_values
            .iter()
            .map(|value| format!("{value:.3}"))
            .collect::<Vec<_>>()
            .join(", ");
        println!(
            "#{index}: Po=({:.3}, {:.3}, {:.3}), Vo=({:.3}, {:.3}), taut={:?}, lambda=[{}]",
            solution.po.x,
            solution.po.y,
            solution.po.z,
            solution.vo.x,
            solution.vo.y,
            solution.taut_cables,
            lambda_values,
        );
    }

    Ok(())
}
```

Expected output:

```text
all solutions: 3
stable solutions: 2
#0: Po=(568.841, 324.728, 336.736), Vo=(238.633, 125.028), taut=[0, 1, 2], lambda=[0.480, 0.039, 0.481]
#1: Po=(557.919, 341.232, 337.247), Vo=(208.794, 152.532), taut=[0, 2, 3], lambda=[0.493, 0.495, 0.012]
```

### Python Usage

After installing `vvcm-rs` from PyPI, import it as `vvcm_rs` and pass C-contiguous NumPy `float32` arrays. Formation and sheet arrays use shape `(n, 2)`, while object reference points use shape `(3,)`.

```python
import numpy as np
from vvcm_rs import VvcmFk

# Robot formation: each row is a robot node position on the world-coordinate
# XY plane, in millimeters.
# Keep dtype=np.float32 and a C-contiguous shape of (n, 2) for zero-copy binding input.
formation = np.array(
    [
        [213.7, 122.7],
        [804.6, 37.2],
        [904.0, 550.0],
        [439.3, 715.9],
    ],
    dtype=np.float32,
)
# Unfolded sheet: each row is a vertex in the sheet's local coordinate
# frame, in millimeters.
# The row order must match the robot/cable order used by formation.
sheet = np.array(
    [
        [-316.1, -421.9],
        [803.4, -384.1],
        [746.1, 712.8],
        [-367.3, 664.2],
    ],
    dtype=np.float32,
)

# Create the solver with a 1000 mm hold height.
# The robot count is inferred from sheet.shape[0].
fk = VvcmFk(1000.0, sheet)

# Solve all candidate equilibria for the current formation.
solutions = fk.update_stable_solutions(formation)

# Report the total branch count and the subset that is stable.
print(f"all solutions: {solutions.all_count()}")
print(f"stable solutions: {solutions.stable_count()}")

# Print each stable branch.
# po, vo, taut_cables, and lambda_values are per-solution arrays.
for index, solution in enumerate(solutions.stable()):
    po = solution.po
    vo = solution.vo
    # lambda_values[i] belongs to taut_cables[i] on the same solution.
    lambda_values = ", ".join(f"{value:.3f}" for value in solution.lambda_values)
    print(
        f"#{index}: Po=({po[0]:.3f}, {po[1]:.3f}, {po[2]:.3f}), "
        f"Vo=({vo[0]:.3f}, {vo[1]:.3f}), taut={solution.taut_cables.tolist()}, "
        f"lambda=[{lambda_values}]"
    )
```

Expected output:

```text
all solutions: 3
stable solutions: 2
#0: Po=(568.841, 324.728, 336.736), Vo=(238.633, 125.028), taut=[0, 1, 2], lambda=[0.480, 0.039, 0.481]
#1: Po=(557.919, 341.232, 337.247), Vo=(208.794, 152.532), taut=[0, 2, 3], lambda=[0.493, 0.495, 0.012]
```

### C++ Usage

After installing the vcpkg package or a release archive, consume the installed CMake package and headers directly. The package exports the C++17 RAII wrapper in `vvcm_rs.hpp` and the raw C ABI in `vvcm_rs.h`.

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

    // Robot formation in row-major [x0, y0, x1, y1, ...] order, in millimeters.
    const std::vector<float> formation = {
        213.7f, 122.7f,
        804.6f, 37.2f,
        904.0f, 550.0f,
        439.3f, 715.9f,
    };

    // Unfolded sheet vertices in the sheet's local frame, using the same
    // row-major layout.
    const std::vector<float> sheet = {
        -316.1f, -421.9f,
        803.4f, -384.1f,
        746.1f, 712.8f,
        -367.3f, 664.2f,
    };

    // The RAII wrapper owns the raw C handle and releases it automatically.
    VvcmFk fk(1000.0f, matrix_view(sheet));

    // matrix_view borrows the vectors and exposes them as VvcmRsMat2f without copying.
    FkSolutions solutions = fk.update_stable_solutions(matrix_view(formation));

    // Report the total branch count and the subset that is stable.
    std::cout << "all solutions: " << solutions.all_count() << "\n";
    std::cout << "stable solutions: " << solutions.stable_count() << "\n";

    // stable() returns owning FkSolution objects; each one carries its
    // taut cables and lambda values.
    std::cout << std::fixed << std::setprecision(3);
    const std::vector<FkSolution> stable = solutions.stable();
    for (std::size_t index = 0; index < stable.size(); ++index) {
        const FkSolution &solution = stable[index];
        const Vec3f &po = solution.po;
        const Vec2f &vo = solution.vo;
        const std::vector<std::size_t> &taut_cables = solution.taut_cables;
        const std::vector<float> &lambda_values = solution.lambda_values;

        std::cout << "#" << index << ": Po=("
                  << po.x << ", " << po.y << ", " << po.z << "), Vo=("
                  << vo.x << ", " << vo.y << "), taut=[";
        for (std::size_t taut_index = 0; taut_index < taut_cables.size(); ++taut_index) {
            if (taut_index > 0) {
                std::cout << ", ";
            }
            std::cout << taut_cables[taut_index];
        }
        std::cout << "], lambda=[";
        for (std::size_t lambda_index = 0; lambda_index < lambda_values.size(); ++lambda_index) {
            if (lambda_index > 0) {
                std::cout << ", ";
            }
            std::cout << lambda_values[lambda_index];
        }
        std::cout << "]\n";
    }
}
```

Expected output:

```text
all solutions: 3
stable solutions: 2
#0: Po=(568.841, 324.728, 336.736), Vo=(238.633, 125.028), taut=[0, 1, 2], lambda=[0.480, 0.039, 0.481]
#1: Po=(557.919, 341.232, 337.247), Vo=(208.794, 152.532), taut=[0, 2, 3], lambda=[0.493, 0.495, 0.012]
```

### C Usage

The raw C ABI uses explicit handles and matrix views. A `VvcmRsMat2f` is a borrowed row-major view over `[x0, y0, x1, y1, ...]` data; the library never takes ownership of the input arrays.

```c
#include <vvcm_rs.h>

#include <stdio.h>
#include <stdlib.h>

static int check(VvcmRsErrorCode code, const char *context) {
    if (code == VVCM_RS_ERROR_OK) {
        return 1;
    }

    fprintf(stderr, "%s failed: %s\n", context, vvcm_rs_last_error_message());
    return 0;
}

int main(void) {
    // Robot formation in row-major [x0, y0, x1, y1, ...] order, in millimeters.
    const float formation_data[] = {
        213.7f, 122.7f,
        804.6f, 37.2f,
        904.0f, 550.0f,
        439.3f, 715.9f,
    };

    // Unfolded sheet vertices in the sheet's local frame, using the same
    // row-major layout.
    const float sheet_data[] = {
        -316.1f, -421.9f,
        803.4f, -384.1f,
        746.1f, 712.8f,
        -367.3f, 664.2f,
    };

    // rows is the point count; stride is the number of floats between adjacent rows.
    const VvcmRsMat2f formation = {formation_data, 4, 2};
    const VvcmRsMat2f sheet = {sheet_data, 4, 2};

    VvcmRsFk *fk = NULL;
    int status = 1;

    // Create the FK solver with a 1000 mm hold height. Release it with vvcm_rs_fk_free.
    if (!check(vvcm_rs_fk_new(1000.0f, sheet, &fk), "vvcm_rs_fk_new")) {
        goto cleanup;
    }

    // Solve all candidate equilibria for the current formation.
    if (!check(vvcm_rs_fk_update_stable_solutions(fk, formation), "vvcm_rs_fk_update_stable_solutions")) {
        goto cleanup;
    }

    // Query aggregate counts before reading individual solution objects.
    size_t all_count = 0;
    size_t stable_count = 0;
    if (!check(vvcm_rs_fk_solution_count(fk, &all_count), "vvcm_rs_fk_solution_count")) {
        goto cleanup;
    }
    if (!check(vvcm_rs_fk_stable_solution_count(fk, &stable_count), "vvcm_rs_fk_stable_solution_count")) {
        goto cleanup;
    }

    printf("all solutions: %zu\n", all_count);
    printf("stable solutions: %zu\n", stable_count);

    // Read each stable solution. The fixed-size fields are in VvcmRsFkSolution.
    size_t stable_index = 0;
    for (size_t index = 0; index < all_count; ++index) {
        VvcmRsFkSolution solution = {0};
        if (!check(vvcm_rs_fk_solution_at(fk, index, &solution), "vvcm_rs_fk_solution_at")) {
            goto cleanup;
        }
        if (!solution.stable) {
            continue;
        }

        // Pass NULL to query the required variable-array length, then pass an
        // allocated buffer.
        size_t taut_count = 0;
        if (!check(vvcm_rs_fk_solution_taut_cables(fk, index, NULL, &taut_count), "vvcm_rs_fk_solution_taut_cables")) {
            goto cleanup;
        }
        size_t *taut_cables = taut_count == 0 ? NULL : (size_t *)malloc(taut_count * sizeof(*taut_cables));
        if (taut_count != 0 && taut_cables == NULL) {
            fprintf(stderr, "failed to allocate taut cable buffer\n");
            goto cleanup;
        }
        size_t taut_capacity = taut_count;
        if (!check(vvcm_rs_fk_solution_taut_cables(fk, index, taut_cables, &taut_capacity), "vvcm_rs_fk_solution_taut_cables")) {
            free(taut_cables);
            goto cleanup;
        }

        // lambda_values[i] belongs to taut_cables[i] on the same solution.
        size_t lambda_count = 0;
        if (!check(vvcm_rs_fk_solution_lambda_values(fk, index, NULL, &lambda_count), "vvcm_rs_fk_solution_lambda_values")) {
            free(taut_cables);
            goto cleanup;
        }
        float *lambda_values = lambda_count == 0 ? NULL : (float *)malloc(lambda_count * sizeof(*lambda_values));
        if (lambda_count != 0 && lambda_values == NULL) {
            fprintf(stderr, "failed to allocate lambda value buffer\n");
            free(taut_cables);
            goto cleanup;
        }
        size_t lambda_capacity = lambda_count;
        if (!check(vvcm_rs_fk_solution_lambda_values(fk, index, lambda_values, &lambda_capacity), "vvcm_rs_fk_solution_lambda_values")) {
            free(lambda_values);
            free(taut_cables);
            goto cleanup;
        }

        printf(
            "#%zu: Po=(%.3f, %.3f, %.3f), Vo=(%.3f, %.3f), taut=[",
            stable_index++,
            solution.po.x,
            solution.po.y,
            solution.po.z,
            solution.vo.x,
            solution.vo.y);
        for (size_t taut_index = 0; taut_index < taut_capacity; ++taut_index) {
            printf("%s%zu", taut_index == 0 ? "" : ", ", taut_cables[taut_index]);
        }
        printf("], lambda=[");
        for (size_t lambda_index = 0; lambda_index < lambda_capacity; ++lambda_index) {
            printf("%s%.3f", lambda_index == 0 ? "" : ", ", lambda_values[lambda_index]);
        }
        printf("]\n");

        free(lambda_values);
        free(taut_cables);
    }

    status = 0;

cleanup:
    vvcm_rs_fk_free(fk);
    return status;
}
```

Expected output:

```text
all solutions: 3
stable solutions: 2
#0: Po=(568.841, 324.728, 336.736), Vo=(238.633, 125.028), taut=[0, 1, 2], lambda=[0.480, 0.039, 0.481]
#1: Po=(557.919, 341.232, 337.247), Vo=(208.794, 152.532), taut=[0, 2, 3], lambda=[0.493, 0.495, 0.012]
```

### JavaScript and TypeScript Usage

After installing `@morningfrog/vvcm-rs` or `vvcm-rs` from npm, import the WebAssembly module and pass row-major `Float32Array` buffers. Formation and sheet arrays are laid out as `[x0, y0, x1, y1, ...]`.

```ts
import { VvcmFk } from "@morningfrog/vvcm-rs";

// Robot formation in row-major [x0, y0, x1, y1, ...] order, in millimeters.
const formation = new Float32Array([
  213.7, 122.7,
  804.6, 37.2,
  904.0, 550.0,
  439.3, 715.9,
]);

// Unfolded sheet vertices in the sheet's local frame, using the same row-major layout.
const sheet = new Float32Array([
  -316.1, -421.9,
  803.4, -384.1,
  746.1, 712.8,
  -367.3, 664.2,
]);

// Create the FK solver with a 1000 mm hold height.
// The robot count is inferred from sheet.length / 2.
const fk = new VvcmFk(1000, sheet);

// Solve all candidate equilibria. Each returned solution object owns its
// own tautCables and lambdaValues arrays.
const solutions = fk.updateStableSolutions(formation);

// Report the total branch count and the subset that is stable.
console.log(`all solutions: ${solutions.allCount}`);
console.log(`stable solutions: ${solutions.stableCount}`);

// Print stable branches only. lambdaValues[i] belongs to tautCables[i]
// on the same solution.
for (const [index, solution] of solutions.solutions.entries()) {
  if (!solution.stable) {
    continue;
  }

  const lambdaValues = solution.lambdaValues.map((value) => value.toFixed(3)).join(", ");
  console.log(
    `#${index}: Po=(${solution.po.x.toFixed(3)}, ${solution.po.y.toFixed(3)}, ${solution.po.z.toFixed(3)}), ` +
      `Vo=(${solution.vo.x.toFixed(3)}, ${solution.vo.y.toFixed(3)}), taut=${JSON.stringify(solution.tautCables)}, ` +
      `lambda=[${lambdaValues}]`,
  );
}

// Free the underlying WASM allocation when the solver is no longer needed.
fk.free();
```

Expected output:

```text
all solutions: 3
stable solutions: 2
#0: Po=(568.841, 324.728, 336.736), Vo=(238.633, 125.028), taut=[0,1,2], lambda=[0.480, 0.039, 0.481]
#1: Po=(557.919, 341.232, 337.247), Vo=(208.794, 152.532), taut=[0,2,3], lambda=[0.493, 0.495, 0.012]
```

Across bindings, lambda values are taut-only: each `lambda_values` or `lambdaValues` entry corresponds to the matching taut cable entry on the same solution object. Slack cables are omitted instead of represented by zero-valued placeholders.

Length units are not encoded in the API. Use one consistent unit for formation coordinates, sheet coordinates, and hold height; `VvcmFk` normalizes coordinates internally for numerical stability and maps returned object positions and virtual object points back to the original coordinate frames.

## Error Handling

Forward-kinematics and simulation solves report failures through each language's normal error channel. Error messages are intended for human diagnostics; branch on Rust enum variants, Python exception classes, C++ and C error codes, or JavaScript error codes when program logic needs to distinguish failure modes.

The snippets below show one simple handling pattern for each language: catch the failure, print the message, and branch on the typed error when you need a specific recovery path.

### Rust

```rust
match fk.update_stable_solutions(&formation) {
    Ok(solutions) => {
        // The successful result has the same Rust-like per-solution shape
        // as the normal example above.
        println!("stable solutions: {}", solutions.stable_count());
    }
    Err(vvcm_rs::VvcmError::InfeasibleFormation) => {
        // Branch on a specific enum variant when program logic can recover
        // from that case.
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
    # Catch a specific subclass when this failure has a dedicated recovery path.
    print(f"formation is infeasible: {error}")
except VvcmError as error:
    # VvcmError is the common base class for all binding-level solve failures.
    print(f"vvcm-rs solve failed: {error}")
else:
    # The success value is an FkSolutions object with per-solution FkSolution entries.
    print(f"stable solutions: {solutions.stable_count()}")
```

The main solve errors in Python are:

- `DimensionMismatchError` for input size mismatches during construction or solve setup.
- `InfeasibleFormationError` when the robot formation cannot be realized by the sheet geometry.
- `NoSolutionError` when no candidate branch can be constructed.
- `NoStableSolutionError` when candidate branches exist but none are stable.
- `VvcmError` remains the common base class, so `except VvcmError as error` still catches any of them and `except InfeasibleFormationError as error` can catch one case specifically.

### C++

```cpp
try {
    // The C++ wrapper throws vvcm_rs::Error instead of returning VvcmRsErrorCode.
    auto solutions = fk.update_stable_solutions(vvcm_rs::matrix_view(formation));
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

### C

```c
VvcmRsMat2f formation = {formation_data, formation_rows, 2};

// Run the solve and inspect the typed C error code.
// last_error_message gives the detailed thread-local message.
VvcmRsErrorCode code = vvcm_rs_fk_update_stable_solutions(fk, formation);
if (code != VVCM_RS_ERROR_OK) {
    fprintf(stderr, "vvcm-rs failed: %s\n", vvcm_rs_last_error_message());
    if (code == VVCM_RS_ERROR_INFEASIBLE_FORMATION) {
        fprintf(stderr, "formation is infeasible\n");
    }
} else {
    // After a successful solve, query aggregate counts directly from the FK handle.
    size_t solution_count = 0;
    code = vvcm_rs_fk_solution_count(fk, &solution_count);
    if (code == VVCM_RS_ERROR_OK) {
        fprintf(stdout, "all solutions: %zu\n", solution_count);
    }

    // solution_at returns the fixed-size fields; variable arrays are copied
    // through separate functions.
    VvcmRsFkSolution solution = {0};
    code = vvcm_rs_fk_solution_at(fk, 0, &solution);
    if (code == VVCM_RS_ERROR_OK) {
        // Pass NULL to query the required taut-cable count before allocating a buffer.
        size_t taut_count = 0;
        code = vvcm_rs_fk_solution_taut_cables(fk, 0, NULL, &taut_count);
        if (code == VVCM_RS_ERROR_OK) {
            fprintf(stdout, "first solution taut cables: %zu\n", taut_count);
        }
    }
}
```

The main solve errors in C are:

- `VVCM_RS_ERROR_DIMENSION_MISMATCH` for input size mismatches during construction or solve setup.
- `VVCM_RS_ERROR_INFEASIBLE_FORMATION` when the robot formation cannot be realized by the sheet geometry.
- `VVCM_RS_ERROR_NO_SOLUTION` when no candidate branch can be constructed.
- `VVCM_RS_ERROR_NO_STABLE_SOLUTION` when candidate branches exist but none are stable.
- `vvcm_rs_last_error_message()` returns the human-readable message for the most recent failure on the current thread, while `vvcm_rs_error_message(code)` returns the generic message for a given code.

### JavaScript and TypeScript

```ts
import { VvcmFk, type VvcmError } from "@morningfrog/vvcm-rs";

function isVvcmError(error: unknown): error is VvcmError {
  // The WASM wrapper throws Error objects with a stable vvcm-rs code field.
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
- `INVALID_ARGUMENT` when a JavaScript typed array has an invalid length for the expected point, formation, sheet, or velocity shape.
