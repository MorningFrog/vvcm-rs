# Developer Guide

## Commit Message Convention

This repository requires all commit messages to follow the **Conventional Commits** specification.

The commit message format is:

```text
<type>(<scope>): <description>
```

Where:

* `type`: the type of change, required
* `scope`: the affected area, optional
* `description`: a short description of the change, required

Examples:

```text
feat(algorithm): add VVCM forward kinematics implementation
fix(python): correct example script for Python bindings
docs: update README
refactor: simplify VVCM stability filtering logic
```

Common commit types:

| Type       | Description                                           |
| ---------- | ----------------------------------------------------- |
| `feat`     | A new feature                                         |
| `fix`      | A bug fix                                             |
| `docs`     | Documentation changes                                 |
| `style`    | Code style changes that do not affect logic           |
| `refactor` | Code changes that neither fix a bug nor add a feature |
| `perf`     | Performance improvements                              |
| `test`     | Adding or updating tests                              |
| `build`    | Build system or dependency changes                    |
| `ci`       | CI/CD configuration changes                           |
| `chore`    | Other maintenance changes                             |
| `revert`   | Reverting a previous commit                           |

For breaking changes, mark them clearly:

```text
feat(api)!: remove deprecated method
```

## Local Development

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies

```bash
cargo build
```

For C++ export work and tests, install a C++17 compiler. On Windows, MSVC is recommended; on Linux and macOS, GCC or Clang is sufficient.

### 3. Install Python Binding Tools

Use a virtual environment for Python binding development and tests:

```bash
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip maturin pytest numpy
```

### 4. Install Packages From Source

Use these commands when you need to test local source changes before a published release exists.

#### Rust

`vvcm-rs` is a library crate, so Rust projects normally consume a local checkout as a path dependency. From another Rust project, run:

```bash
cargo add vvcm-rs --path <path-to-vvcm-rs>
```

or edit `Cargo.toml` directly:

```toml
[dependencies]
vvcm-rs = { path = "<path-to-vvcm-rs>" }
```

Build the local Rust library with the standard Cargo profiles:

```bash
cargo build --lib
cargo build --lib --release
```

The debug build writes artifacts under `target/debug`; the release build writes optimized artifacts under `target/release`.

#### C and C++

Use the repo-local vcpkg overlay port for C/C++ source installs. It builds the native Rust library with Cargo and installs the C header, C++17 wrapper, and CMake package metadata:

```bash
vcpkg install vvcm-rs --overlay-ports=<path-to-vvcm-rs>/vcpkg/ports --triplet <triplet>
```

With the default vcpkg triplets, the port builds both Cargo profiles: `cargo build --lib --locked` for debug and `cargo build --lib --locked --release` for release. The installed debug library is placed under the vcpkg `debug/lib` directory, and the release library is placed under `lib`.

For debug-only or release-only installs, use a custom triplet based on your platform triplet and set `VCPKG_BUILD_TYPE`:

```cmake
set(VCPKG_BUILD_TYPE debug)
```

or:

```cmake
set(VCPKG_BUILD_TYPE release)
```

Then pass that triplet to `vcpkg install`:

```bash
vcpkg install vvcm-rs --overlay-ports=<path-to-vvcm-rs>/vcpkg/ports --overlay-triplets=<path-to-triplets> --triplet <debug-or-release-triplet>
```

After installation, consume the package from CMake:

```cmake
find_package(vvcm-rs CONFIG REQUIRED)
target_link_libraries(app PRIVATE vvcm_rs::vvcm_rs)
```

#### Python

Install the Python package from the source tree into the active virtual environment with maturin. The debug install is useful while developing because `maturin develop` uses Cargo's debug profile by default:

```bash
maturin develop
```

Use a release build when you need optimized Python extension performance:

```bash
maturin develop --release
```

To build a distributable wheel from source, use the release build:

```bash
maturin build --release --locked --out dist
python -m pip install --force-reinstall dist/<wheel-file>.whl
```

### 5. Start the Example

```bash
cargo run --example basic_fk
```

## Repository Structure

The repository centers on a single Rust crate with Python bindings and C/C++ exports:

* `src/` contains the core Rust implementation, including the FK solver, simulation wrappers, FFI layer, Python bindings, math helpers, error types, and public domain types.
* `include/` contains the C header and C++ wrapper for native consumers.
* `python/` contains the published Python package, module entry point, and type information shipped with the wheel.
* `examples/` contains runnable Rust examples and timing demos.
* `tests/` contains Rust smoke tests, the C++ export smoke test, and Python binding tests.
* `vcpkg/ports/vvcm-rs/` contains the repo-local vcpkg overlay port and its packaging metadata.
* `vcpkg/prebuilt-ports/vvcm-rs/` contains the template used to generate the prebuilt vcpkg overlay archive published with releases.
* `scripts/` contains packaging helpers for native release zips and the generated prebuilt vcpkg overlay archive.
* `.github/workflows/` contains CI and release automation, including the published release workflow.
* Root metadata files such as `Cargo.toml`, `pyproject.toml`, `README.md`, `CHANGELOG.md`, `TODO.md`, and `LICENSE` describe the package, docs, and release history.

## Code Style

Before committing code, make sure formatting and static checks pass:

```bash
cargo fmt
cargo clippy
```

When changing Python bindings, also confirm the editable extension build succeeds:

```bash
maturin develop
python -m pytest tests/python
```

When changing C/C++ export headers or FFI, also confirm the C++ smoke test succeeds:

```bash
cargo test --test cpp_export_smoke
```

Code requirements:

* Use clear and meaningful names
* Avoid unnecessary abbreviations
* Add Rustdoc comments for public API items
* Avoid unrelated changes in the same commit
* Remove unused code and debug logs
* Keep functions focused on a single responsibility
* Add comments for complex or non-obvious logic

## Testing Requirements

Before submitting a Pull Request, make sure the relevant tests pass:

```bash
cargo test
```

Python binding changes should also pass:

```bash
maturin develop
python -m pytest tests/python
```

C/C++ export changes should also pass:

```bash
cargo test --test cpp_export_smoke
```

When adding a feature or fixing a bug, add or update tests whenever possible.

Tests should cover:

* Successful flows
* Error flows
* Edge cases
* Important business logic

## Pull Request Guidelines

Before submitting a Pull Request, confirm that:

* The branch is up to date with the latest target branch
* Commit messages follow the Conventional Commits specification
* Local tests have passed
* Code has been formatted
* No unrelated files or debug code are included
* The PR description clearly explains the changes

PR titles should also follow the Conventional Commits format when possible:

```text
feat(algorithm): add parameter validation to VVCM forward kinematics
fix(algorithm): correct stability filtering logic for edge cases
docs: update deployment guide
```

## Release Process

Releases are published only by authors or collaborators through the GitHub Actions workflow named "Release". Do not publish packages, create tags, or create GitHub releases manually from a local machine. The workflow itself is the release path, and it is triggered by hand when a release is ready.

The release workflow validates the Rust checks, the Python sdist, the CPython 3.10 through 3.14 wheel matrix, the source overlay, the native package matrix for Windows x64, Linux x64, and macOS arm64, and the prebuilt vcpkg overlay artifacts, then creates the Git tag and GitHub release before publishing to crates.io and PyPI.

## Issue Guidelines

When creating an issue, include as much useful context as possible.

Recommended issue template:

```markdown
## Description

Describe the problem.

## Steps to Reproduce

1. 
2. 
3. 

## Expected Result

Describe what you expected to happen.

## Actual Result

Describe what actually happened.

## Environment

- OS:
- Runtime:
- Version:

## Additional Context

Logs, screenshots, or other relevant information.
```
