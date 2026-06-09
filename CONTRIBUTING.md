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

For C++ export work and tests, install a C++17 compiler. On Windows, MSVC is
recommended; on Linux and macOS, GCC or Clang is sufficient.

### 3. Install Python Binding Tools

Use a virtual environment for Python binding development and tests:

```bash
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip maturin pytest numpy
```

### 4. Start the Example

```bash
cargo run --example basic_fk
```

## Repository Structure

The repository centers on a single Rust crate with Python bindings and C/C++
exports:

* `src/` contains the core Rust implementation, including the FK solver,
  simulation wrappers, FFI layer, Python bindings, math helpers, error types,
  and public domain types.
* `include/` contains the C header and C++ wrapper for native consumers.
* `python/` contains the published Python package, module entry point, and type
  information shipped with the wheel.
* `examples/` contains runnable Rust examples and timing demos.
* `tests/` contains Rust smoke tests, the C++ export smoke test, and Python
  binding tests.
* `vcpkg/ports/vvcm-rs/` contains the repo-local vcpkg overlay port and its
  packaging metadata.
* `.github/workflows/` contains CI and release automation, including the
  published release workflow.
* Root metadata files such as `Cargo.toml`, `pyproject.toml`, `README.md`,
  `CHANGELOG.md`, `TODO.md`, and `LICENSE` describe the package, docs, and
  release history.

## Code Style

Before committing code, make sure formatting and static checks pass:

```bash
cargo fmt
cargo clippy
```

When changing Python bindings, also confirm the editable extension build
succeeds:

```bash
maturin develop
python -m pytest tests/python
```

When changing C/C++ export headers or FFI, also confirm the C++ smoke test
succeeds:

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

Releases are published only by authors or collaborators through the GitHub
Actions workflow named "Release". Do not publish packages, create tags, or
create GitHub releases manually from a local machine. The workflow itself is
the release path, and it is triggered by hand when a release is ready.

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
