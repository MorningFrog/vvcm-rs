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

Before releasing, confirm that:

* All planned changes have been merged
* All tests have passed
* The version number has been updated
* The changelog has been updated
* CI/CD checks have passed

The release workflow publishes to crates.io and PyPI, and creates a GitHub
release asset that contains a source tree with the vcpkg overlay port. Configure
the crates.io secret before running it, and add a PyPI Trusted Publisher for the
GitHub Actions workflow plus the `pypi` environment in your PyPI project
settings:

| Secret                 | Purpose                         |
| ---------------------- | ------------------------------- |
| `CARGO_REGISTRY_TOKEN` | crates.io publishing token      |

Version numbers should follow Semantic Versioning:

```text
MAJOR.MINOR.PATCH
```

Examples:

```text
1.0.0
1.1.0
1.1.1
```

Version meaning:

| Version | Description                      |
| ------- | -------------------------------- |
| `MAJOR` | Breaking changes                 |
| `MINOR` | Backward-compatible new features |
| `PATCH` | Backward-compatible bug fixes    |

The release process requires a manual run of the GitHub Actions workflow named "Release". It has a `dry-run` input that defaults to `true`. Keep `dry-run` enabled to validate the release without creating a tag, GitHub release, crates.io upload, or PyPI upload. Set `dry-run` to `false` only for the final publishing run.

The workflow will:

1. Verify that `Cargo.toml`, `pyproject.toml`, and `vcpkg/ports/vvcm-rs/vcpkg.json` use the same version.
2. Check that the crates.io secret is configured. The workflow validates the token with the authenticated `/api/v1/me` endpoint.
3. When `dry-run` is `true`, validate the PyPI Trusted Publisher configuration by minting a short-lived OIDC-backed token without uploading anything.
4. Run Rust formatting, clippy, tests, and a crates.io dry run.
5. Build and check the Python source distribution and wheel.
6. Build the repo-local vcpkg overlay port.
7. Generate and print the GitHub release notes from the top `CHANGELOG.md` unreleased section.
8. When `dry-run` is `false`, publish the Rust crate to crates.io.
9. When `dry-run` is `false`, publish the Python distributions to PyPI using `pypa/gh-action-pypi-publish`.
10. When `dry-run` is `false`, create a new Git tag named `v<version>` from the version in `Cargo.toml`.
11. When `dry-run` is `false`, create a GitHub release and attach the Python distributions plus a vcpkg-ready source archive.

The vcpkg overlay port in `vcpkg/ports/vvcm-rs` is ready for local or release
source-archive use. Publishing it to the official vcpkg registry requires a
separate upstream PR after a tagged GitHub release is available.

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
