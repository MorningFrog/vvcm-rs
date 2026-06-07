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

### 3. Start the Example

```bash
cargo run --example basic_fk
```

## Code Style

Before committing code, make sure formatting and static checks pass:

```bash
cargo fmt
cargo clippy
```

Code requirements:

* Use clear and meaningful names
* Avoid unnecessary abbreviations
* Avoid unrelated changes in the same commit
* Remove unused code and debug logs
* Keep functions focused on a single responsibility
* Add comments for complex or non-obvious logic

## Testing Requirements

Before submitting a Pull Request, make sure the relevant tests pass:

```bash
cargo test
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

The release process requires a manual run of the Github Actions workflow named "Release". This workflow will:

1. Create a new Git tag with the version number from `Cargo.toml`.
2. Create a new release on GitHub with the version number in `Cargo.toml` and changelog in `CHANGELOG.md`.
3. Publish the crate to crates.io.
4. Publish the python package to PyPI.

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
