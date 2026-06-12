#!/usr/bin/env python3
"""Check or update vvcm-rs package versions across release manifests."""

from __future__ import annotations

import argparse
import json
import re
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Callable


PACKAGE_NAME = "vvcm-rs"
REPO_ROOT = Path(__file__).resolve().parents[1]
SEMVER_RE = re.compile(
    r"^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)"
    r"(?:-(?:0|[1-9]\d*|[0-9A-Za-z-]*[A-Za-z-][0-9A-Za-z-]*)"
    r"(?:\.(?:0|[1-9]\d*|[0-9A-Za-z-]*[A-Za-z-][0-9A-Za-z-]*))*)?$"
)


@dataclass(frozen=True)
class VersionFile:
    path: Path
    read: Callable[[Path], str]
    write: Callable[[Path, str], None]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check or update vvcm-rs package versions across release manifests."
    )
    parser.add_argument(
        "version",
        nargs="?",
        help="Version to write, such as 1.2.3. Omit to check that versions match.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Only verify that all version files are synchronized.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show the files that would be updated without writing them.",
    )
    parser.add_argument(
        "--print-version",
        action="store_true",
        help="Print only the synchronized version, useful for automation.",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=REPO_ROOT,
        help="Repository root. Defaults to the parent of this script directory.",
    )
    args = parser.parse_args()
    if args.version and args.check:
        parser.error(
            "pass either VERSION to update files or --check to verify them, not both"
        )
    if args.dry_run and not args.version:
        parser.error("--dry-run requires VERSION")
    return args


def validate_version(version: str) -> None:
    if not SEMVER_RE.fullmatch(version):
        raise SystemExit(
            f"invalid version {version!r}; use portable SemVer such as 1.2.3 or 1.2.3-rc.1"
        )


def require_string(value: object, path: Path, field: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be a string in {path}")
    return value


def read_toml_version(path: Path, section: str) -> str:
    data = tomllib.loads(path.read_text(encoding="utf-8"))
    value = data[section]["version"]
    return require_string(value, path, f"{section}.version")


def write_toml_version(path: Path, section: str, version: str) -> None:
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    in_section = False
    found_section = False

    for index, line in enumerate(lines):
        header = line.strip()
        if header.startswith("[") and header.endswith("]") and not header.startswith("[["):
            in_section = header == f"[{section}]"
            found_section = found_section or in_section
            continue

        if not in_section:
            continue

        line_ending = "\r\n" if line.endswith("\r\n") else "\n" if line.endswith("\n") else ""
        body = line[: -len(line_ending)] if line_ending else line
        match = re.fullmatch(r'(\s*version\s*=\s*")([^"]*)(".*)', body)
        if match:
            lines[index] = f"{match.group(1)}{version}{match.group(3)}{line_ending}"
            path.write_text("".join(lines), encoding="utf-8")
            return

    if found_section:
        raise ValueError(f"missing version in [{section}] section of {path}")
    raise ValueError(f"missing [{section}] section in {path}")


def read_json_version(path: Path, field: str) -> str:
    data = json.loads(path.read_text(encoding="utf-8"))
    value = data[field]
    return require_string(value, path, field)


def write_json_version(path: Path, field: str, version: str) -> None:
    data = json.loads(path.read_text(encoding="utf-8"))
    if field not in data:
        raise ValueError(f"missing {field} in {path}")
    data[field] = version
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def read_cargo_lock_version(path: Path) -> str:
    data = tomllib.loads(path.read_text(encoding="utf-8"))
    packages = data.get("package", [])
    candidates = [
        package
        for package in packages
        if package.get("name") == PACKAGE_NAME and "source" not in package
    ]
    if not candidates:
        candidates = [package for package in packages if package.get("name") == PACKAGE_NAME]
    if len(candidates) != 1:
        raise ValueError(f"expected one {PACKAGE_NAME} package entry in {path}")
    return require_string(candidates[0]["version"], path, f"{PACKAGE_NAME}.version")


def write_cargo_lock_version(path: Path, version: str) -> None:
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    in_package = False
    in_target_package = False

    for index, line in enumerate(lines):
        body = line.rstrip("\r\n")
        if body == "[[package]]":
            in_package = True
            in_target_package = False
            continue

        if not in_package:
            continue

        if re.fullmatch(rf'\s*name\s*=\s*"{re.escape(PACKAGE_NAME)}"\s*', body):
            in_target_package = True
            continue

        if in_target_package:
            line_ending = "\r\n" if line.endswith("\r\n") else "\n" if line.endswith("\n") else ""
            match = re.fullmatch(r'(\s*version\s*=\s*")([^"]*)(".*)', body)
            if match:
                lines[index] = f"{match.group(1)}{version}{match.group(3)}{line_ending}"
                path.write_text("".join(lines), encoding="utf-8")
                return

    raise ValueError(f"missing {PACKAGE_NAME} package version in {path}")


def version_files() -> list[VersionFile]:
    return [
        VersionFile(
            Path("Cargo.toml"),
            lambda path: read_toml_version(path, "package"),
            lambda path, version: write_toml_version(path, "package", version),
        ),
        VersionFile(Path("Cargo.lock"), read_cargo_lock_version, write_cargo_lock_version),
        VersionFile(
            Path("pyproject.toml"),
            lambda path: read_toml_version(path, "project"),
            lambda path, version: write_toml_version(path, "project", version),
        ),
        VersionFile(
            Path("wasm/package.json"),
            lambda path: read_json_version(path, "version"),
            lambda path, version: write_json_version(path, "version", version),
        ),
        VersionFile(
            Path("vcpkg/ports/vvcm-rs/vcpkg.json"),
            lambda path: read_json_version(path, "version-semver"),
            lambda path, version: write_json_version(path, "version-semver", version),
        ),
        VersionFile(
            Path("vcpkg/prebuilt-ports/vvcm-rs/vcpkg.json"),
            lambda path: read_json_version(path, "version-semver"),
            lambda path, version: write_json_version(path, "version-semver", version),
        ),
    ]


def read_versions(root: Path, files: list[VersionFile]) -> dict[Path, str]:
    versions: dict[Path, str] = {}
    for version_file in files:
        path = root / version_file.path
        try:
            versions[version_file.path] = version_file.read(path)
        except (
            OSError,
            KeyError,
            ValueError,
            tomllib.TOMLDecodeError,
            json.JSONDecodeError,
        ) as error:
            raise SystemExit(f"{version_file.path}: {error}") from error
    return versions


def synchronized_version(versions: dict[Path, str]) -> str:
    cargo_version = versions[Path("Cargo.toml")]
    mismatches = {
        path: version for path, version in versions.items() if version != cargo_version
    }
    if mismatches:
        lines = ["package versions do not match:"]
        lines.extend(f"  {path.as_posix()}: {version}" for path, version in versions.items())
        lines.append("Run `python scripts/sync_versions.py <version>` to update them.")
        raise SystemExit("\n".join(lines))
    return cargo_version


def print_version_table(version: str, versions: dict[Path, str]) -> None:
    print(f"All package versions are synchronized at {version}.")
    for path in versions:
        print(f"  {path.as_posix()}: {versions[path]}")


def update_versions(
    root: Path,
    files: list[VersionFile],
    versions: dict[Path, str],
    target_version: str,
    dry_run: bool,
) -> list[tuple[Path, str, str]]:
    changes = [
        (version_file.path, versions[version_file.path], target_version)
        for version_file in files
        if versions[version_file.path] != target_version
    ]
    if dry_run:
        return changes

    for version_file in files:
        old_version = versions[version_file.path]
        if old_version != target_version:
            version_file.write(root / version_file.path, target_version)
    return changes


def main() -> None:
    args = parse_args()
    root = args.root.resolve()
    files = version_files()
    versions = read_versions(root, files)

    if args.version:
        validate_version(args.version)
        changes = update_versions(root, files, versions, args.version, args.dry_run)
        if args.print_version:
            print(args.version)
            return
        if not changes:
            print(f"All package version files already use {args.version}.")
            return
        action = "Would update" if args.dry_run else "Updated"
        for path, old_version, new_version in changes:
            print(f"{action} {path.as_posix()}: {old_version} -> {new_version}")
        return

    version = synchronized_version(versions)
    if args.print_version:
        print(version)
    else:
        print_version_table(version, versions)


if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:
        sys.exit(1)
