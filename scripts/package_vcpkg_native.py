#!/usr/bin/env python3
"""Package vvcm-rs native artifacts for the prebuilt vcpkg overlay."""

from __future__ import annotations

import argparse
import shutil
import tempfile
import zipfile
from pathlib import Path


SUPPORTED_TRIPLETS = ("x64-windows", "x64-linux", "arm64-osx")
PROFILES = ("release", "debug")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", required=True)
    parser.add_argument("--triplet", required=True, choices=SUPPORTED_TRIPLETS)
    parser.add_argument("--target-dir", default="target")
    parser.add_argument("--out-dir", default="native-dist")
    return parser.parse_args()


def copy_required(source: Path, destination: Path) -> None:
    if not source.exists():
        raise SystemExit(f"required build artifact is missing: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def profile_prefix(root: Path, linkage: str, profile: str) -> Path:
    base = root / linkage
    if profile == "debug":
        base = base / "debug"
    return base


def copy_headers(repo_root: Path, package_root: Path) -> None:
    include_dir = package_root / "include"
    include_dir.mkdir(parents=True, exist_ok=True)
    copy_required(repo_root / "include" / "vvcm_rs.h", include_dir / "vvcm_rs.h")
    copy_required(repo_root / "include" / "vvcm_rs.hpp", include_dir / "vvcm_rs.hpp")
    copy_required(repo_root / "LICENSE", package_root / "LICENSE")


def package_windows(target_dir: Path, package_root: Path) -> None:
    for profile in PROFILES:
        source_dir = target_dir / profile
        dynamic_root = profile_prefix(package_root, "dynamic", profile)
        static_root = profile_prefix(package_root, "static", profile)

        copy_required(
            source_dir / "vvcm_rs.dll",
            dynamic_root / "bin" / "vvcm_rs.dll",
        )
        copy_required(
            source_dir / "vvcm_rs.dll.lib",
            dynamic_root / "lib" / "vvcm_rs.lib",
        )
        copy_required(
            source_dir / "vvcm_rs.lib",
            static_root / "lib" / "vvcm_rs.lib",
        )


def package_unix(target_dir: Path, package_root: Path, shared_name: str) -> None:
    for profile in PROFILES:
        source_dir = target_dir / profile
        dynamic_root = profile_prefix(package_root, "dynamic", profile)
        static_root = profile_prefix(package_root, "static", profile)

        copy_required(
            source_dir / shared_name,
            dynamic_root / "lib" / shared_name,
        )
        copy_required(
            source_dir / "libvvcm_rs.a",
            static_root / "lib" / "libvvcm_rs.a",
        )


def zip_directory(source_dir: Path, archive_path: Path) -> None:
    archive_path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(source_dir.rglob("*")):
            if path.is_file():
                archive.write(path, path.relative_to(source_dir.parent).as_posix())


def main() -> None:
    args = parse_args()
    repo_root = Path.cwd()
    target_dir = (repo_root / args.target_dir).resolve()
    out_dir = (repo_root / args.out_dir).resolve()

    package_name = f"vvcm-rs-{args.version}-{args.triplet}"
    archive_path = out_dir / f"{package_name}.zip"

    with tempfile.TemporaryDirectory(prefix="vvcm-rs-native-") as temp_dir:
        package_root = Path(temp_dir) / package_name
        copy_headers(repo_root, package_root)

        if args.triplet == "x64-windows":
            package_windows(target_dir, package_root)
        elif args.triplet == "x64-linux":
            package_unix(target_dir, package_root, "libvvcm_rs.so")
        elif args.triplet == "arm64-osx":
            package_unix(target_dir, package_root, "libvvcm_rs.dylib")
        else:
            raise SystemExit(f"unsupported triplet: {args.triplet}")

        zip_directory(package_root, archive_path)

    print(f"Wrote {archive_path}")


if __name__ == "__main__":
    main()
