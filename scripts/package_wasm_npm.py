#!/usr/bin/env python3
"""Prepare scoped and unscoped npm package directories for the WASM build."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


PACKAGE_NAMES = ("@morningfrog/vvcm-rs", "vvcm-rs")


def package_dir_name(package_name: str) -> str:
    return package_name.replace("@", "").replace("/", "-")


def copy_package(source_dir: Path, output_dir: Path, package_name: str) -> Path:
    target_dir = output_dir / package_dir_name(package_name)
    if target_dir.exists():
        shutil.rmtree(target_dir)
    shutil.copytree(
        source_dir,
        target_dir,
        ignore=shutil.ignore_patterns("node_modules", "package-lock.json", "*.tgz", ".gitignore"),
    )

    manifest_path = target_dir / "package.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["name"] = package_name
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    return target_dir


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Prepare npm package directories for vvcm-rs WASM publishing."
    )
    parser.add_argument(
        "--source-dir",
        type=Path,
        default=Path("wasm"),
        help="Built npm package source directory.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("npm-dist"),
        help="Directory that receives publish-ready package directories.",
    )
    args = parser.parse_args()

    source_dir = args.source_dir
    pkg_dir = source_dir / "pkg"
    if not (pkg_dir / "vvcm_rs.js").exists() or not (pkg_dir / "vvcm_rs_bg.wasm").exists():
        raise SystemExit(
            "missing wasm/pkg output; run wasm-pack build before packaging npm artifacts"
        )

    args.out_dir.mkdir(parents=True, exist_ok=True)
    for package_name in PACKAGE_NAMES:
        target = copy_package(source_dir, args.out_dir, package_name)
        print(f"prepared {package_name} at {target}")


if __name__ == "__main__":
    main()
