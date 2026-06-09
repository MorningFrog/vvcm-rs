#!/usr/bin/env python3
"""Generate the release-ready prebuilt vcpkg overlay."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import tempfile
import zipfile
from pathlib import Path


SUPPORTED_TRIPLETS = ("x64-windows", "x64-linux", "x64-osx")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", required=True)
    parser.add_argument("--native-dist-dir", default="native-dist")
    parser.add_argument("--template-dir", default="vcpkg/prebuilt-ports/vvcm-rs")
    parser.add_argument("--out-dir", default="prebuilt-vcpkg-dist")
    return parser.parse_args()


def sha512(path: Path) -> str:
    digest = hashlib.sha512()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def copy_template(template_dir: Path, port_dir: Path) -> None:
    def ignore(_directory: str, names: list[str]) -> set[str]:
        return {name for name in names if name.endswith(".in")}

    shutil.copytree(template_dir, port_dir, ignore=ignore)


def render_portfile(template_dir: Path, port_dir: Path, version: str, hashes: dict[str, str]) -> None:
    template = (template_dir / "portfile.cmake.in").read_text(encoding="utf-8")
    values = {
        "@VVCM_RS_RELEASE_TAG@": f"v{version}",
        "@VVCM_RS_SHA512_X64_WINDOWS@": hashes["x64-windows"],
        "@VVCM_RS_SHA512_X64_LINUX@": hashes["x64-linux"],
        "@VVCM_RS_SHA512_X64_OSX@": hashes["x64-osx"],
    }
    for token, value in values.items():
        template = template.replace(token, value)
    (port_dir / "portfile.cmake").write_text(template, encoding="utf-8")


def update_manifest(port_dir: Path, version: str) -> None:
    manifest_path = port_dir / "vcpkg.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["version-semver"] = version
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def write_readme(root: Path, version: str) -> None:
    (root / "README.md").write_text(
        "\n".join(
            [
                f"# vvcm-rs {version} prebuilt vcpkg overlay",
                "",
                "Install with:",
                "",
                "```shell",
                "vcpkg install vvcm-rs --overlay-ports=<this-directory>/ports --triplet x64-windows",
                "```",
                "",
                "Supported prebuilt asset triplets: x64-windows, x64-linux, x64-osx.",
                "The overlay downloads native binaries from the matching GitHub Release and does not require Rust.",
                "",
            ]
        ),
        encoding="utf-8",
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
    native_dist_dir = (repo_root / args.native_dist_dir).resolve()
    template_dir = (repo_root / args.template_dir).resolve()
    out_dir = (repo_root / args.out_dir).resolve()

    hashes: dict[str, str] = {}
    for triplet in SUPPORTED_TRIPLETS:
        archive = native_dist_dir / f"vvcm-rs-{args.version}-{triplet}.zip"
        if not archive.exists():
            raise SystemExit(f"required native package is missing: {archive}")
        hashes[triplet] = sha512(archive)

    archive_name = f"vvcm-rs-{args.version}-vcpkg-prebuilt-overlay"
    archive_path = out_dir / f"{archive_name}.zip"

    with tempfile.TemporaryDirectory(prefix="vvcm-rs-prebuilt-port-") as temp_dir:
        root = Path(temp_dir) / archive_name
        port_dir = root / "ports" / "vvcm-rs"
        copy_template(template_dir, port_dir)
        render_portfile(template_dir, port_dir, args.version, hashes)
        update_manifest(port_dir, args.version)
        write_readme(root, args.version)
        zip_directory(root, archive_path)

    print(f"Wrote {archive_path}")


if __name__ == "__main__":
    main()
