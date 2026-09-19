"""Create a SHA-256 pip lock from an explicitly resolved, platform-local wheelhouse.

No download, install, dependency resolution, environment freeze or secret echo.
The lock becomes verified only after a fresh --no-index --require-hashes install
and pip check. Keep separate wheelhouses for each OS/architecture/Python.
"""
from __future__ import annotations

import argparse
import hashlib
import re
import zipfile
from email.parser import BytesParser
from pathlib import Path


def bootstrap_version(root: Path) -> str:
    lines = [s.strip() for s in (root / "tools/requirements-platformio.txt").read_text(encoding="utf-8").splitlines()
             if s.strip() and not s.lstrip().startswith("#")]
    if len(lines) != 1 or not re.fullmatch(r"platformio==[0-9]+(?:\.[0-9]+)+", lines[0]):
        raise ValueError("expected one exact PlatformIO bootstrap pin")
    return lines[0].split("==")[1]


def render_lock(wheelhouse: Path, expected_platformio: str) -> str:
    packages: dict[str, tuple[str, list[str]]] = {}
    wheels = sorted(wheelhouse.glob("*.whl"))
    if not wheels:
        raise ValueError("wheelhouse contains no wheels")
    for path in wheels:
        with zipfile.ZipFile(path) as wheel:
            entries = [x for x in wheel.infolist() if x.filename.endswith(".dist-info/METADATA")]
            if len(entries) != 1 or entries[0].file_size > 1024 * 1024:
                raise ValueError("wheel must contain one bounded package METADATA")
            meta = BytesParser().parsebytes(wheel.read(entries[0]))
        if len(meta.get_all("Name", [])) != 1 or len(meta.get_all("Version", [])) != 1:
            raise ValueError("wheel must have one Name and one Version header")
        name, version = meta.get("Name", ""), meta.get("Version", "")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", name):
            raise ValueError("invalid wheel package name")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.!+_-]*", version):
            raise ValueError("invalid wheel package version")
        name = re.sub(r"[-_.]+", "-", name).lower()
        if name in packages and packages[name][0] != version:
            raise ValueError(f"multiple versions for {name}; use an empty wheelhouse")
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        packages.setdefault(name, (version, []))[1].append(digest.hexdigest())
    if packages.get("platformio", (None, []))[0] != expected_platformio:
        raise ValueError("wheelhouse PlatformIO version does not match bootstrap pin")
    result = ["# Generated from wheel bytes; validate with an offline hash-checked install.",
              "# Platform-specific; dependency closure is verified by pip install + pip check."]
    for name, (version, hashes) in sorted(packages.items()):
        result.append(f"{name}=={version} " + " ".join(f"--hash=sha256:{h}" for h in sorted(set(hashes))))
    return "\n".join(result) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wheelhouse", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        content = render_lock(args.wheelhouse, bootstrap_version(Path(__file__).resolve().parents[1]))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        # Never silently replace an accepted lock with a new dependency solution.
        with args.output.open("x", encoding="utf-8", newline="\n") as stream:
            stream.write(content)
    except (OSError, ValueError, zipfile.BadZipFile) as exc:
        parser.exit(1, f"FAIL wheel lock: {exc}\n")
    print(f"WROTE {args.output}; installation/dependency closure NOT YET VERIFIED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
