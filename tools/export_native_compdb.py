#!/usr/bin/env python3
"""Export successful native compile argv for navigation; never execute report commands."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def entries(data: dict) -> list[dict]:
    if (data.get("schema") != "tracker-gate-run-v1" or data.get("runner") != "native"
            or data.get("state") != "completed" or data.get("returncode") != 0
            or data.get("scope") not in ("focused", "focused-build", "full-native", "build-only")):
        raise ValueError("requires a completed successful native report; navigation only")
    result = []
    for record in data.get("commands", []):
        argv = record.get("command")
        if not isinstance(argv, list) or not all(isinstance(x, str) for x in argv):
            raise ValueError("invalid command argv")
        if "-c" not in argv:
            continue
        if record.get("returncode") != 0 or record.get("state") != "completed":
            raise ValueError("unsuccessful compile record")
        directory = Path(record["cwd"])
        if directory.resolve() != ROOT.resolve():
            raise ValueError("report belongs to another checkout; generate it locally")
        index = argv.index("-c")
        if index < 2 or "-o" not in argv:
            raise ValueError("invalid compile record")
        source = Path(argv[index - 1])
        source = source if source.is_absolute() else directory / source
        source = source.resolve()
        if not source.is_relative_to(ROOT.resolve()) or not source.is_file():
            raise ValueError("compile source is missing or outside checkout")
        result.append({"directory": str(directory), "file": str(source), "arguments": argv})
    if not result:
        raise ValueError("report contains no compiled sources")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args()
    temporary = None
    try:
        data = json.loads(args.report.read_text(encoding="utf-8"))
        commands = entries(data)
        target = ROOT / "build" / "clangd-native" / "compile_commands.json"
        target.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=target.parent,
                                         suffix=".tmp", delete=False) as handle:
            temporary = Path(handle.name)
            json.dump(commands, handle, indent=2)
            handle.write("\n")
        os.replace(temporary, target)
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        print(f"FAIL native compilation database: {exc}")
        return 1
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    print(f"OK native navigation database: {len(commands)} units; scope={data['scope']}; {target}")
    print("# Host stubs/flags only; not ESP32 proof. Regenerate after compile configuration changes.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
