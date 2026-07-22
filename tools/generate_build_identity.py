#!/usr/bin/env python3
"""PlatformIO pre-build hook for deterministic firmware revision metadata."""

from __future__ import annotations

import sys
from pathlib import Path

# Importing the helper must not create an untracked __pycache__ that would make
# an otherwise clean repository identify itself as dirty.
sys.dont_write_bytecode = True

Import("env")  # type: ignore[name-defined]  # Provided by PlatformIO/SCons.

# PlatformIO executes extra_scripts through SCons exec(). In that context
# __file__ is not guaranteed to exist, so derive the helper location from the
# project root supplied by PlatformIO instead of relying on Python script globals.
project_dir = Path(env.subst("$PROJECT_DIR")).resolve()
TOOLS_DIR = project_dir / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from build_identity import collect_build_identity, render_generated_header, write_if_changed

build_dir = Path(env.subst("$BUILD_DIR")).resolve()
pio_environment = str(env.subst("$PIOENV"))
generated_dir = build_dir / "generated"
generated_header = generated_dir / "tracker_build_identity_generated.hpp"

identity = collect_build_identity(project_dir)
content = render_generated_header(identity, pio_environment)
changed = write_if_changed(generated_header, content)

env.Prepend(CPPPATH=[str(generated_dir)])
state = "updated" if changed else "current"
print(f"# build identity ({state}): env={pio_environment} git={identity.identity}")
