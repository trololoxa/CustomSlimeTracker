#!/usr/bin/env python3
"""Guard 0026c command-hook declaration order and compile coverage."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def main() -> int:
    command_hooks = read("src/app/hooks/tracker_app_command_hooks.hpp")
    runtime_hooks = read("src/app/hooks/tracker_app_runtime_hooks.hpp")
    runner = read("tools/run_standalone_tests.py")

    declaration = "static bool setSlimeVRMotionPacketPolicyHook(\n    SlimeVRMotionPacketPolicy policy,\n    void* user\n);"
    factory = "static TrackerCommandRuntimeHooks makeTrackerCommandRuntimeHooks()"
    assignment = "hooks.setSlimeVRMotionPacketPolicy = setSlimeVRMotionPacketPolicyHook;"
    definition = "static bool setSlimeVRMotionPacketPolicyHook(SlimeVRMotionPacketPolicy policy, void* user) {"

    declaration_pos = command_hooks.find(declaration)
    factory_pos = command_hooks.find(factory)
    assignment_pos = command_hooks.find(assignment)
    if declaration_pos < 0:
        raise SystemExit("missing setSlimeVRMotionPacketPolicyHook forward declaration")
    if factory_pos < 0 or assignment_pos < 0:
        raise SystemExit("missing command-hook factory wiring")
    if not declaration_pos < factory_pos <= assignment_pos:
        raise SystemExit("motion-policy hook must be declared before command-hook factory wiring")
    if definition not in runtime_hooks:
        raise SystemExit("missing motion-policy hook implementation in runtime hooks")

    # This is the real compilation regression for the original failure: tracker_app.cpp
    # includes the aggregate app hook chain under ARDUINO and therefore catches a
    # declaration-order break before target PlatformIO builds.
    arduino_compile_entry = 'pathlib.Path("src/app/tracker_app.cpp")'
    if arduino_compile_entry not in runner:
        raise SystemExit("standalone runner no longer compile-checks src/app/tracker_app.cpp")

    print("# 0026c_command_hook_declaration_order_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
