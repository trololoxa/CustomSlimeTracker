#!/usr/bin/env python3
"""Guard 0026a motion-policy isolation and high-dip magnetic hardening."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(text: str, needle: str, label: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {label}: {needle}")


def forbid(text: str, needle: str, label: str) -> None:
    if needle in text:
        raise SystemExit(f"forbidden {label}: {needle}")


def main() -> int:
    core = read("src/core/slimevr_motion_policy.hpp")
    schema = read("src/config/tracker_config_schema.hpp")
    config_runtime = read("src/config/tracker_config_runtime.cpp")
    store = read("src/config/tracker_config_store.cpp")
    runtime_h = read("src/runtime/slimevr_output_runtime.hpp")
    runtime = read("src/runtime/slimevr_output_runtime_impl.inc")
    serial = read("src/serial/tracker_slimevr_commands.cpp")
    serial_ctx = read("src/serial/tracker_serial_context.hpp")
    app_hooks = read("src/app/hooks/tracker_app_runtime_hooks.hpp")
    horizontal = read("src/sensor/mag_horizontal_trust.hpp")
    reliability = read("src/sensor/mag_field_reliability.cpp")
    reliability_h = read("src/sensor/mag_field_reliability.hpp")
    yaw_h = read("src/sensor/mag_yaw_correction.hpp")
    mag_test = read("tests/native/test_mag_heading_reliability.cpp")
    motion_test = read("tests/native/test_slimevr_motion_mode_command.cpp")
    docs = read("docs/slimevr_network.md") + read("docs/magnetic_heading_reliability.md")

    # One canonical persisted/runtime policy type; the effective negotiated wire
    # mode deliberately remains separate.
    require(core, "enum class SlimeVRMotionPacketPolicy", "canonical motion policy enum")
    require(core, "slimevrMotionPacketPolicyValid", "canonical policy validation")
    require(core, "slimevrMotionPacketPolicyName", "canonical policy diagnostics")
    require(schema, '#include "core/slimevr_motion_policy.hpp"', "config use of canonical policy")
    require(runtime_h, '#include "core/slimevr_motion_policy.hpp"', "runtime use of canonical policy")
    require(runtime_h, "enum class SlimeVRMotionPacketMode", "separate effective wire mode")
    forbid(schema, "TrackerSlimeVRMotionMode", "duplicate config policy enum")
    forbid(runtime_h, "enum class SlimeVRMotionPacketPolicy", "duplicate runtime policy enum")
    forbid(serial, "motionPacketPolicyFromConfig", "serial policy mapping copy")
    forbid(app_hooks, "appSlimeVRMotionPacketPolicy", "app policy mapping copy")

    # The command must go through an app/domain hook and storage must persist only
    # the selected policy from the authoritative NVS generation.
    require(serial_ctx, "setSlimeVRMotionPacketPolicy", "motion policy domain hook")
    require(serial, "ctx.setSlimeVRMotionPacketPolicy(mode", "CLI domain-hook invocation")
    forbid(serial, "ctx.configStore->save(*ctx.config", "whole-config motion-mode save")
    require(store, "saveSlimeVRMotionPacketPolicy", "field-isolated storage API")
    require(store, "verify(*persisted)", "authoritative stored-generation read")
    require(store, "persisted->setSlimeVRMotionPacketPolicy(policy)", "candidate field-only edit")
    require(store, "activeConfig.setSlimeVRMotionPacketPolicy(policy)", "active field-only commit")
    require(app_hooks, "g_configStore.saveSlimeVRMotionPacketPolicy", "app persistence ownership")
    require(app_hooks, "g_slimevrRuntime.setMotionPacketPolicy(policy)", "field-local runtime apply")
    require(runtime, "void SlimeVROutputRuntime::setMotionPacketPolicy", "runtime field setter")
    require(motion_test, "remain RAM-only", "collateral-save regression")
    require(motion_test, "motion-mode control unavailable", "missing-hook fail-closed regression")

    # High-dip tuning is typed, shared, and all direction-only discontinuity gates
    # respect observability and the same geometry scale.
    require(horizontal, "struct MagHorizontalTrustConfig", "typed horizontal trust policy")
    require(horizontal, "referenceBadFraction", "reference bad fraction policy")
    require(horizontal, "headingNoiseScaleMax", "heading noise scale policy")
    forbid(horizontal, "constexpr float kAbsoluteBadFloor", "hidden horizontal floor tuning")
    require(reliability_h, "MagHorizontalTrustConfig horizontalTrust", "reliability typed policy")
    forbid(yaw_h, "MagHorizontalTrustConfig horizontalTrust", "duplicate yaw horizontal policy state")
    require(reliability, "magnitudeExceedsScaledThreshold", "scaled legacy step gate")
    require(reliability, "cfg.headingStepSoftDeg", "scaled legacy soft step threshold")
    require(reliability, "cfg.headingStepHardDeg", "scaled legacy hard step threshold")
    require(reliability, "stationaryForHeadingCheck && headingObservable", "observability-gated step detection")
    require(reliability, "magnitudeAtMostScaledThreshold", "scaled recovery heading gate")
    require(mag_test, "testThirtyMinuteHighDipNoiseAndDisturbanceRecovery", "30-minute high-dip regression")
    require(mag_test, "testNearVerticalFieldRemainsFailClosedForYaw", "near-vertical fail-closed regression")
    require(mag_test, "testHighDipLegacyHeadingStepGateScalesWithObservability", "legacy step scaling regression")
    require(docs, "persists only", "isolated persistence documentation")
    require(docs, "typed", "typed magnetic tuning documentation")

    # Forward/default semantics from 0026 remain unchanged.
    require(config_runtime, "OUTPUT_PACKET_MODE_MARKER", "persisted marker contract")
    require(config_runtime, "SlimeVRMotionPacketPolicy::QuaternionOnly", "quaternion-only legacy/default target")

    print("# 0026a_motion_policy_and_magnetic_reliability_hardening_policy: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
