#!/usr/bin/env python3
"""Guard transform_cache transform-cache and overload-telemetry semantics."""

from __future__ import annotations

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import (
    compiler, forbid, require, run, stack_usage,
)


from quality_gate_runtime import project_temp_directory, strongest_supported_sanitizer_flags

ROOT = Path(__file__).resolve().parents[2]


def text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def main() -> int:
    frame = text("src/sensor/frame_transform.hpp")
    pipeline = text("src/runtime/imu_sample_pipeline.cpp")
    pipeline_h = text("src/runtime/imu_sample_pipeline.hpp")
    mag_h = text("src/sensor/mag_runtime.hpp")
    mag = text("src/sensor/mag_runtime.cpp")
    controller_h = text("src/runtime/mag_runtime_controller.hpp")
    controller = text("src/runtime/mag_runtime_controller.cpp")
    profiler_h = text("src/runtime/runtime_profiler.hpp")
    profiler = text("src/runtime/runtime_profiler.cpp")
    tuning = text("src/build_config/tracking_tuning.hpp")
    ahrs = text("src/sensor/ahrs_6dof.cpp")
    check_all = text("tools/check_all.py")

    # The cache is revision-bound and still validates changed candidates.
    require(frame, "class SensorToDeviceFrameCache", "frame cache")
    require(frame, "configRevision_ == configRevision", "constant-time cache hit")
    require(frame, "frame_ = makeSensorToDeviceFrame(valid, rotation)", "fail-closed refresh")
    require(frame, "++refreshes_", "refresh diagnostic")
    require(pipeline_h, "SensorToDeviceFrameCache* sensorToDeviceFrameCache", "pipeline cache dependency")
    require(pipeline, "deps.sensorToDeviceFrameCache->resolve", "IMU hotpath cache use")
    require(pipeline, "deps.config.data.crc32", "config revision")
    require(pipeline, "fallbackFrame = makeSensorToDeviceFrame", "standalone fallback")

    # Magnetic processing reuses the same accepted frame and caches the large
    # immutable runtime config. Direct callers retain validation fallback.
    require(mag_h, "sensorToDevicePrevalidated", "prevalidated frame contract")
    require(mag, "cfg.sensorToDevicePrevalidated", "mag cache use")
    require(mag, "makeSensorToDeviceFrame(cfg.sensorToDeviceValid", "mag fallback validation")
    require(controller_h, "const MagRuntimeConfig& runtimeConfig() const", "cached config reference")
    require(controller_h, "runtimeConfigCacheRevision_", "mag config revision")
    require(controller, "runtimeConfigCacheRevision_ == revision", "mag cache hit")
    require(controller, "c.sensorToDevicePrevalidated = true", "accepted frame publication")
    require(controller, "const MagRuntimeConfig& magCfg = runtimeConfig()", "no per-sample config copy")

    # Overload telemetry must remain useful beyond 100 ms and never print
    # UINT32_MAX merely because a percentile entered the open-ended bucket.
    require(profiler_h, "kBucketCount = 24u", "extended fixed histogram")
    for bound in ("150000u", "250000u", "500000u", "750000u", "1000000u", "2000000u"):
        require(profiler, bound, f"age histogram bound {bound}")
    require(profiler, "return i + 1u == kBucketCount ? maxUs", "open-ended bucket reporting")

    # Tracking-quality invariants stay unchanged.
    require(tuning, "AHRS_ACCEL_CORRECTION_DIVISOR = 4", "accel correction cadence")
    require(tuning, "PREPARED_OUTPUT_MIN_INTERVAL_US = 4000", "prepared output cadence")
    require(ahrs, "const Quat predicted = integrateBodyRateFast(q_, gyroUsed, dtS);", "transactional full-rate gyro prediction")
    require(ahrs, "q_ = predicted;", "checked full-rate gyro commit")
    for forbidden in ("dropOldest", "discardRaw", "decimateGyro"):
        forbid(pipeline, forbidden, f"sample loss {forbidden}")


    cxx = compiler()
    with project_temp_directory(ROOT, "tracker-pre0024ab-") as temp_name:
        tmp = Path(temp_name)
        common = ["-std=c++20", "-O2", "-Wall", "-Wextra", "-Werror",
                  "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native")]

        run([
            cxx, *common,
            str(ROOT / "tests/native/policy_frame_cache.cpp"),
            str(ROOT / "tests/native/sanitizer_runtime_options.cpp"),
            str(ROOT / "src/sensor/mag_runtime.cpp"),
            "-o", str(tmp / "frame_cache"),
        ])
        run([str(tmp / "frame_cache")])

        sanitizer_name, sanitizer_flags = strongest_supported_sanitizer_flags(cxx, ROOT)
        if sanitizer_flags:
            print(f"# transform_cache sanitizer={sanitizer_name}")
            sanitizer = ["-std=c++20", "-O1", "-g", "-Wall", "-Wextra", "-Werror",
                         *sanitizer_flags,
                         "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native")]
            run([
                cxx, *sanitizer,
                str(ROOT / "tests/native/policy_frame_cache.cpp"),
                str(ROOT / "tests/native/sanitizer_runtime_options.cpp"),
                str(ROOT / "src/sensor/mag_runtime.cpp"),
                "-o", str(tmp / "frame_cache_sanitized"),
            ])
            run([str(tmp / "frame_cache_sanitized")])
        else:
            print("# transform_cache sanitizer: SKIP (toolchain cannot link ASan/UBSan)")

        run([
            cxx, *common,
            str(ROOT / "tests/native/test_runtime_profiler.cpp"),
            str(ROOT / "tests/native/sanitizer_runtime_options.cpp"),
            str(ROOT / "src/runtime/runtime_profiler.cpp"),
            "-o", str(tmp / "runtime_profiler"),
        ])
        run([str(tmp / "runtime_profiler")])

        for source in (
            "src/runtime/imu_sample_pipeline.cpp",
            "src/runtime/mag_runtime_controller.cpp",
        ):
            run([
                cxx, "-std=c++20", "-O2", "-fstack-usage",
                "-I", str(ROOT / "src"), "-I", str(ROOT / "tests/native"),
                "-c", str(ROOT / source),
                "-o", str(tmp / (Path(source).stem + ".o")),
            ])
        for symbol, ceiling in (
            ("imuSamplePipelineProcessRaw", 512),
            ("MagRuntimeController::processRawSample", 1024),
        ):
            measured = stack_usage(tmp, symbol)
            if measured > ceiling:
                raise SystemExit(f"{symbol} stack {measured} exceeds {ceiling}")

    print("# transform_cache: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
