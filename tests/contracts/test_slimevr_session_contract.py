#!/usr/bin/env python3
"""Source-level composition checks for the patch-0020 session contract."""

import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "tools"),
                str(Path(__file__).resolve().parents[1] / "support")]
from contract_checks import forbid, require


ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    app = (ROOT / "src/app/tracker_app.cpp").read_text(encoding="utf-8")
    hooks = (ROOT / "src/app/hooks/tracker_app_runtime_hooks.hpp").read_text(encoding="utf-8")
    runtime = (ROOT / "src/runtime/slimevr_output_runtime_impl.inc").read_text(encoding="utf-8")
    version = (ROOT / "src/build_config/firmware_feature_version.hpp").read_text(encoding="utf-8")

    start = app[app.index("void TrackerApp::startMagFromConfig"):]
    require(start, "startMagRuntimeFromPreconfiguredFifo", "preconfigured mag startup callback")
    forbid(start.split("}\n", 1)[0], "setMagRuntimeEnabled(true, false)", "redundant boot FIFO reconfigure")
    require(hooks, "cfg.physicalTapUserAction = g_networkConfig.tapUserAction();", "boot tap mapping")
    require(hooks, "setMagYawCorrectionApplyEnabledHook(enabled, true", "persistent SetConfigFlag")
    require(runtime, "parseShortSensorInfoAck", "short SensorInfo ACK parser")
    require(runtime, "SlimeVRSensorInfoSyncState::Acknowledged", "SensorInfo acknowledged state")
    require(runtime, "remote != serverEndpoint_", "strict endpoint gate")
    require(runtime, "len < SLIMEVR_PACKET_HEADER_SIZE", "strict normal-packet header")
    require(runtime, "malformedDatagramLength_", "truncated datagram rejection")
    require(runtime, "resolveHost(manualServerHost_", "manual server resolver")
    require(runtime, "remote != manualServerEndpoint_", "manual endpoint exclusivity")
    require(hooks, "cfg.manualServerHost = g_networkConfig.data.serverHost;", "manual host wiring")

    startup = app[app.index("bool TrackerApp::setupSensorRuntime"):
                  app.index("void TrackerApp::enterFatalDegraded")]
    pause_i = startup.index("pauseFifo()")
    mag_i = startup.index("startMagFromConfig")
    reset_i = startup.index("resetFifo()")
    attach_i = startup.index("attachFifoInterrupt")
    if not (pause_i < mag_i < reset_i < attach_i):
        raise AssertionError("boot FIFO epoch order must be pause -> mag -> reset -> attach")

    print("PASS slimevr_session_contract_policy")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
