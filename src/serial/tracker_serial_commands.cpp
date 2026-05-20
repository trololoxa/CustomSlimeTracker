#include "serial/tracker_serial_commands.hpp"

namespace tracker {

void TrackerCommandDispatcher::dispatch(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    if (argc <= 0 || !argv || !argv[0]) return;

#if TRACKER_ENABLE_SERIAL_CLI
#if TRACKER_ENABLE_SETUP_COMMANDS
    if (tracker_serial_detail::eqIgnoreCase(argv[0], "setup")) {
        trackerSerialDispatchSetupCommand(ctx, argc, argv);
        return;
    }
#endif

#if TRACKER_ENABLE_SLIMEVR_SERIAL_COMPAT
    if (trackerSerialDispatchSlimeVRSerialCompatCommand(ctx, argc, argv)) {
        return;
    }
#endif

    if (trackerSerialDispatchSystemCommand(ctx, argc, argv)) {
        return;
    }

#if TRACKER_ENABLE_CONFIG_COMMANDS
    if (tracker_serial_detail::eqIgnoreCase(argv[0], "config")) {
        trackerSerialDispatchConfigCommand(ctx, argc, argv);
        return;
    }
#endif

#if TRACKER_ENABLE_MAG_COMMANDS
    if (tracker_serial_detail::eqIgnoreCase(argv[0], "mag")) {
        trackerSerialDispatchMagCommand(ctx, argc, argv);
        return;
    }
#endif

#if TRACKER_ENABLE_FULL_CLI
    if (tracker_serial_detail::eqIgnoreCase(argv[0], "fifo")) {
        trackerSerialDispatchFifoCommand(ctx, argc, argv);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[0], "imu")) {
        trackerSerialDispatchImuCommand(ctx, argc, argv);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[0], "quality")) {
        trackerSerialDispatchQualityCommand(ctx, argc, argv);
        return;
    }
#endif

#if TRACKER_ENABLE_CALIBRATION_COMMANDS
    if (tracker_serial_detail::eqIgnoreCase(argv[0], "cal")) {
        trackerSerialDispatchCalibrationCommand(ctx, argc, argv);
        return;
    }
#endif

#if TRACKER_ENABLE_FULL_CLI
    if (tracker_serial_detail::eqIgnoreCase(argv[0], "ahrs")) {
        trackerSerialDispatchAhrsCommand(ctx, argc, argv);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[0], "stream")) {
        trackerSerialDispatchStreamCommand(ctx, argc, argv);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[0], "log")) {
        trackerSerialDispatchLogCommand(ctx, argc, argv);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[0], "bias")) {
        trackerSerialDispatchBiasCommand(ctx, argc, argv);
        return;
    }
#endif

#if TRACKER_ENABLE_TAP_RUNTIME
    if (tracker_serial_detail::eqIgnoreCase(argv[0], "tap")) {
        trackerSerialDispatchTapCommand(ctx, argc, argv);
        return;
    }
#endif

#if TRACKER_ENABLE_STATUS_LED
    if (tracker_serial_detail::eqIgnoreCase(argv[0], "led")) {
        trackerSerialDispatchLedCommand(ctx, argc, argv);
        return;
    }
#endif

#if TRACKER_ENABLE_BATTERY_RUNTIME
    if (tracker_serial_detail::eqIgnoreCase(argv[0], "battery") ||
        tracker_serial_detail::eqIgnoreCase(argv[0], "bat")) {
        trackerSerialDispatchBatteryCommand(ctx, argc, argv);
        return;
    }
#endif

#if TRACKER_ENABLE_TEST_COMMANDS
    if (tracker_serial_detail::eqIgnoreCase(argv[0], "test")) {
        trackerSerialDispatchTestCommand(ctx, argc, argv);
        return;
    }
#endif

#if TRACKER_ENABLE_FULL_CLI
    if (tracker_serial_detail::eqIgnoreCase(argv[0], "output")) {
        trackerSerialDispatchOutputCommand(ctx, argc, argv);
        return;
    }
#endif

#if TRACKER_ENABLE_NETWORK_COMMANDS
    if (tracker_serial_detail::eqIgnoreCase(argv[0], "net")) {
        trackerSerialDispatchNetworkCommand(ctx, argc, argv);
        return;
    }
#endif

#if TRACKER_ENABLE_SLIMEVR_COMMANDS
    if (trackerSerialDispatchSlimeVRCommand(ctx, argc, argv)) {
        return;
    }
#endif

    Stream& out = ctx.io ? *ctx.io : Serial;
    tracker_serial_detail::printErr(out, "unknown command; type help");
#else
    (void)ctx;
#endif
}

} // namespace tracker
