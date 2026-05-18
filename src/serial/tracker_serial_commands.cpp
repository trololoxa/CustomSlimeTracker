#include "serial/tracker_serial_commands.hpp"

namespace tracker {

void TrackerCommandDispatcher::dispatch(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    if (argc <= 0 || !argv || !argv[0]) return;

    if (tracker_serial_detail::eqIgnoreCase(argv[0], "setup")) {
        trackerSerialDispatchSetupCommand(ctx, argc, argv);
        return;
    }

    if (trackerSerialDispatchSlimeVRSerialCompatCommand(ctx, argc, argv)) {
        return;
    }

    if (trackerSerialDispatchSystemCommand(ctx, argc, argv)) {
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[0], "config")) {
        trackerSerialDispatchConfigCommand(ctx, argc, argv);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[0], "mag")) {
        trackerSerialDispatchMagCommand(ctx, argc, argv);
        return;
    }

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

    if (tracker_serial_detail::eqIgnoreCase(argv[0], "cal")) {
        trackerSerialDispatchCalibrationCommand(ctx, argc, argv);
        return;
    }

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

    if (tracker_serial_detail::eqIgnoreCase(argv[0], "tap")) {
        trackerSerialDispatchTapCommand(ctx, argc, argv);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[0], "led")) {
        trackerSerialDispatchLedCommand(ctx, argc, argv);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[0], "test")) {
        trackerSerialDispatchTestCommand(ctx, argc, argv);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[0], "output")) {
        trackerSerialDispatchOutputCommand(ctx, argc, argv);
        return;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[0], "net")) {
        trackerSerialDispatchNetworkCommand(ctx, argc, argv);
        return;
    }

    if (trackerSerialDispatchSlimeVRCommand(ctx, argc, argv)) {
        return;
    }

    Stream& out = ctx.io ? *ctx.io : Serial;
    tracker_serial_detail::printErr(out, "unknown command; type help");
}

} // namespace tracker
