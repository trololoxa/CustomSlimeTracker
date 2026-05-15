#include "serial/tracker_system_commands.hpp"

#include <Arduino.h>

#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/calibration.hpp"
#include "serial/tracker_serial_context.hpp"
#include "serial/tracker_imu_fifo_commands.hpp"

namespace tracker {

Stream& trackerSerialSystemStream(TrackerSerialCommandContext& ctx) {
    return ctx.io ? *ctx.io : Serial;
}

bool trackerSerialSystemIs(const char* a, const char* b) {
    return tracker_serial_detail::eqIgnoreCase(a, b);
}

const char* trackerSerialSystemStreamModeName(TrackerStreamMode mode) {
    switch (mode) {
        case TrackerStreamMode::Off:        return "off";
        case TrackerStreamMode::Heartbeat:  return "heartbeat";
        case TrackerStreamMode::Raw:        return "raw";
        case TrackerStreamMode::Scaled:     return "scaled";
        case TrackerStreamMode::Quat:       return "quat";
        case TrackerStreamMode::Debug:      return "debug";
    }
    return "unknown";
}

void trackerSerialPrintHelp(Stream& out) {
    out.println("==============================================================================");
    out.println("TRACKER SERIAL COMMANDS");
    out.println("==============================================================================");
    out.println("[core]");
    out.println("  help | ?");
    out.println("  status | health | setup status | version | reboot | factory_reset");
    out.println();
    out.println("[config]");
    out.println("  config print | load | save | defaults | erase | crc | nvs");
    out.println("  config spi <hz> [save]          (live SPI clock, e.g. 1000000/4000000/8000000)");
    out.println();
    out.println("[imu/fifo/quality]");
    out.println("  imu status | whoami | read");
    out.println("  imu rate <120|240|480|960> [save]   (live IMU+FIFO ODR reconfigure)");
    out.println("  fifo status | stats | reset");
    out.println("  fifo watermark <words> [save] | drain <max_words> <rounds> [save]");
    out.println("  quality stats | reset");
    out.println();
    out.println("[calibration]");
    out.println("  cal gyro | cal gyro save | cal gyro clear");
    out.println("  cal accel face XP|XN|YP|YN|ZP|ZN");
    out.println("  cal accel compute | dump | save | clear");
    out.println("  cal temp print | enable [save] | disable [save]");
    out.println("  cal temp set_slope X Y Z [save] | fit_static [save] | clear [save]");
    out.println("  cal save | clear_all");
    out.println();
    out.println("[ahrs]");
    out.println("  ahrs status | config | reset | defaults [save]");
    out.println("  ahrs accel on|off [save] | adaptive on|off [save]");
    out.println("  ahrs accel_kp <gain> [save] | max_step <deg> [save]");
    out.println("  ahrs accel_norm <goodErrG> <badErrG> [save]");
    out.println("  ahrs accel_innovation <goodDeg> <badDeg> [save]");
    out.println("  ahrs accel_var <goodStdG> <badStdG> [save]");
    out.println("  ahrs gyro_gate <goodDps> <badDps> [save] | dt <minMs> <maxMs> [save]");
    out.println();
    out.println("[mag]");
    out.println("  mag status | enable [save] | disable [save]");
    out.println("  mag id | qmcstatus | regs | hub | fifo | processed | trust");
    out.println("  mag heading | heading ref | heading status | heading clear");
    out.println("  mag heading auto on | auto off | auto status");
    out.println("  mag yaw status | yaw reset | yaw enable [save] | yaw disable [save]");
    out.println("  mag yaw defaults [save] | tc <s> [save] | innovation <deg> [save]");
    out.println("  mag yaw gyro_gate <goodDps> <badDps> [save]");
    out.println("  mag yaw horiz_gate <bad> <good> [save]");
    out.println("  mag yaw accel_gate <bad> <good> [save]");
    out.println("  mag yaw age <ms> [save] | rate <deg_s> [save] | step <deg> [save]");
    out.println("  mag axis print | set <bodyX> <bodyY> <bodyZ> [save]");
    out.println("  mag axis identity [save] | clear [save]");
    out.println("  mag cal start | stop | reset | status | print | apply [save]");
    out.println();
    out.println("[network]");
    out.println("  net status | print | help");
    out.println("  net set ssid <ssid> [save] | set pass <password> [save] | clear pass [save]");
    out.println("  net set name <deviceName> [save] | set server <host> [port] [save]");
    out.println("  net discovery on|off [save] | enable [save] | disable [save]");
    out.println("  net save | load | defaults | erase | reconnect | counters reset | scan [visible|hidden] [limit N]");
    out.println("  slime status | start | stop | reconnect | counters reset");
    out.println();
    out.println("[output/log/test]");
    out.println("  stream off | heartbeat | raw | scaled | quat | debug");
    out.println("  stream rate <hz>");
    out.println("  log off | basic | full | start [basic|full] | stop | rate <hz> | header | summary | reset");
    out.println("  bias status | on | off | reset");
    out.println("  test static <seconds> | test stop | test status");
    out.println("  output mode debug | output rate <hz> | output start | output stop");
    out.println("  # SlimeVR UDP discovery is available through slime start/status.");
    out.println("  # RotationData output is not enabled yet.");
    out.println();
    out.println("[replay capture baseline]");
    out.println("  log full");
    out.println("  log rate 20");
    out.println("  log header");
    out.println("  test static 600");
    out.println("  log summary");
    out.println("  log off");
    out.println("==============================================================================");
}


static const char* yesNo(bool v) {
    return v ? "yes" : "no";
}

void trackerSerialPrintSetupStatus(TrackerSerialCommandContext& ctx) {
    Stream& out = trackerSerialSystemStream(ctx);
    out.println("# SETUP STATUS");

    const bool gyroReady = ctx.imuCal && ctx.imuCal->gyroBiasValid;
    const bool accelReady = ctx.imuCal && ctx.imuCal->accelCalValid;
    const bool configReady = ctx.config && ctx.config->validate();
    const bool magDriverEnabled = ctx.config && ctx.config->data.magCal.driverEnabled;
    const bool magCalReady = ctx.config && ctx.config->data.magCal.calibrationValid;
    const bool magAxisReady = ctx.config && ctx.config->data.magCal.axisAlignmentValid;
    const bool outputReady = ctx.config && ctx.config->data.output.packetFormat == 0;

    out.print("config_valid="); out.println(yesNo(configReady));
    out.print("gyro_bias_ready="); out.println(yesNo(gyroReady));
    out.print("accel_cal_ready="); out.println(yesNo(accelReady));
    out.print("mag_driver_enabled="); out.println(yesNo(magDriverEnabled));
    out.print("mag_cal_ready="); out.println(yesNo(magCalReady));
    out.print("mag_axis_ready="); out.println(yesNo(magAxisReady));
    out.print("output_backend_ready="); out.println(yesNo(outputReady));

    out.print("setup_ready_6dof="); out.println(yesNo(configReady && gyroReady && accelReady));
    out.print("setup_ready_mag_yaw="); out.println(yesNo(configReady && gyroReady && accelReady && magDriverEnabled && magCalReady && magAxisReady));

    if (!gyroReady) out.println("next: cal gyro; cal gyro save; config save");
    if (!accelReady) out.println("next: cal accel face XP/XN/YP/YN/ZP/ZN; cal accel compute; cal accel save; config save");
    if (magDriverEnabled && !magAxisReady) out.println("next: mag axis print; mag axis set <bodyX> <bodyY> <bodyZ> [save]");
    if (magDriverEnabled && !magCalReady) out.println("next: mag cal start; rotate tracker through many orientations; mag cal apply save; config save");
    if (!configReady) out.println("next: config print; config defaults/save if this is intentional");
    out.println("replay baseline: log full; log rate 20; log header; test static 600; log summary; log off");
}

void trackerSerialPrintStatus(TrackerSerialCommandContext& ctx) {
    Stream& out = trackerSerialSystemStream(ctx);
    out.println("# STATUS");

    if (ctx.printRuntimeStatus) {
        ctx.printRuntimeStatus(out, ctx.printRuntimeStatusUser);
        return;
    }

    if (ctx.lsm) {
        out.print("lsm_initialized="); out.println(ctx.lsm->isInitialized() ? "yes" : "no");
        out.print("lsm_last_error="); out.println(static_cast<int>(ctx.lsm->lastError()));
        out.print("lsm_who=0x"); out.println(ctx.lsm->lastWhoAmI(), HEX);
    }

    if (ctx.config) {
        out.print("config_valid="); out.println(ctx.config->validate() ? "yes" : "no");
    }

    if (ctx.imuCal) {
        out.print("gyro_bias_valid="); out.println(ctx.imuCal->gyroBiasValid ? "yes" : "no");
        out.print("accel_cal_valid="); out.println(ctx.imuCal->accelCalValid ? "yes" : "no");
    }

    if (ctx.fifo) {
        const auto& fs = ctx.fifo->stats();
        out.print("fifo_samples="); out.println(fs.imuSamplesProduced);
        out.print("fifo_hw_ts="); out.println(fs.hwTimestampAssigned);
        out.print("fifo_fb_ts="); out.println(fs.fallbackTimestampAssigned);
        out.print("fifo_ovr="); out.println(fs.overrunEvents);
        out.print("fifo_full="); out.println(fs.fullEvents);
        out.print("fifo_unknown="); out.println(fs.unknownWords);
    }

    if (ctx.streamState) {
        out.print("stream_mode="); out.println(trackerSerialSystemStreamModeName(ctx.streamState->mode));
        out.print("stream_rate_hz="); out.println(ctx.streamState->rateHz);
    }
}

void trackerSerialPrintHealth(TrackerSerialCommandContext& ctx) {
    Stream& out = trackerSerialSystemStream(ctx);
    out.println("# HEALTH");

    if (ctx.printRuntimeHealth) {
        ctx.printRuntimeHealth(out, ctx.printRuntimeHealthUser);
        return;
    }

    trackerSerialPrintStatus(ctx);
    if (ctx.fifo) trackerSerialPrintFifoStats(out, ctx.fifo->stats());
    if (ctx.quality) trackerSerialPrintQualityStats(out, ctx.quality->counters());
}

void trackerSerialFactoryReset(TrackerSerialCommandContext& ctx) {
    Stream& out = trackerSerialSystemStream(ctx);
    bool ok = true;

    if (ctx.config) {
        ctx.config->resetDefaults();
    }
    if (ctx.configStore) {
        ok = ctx.configStore->erase();
    }

    if (ok) tracker_serial_detail::printOk(out, "factory reset done; reboot recommended");
    else {
        out.print("# ERR factory reset failed: ");
        out.println(ctx.configStore ? ctx.configStore->lastErrorName() : "no config store");
    }
}

bool trackerSerialDispatchSystemCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    if (argc <= 0 || !argv || !argv[0]) return true;

    Stream& out = trackerSerialSystemStream(ctx);

    if (trackerSerialSystemIs(argv[0], "help") || trackerSerialSystemIs(argv[0], "?")) {
        trackerSerialPrintHelp(out);
        return true;
    }

    if (trackerSerialSystemIs(argv[0], "setup")) {
        if (argc >= 2 && trackerSerialSystemIs(argv[1], "status")) {
            trackerSerialPrintSetupStatus(ctx);
        } else {
            tracker_serial_detail::printErr(out, "usage: setup status");
        }
        return true;
    }

    if (trackerSerialSystemIs(argv[0], "status")) {
        trackerSerialPrintStatus(ctx);
        return true;
    }

    if (trackerSerialSystemIs(argv[0], "health")) {
        trackerSerialPrintHealth(ctx);
        return true;
    }

    if (trackerSerialSystemIs(argv[0], "version")) {
        out.println("# tracker firmware proto=serial-cli-v1");
        return true;
    }

    if (trackerSerialSystemIs(argv[0], "reboot")) {
        out.println("# OK rebooting");
        out.flush();
        delay(50);
        ESP.restart();
        return true;
    }

    if (trackerSerialSystemIs(argv[0], "factory_reset")) {
        trackerSerialFactoryReset(ctx);
        return true;
    }

    return false;
}

} // namespace tracker
