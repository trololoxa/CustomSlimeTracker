#include "serial/tracker_setup_commands.hpp"

#include "sensor/frame_transform.hpp"

#include <Arduino.h>
#include <cstdint>
#include <cstring>
#include <cmath>

#include "config/tracker_config.hpp"
#include "config/tracker_network_config.hpp"
#include "network/wifi_manager.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "runtime/gyro_temp_calibration_capture.hpp"
#include "runtime/runtime_bias_types.hpp"
#include "runtime/tap_runtime_controller.hpp"
#include "runtime/tracker_console_suppress.hpp"
#include "defines.h"
#include "sensor/accel_6pos_calibration.hpp"
#include "sensor/calibration.hpp"
#include "sensor/fifo_calibrations.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/mag_runtime.hpp"
#include "sensor/sensor_to_device_alignment.hpp"
#include "serial/tracker_calibration_commands.hpp"
#include "serial/tracker_config_commands.hpp"
#include "serial/tracker_mag_commands.hpp"
#include "serial/tracker_network_commands.hpp"
#include "serial/tracker_slimevr_commands.hpp"
#include "serial/tracker_serial_print.hpp"

namespace tracker {
namespace {

Stream& out(TrackerSerialCommandContext& ctx) {
    return ctx.io ? *ctx.io : Serial;
}

bool is(const char* a, const char* b) {
    return tracker_serial_detail::eqIgnoreCase(a, b);
}

const char* yesNo(bool v) {
    return v ? "yes" : "no";
}

const char* readyWord(bool v) {
    return v ? "ready" : "missing";
}

void copySetupCString(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    if (!src) src = "";

    size_t i = 0;
    for (; i + 1 < dstSize && src[i] != '\0'; ++i) {
        dst[i] = src[i];
    }
    dst[i] = '\0';
}

TrackerWifiManagerConfig makeSetupWifiManagerConfig(const TrackerNetworkConfig& net) {
    TrackerWifiManagerConfig cfg;
    cfg.enabled = net.data.wifiEnabled;
    cfg.credentialsValid = net.data.credentialsValid;
    cfg.ssid = net.data.ssid;
    cfg.password = net.data.password;
    cfg.hostname = net.data.deviceName;
    cfg.connectTimeoutMs = 15000;
    cfg.reconnectBackoffMs = 5000;
    cfg.statusPollIntervalMs = 250;
    return cfg;
}

void applySetupWifiConfig(TrackerSerialCommandContext& ctx) {
    if (!ctx.networkConfig) return;
    ctx.networkConfig->sanitize();
    if (ctx.wifiManager) {
        ctx.wifiManager->configure(makeSetupWifiManagerConfig(*ctx.networkConfig));
    }
}

bool saveSetupNetworkConfig(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    if (!ctx.networkConfig || !ctx.networkConfigStore) {
        tracker_serial_detail::printErr(s, "network config store not available");
        return false;
    }
    ctx.networkConfig->sanitize();
    if (!ctx.networkConfigStore->save(*ctx.networkConfig)) {
        s.print("# ERR setup wifi save failed: ");
        s.println(ctx.networkConfigStore->lastErrorName());
        return false;
    }
    if (ctx.networkConfigLoadedFromNvs) *ctx.networkConfigLoadedFromNvs = true;
    tracker_serial_detail::printOk(s, "Wi-Fi config saved to NVS");
    return true;
}

struct SetupReadiness {
    bool configValid = false;
    bool configLoaded = false;
    bool gyroReady = false;
    bool accelReady = false;
    bool frameReady = false;
    bool tempReady = false;
    bool tempEnabled = false;
    bool tempRangeValid = false;
    bool tempSoftExtrapolated = false;
    bool tempHardExtrapolated = false;
    float tempDistanceToRangeC = 0.0f;
    float tempExtrapolationConfidence = 1.0f;
    bool magDriver = false;
    bool magCal = false;
    bool magAxis = false;
    bool magYawApply = false;
    bool wifiConfigured = false;
    bool wifiEnabled = false;
    bool wifiConnected = false;
    bool slimeEnabled = false;
    bool slimeServerFound = false;
    bool runtimeBiasReady = false;
    bool runtimeBiasEnabled = false;
    bool tapReady = false;
    bool tapEnabled = false;
    bool tapHardwareConfigured = false;
    bool localOutputReady = false;
    bool runtimeWired = false;

    bool tracking6dof() const { return configValid && gyroReady && accelReady && frameReady; }
    bool tempQuality() const { return gyroReady && tempReady && !tempHardExtrapolated; }
    bool runtimeBias() const { return runtimeBiasReady && runtimeBiasEnabled; }
    bool magYaw() const { return tracking6dof() && magDriver && magCal && magAxis; }
    bool network() const { return wifiConfigured && wifiEnabled; }
    bool slimevr() const { return tracking6dof() && network() && localOutputReady && runtimeWired; }
    bool production() const { return slimevr() && tempQuality() && runtimeBias() && magYaw() && tapReady; }
};

SetupReadiness readSetupReadiness(TrackerSerialCommandContext& ctx) {
    SetupReadiness r;
    r.configValid = ctx.config && ctx.config->validate();
    r.configLoaded = r.configValid;
    r.gyroReady = ctx.imuCal && ctx.imuCal->gyroBiasValid;
    r.accelReady = ctx.imuCal && ctx.imuCal->accelCalValid;

    const float tempC = ctx.calibrationIo ? ctx.calibrationIo->latestTempC : 25.0f;
    if (ctx.gyroTempComp) {
        const GyroTempCompSnapshot s = ctx.gyroTempComp->snapshot(tempC);
        r.tempReady = s.valid && s.enabled && s.hasCalibratedRange && s.fitQuality > 0.0f;
        r.tempEnabled = s.enabled;
        r.tempRangeValid = s.hasCalibratedRange && !s.tempOutOfRange;
        r.tempSoftExtrapolated = s.tempSoftExtrapolated;
        r.tempHardExtrapolated = s.tempHardExtrapolated;
        r.tempDistanceToRangeC = s.tempDistanceToRangeC;
        r.tempExtrapolationConfidence = s.extrapolationConfidence;
    } else if (ctx.config) {
        r.tempReady = ctx.config->data.gyroCal.tempCompValid &&
                      ctx.config->data.gyroTempQuality.fitQuality > 0.0f;
        r.tempEnabled = ctx.config->data.gyroCal.tempCompEnabled;
        r.tempRangeValid = ctx.config->data.gyroTempQuality.tempRangeMaxC >
                           ctx.config->data.gyroTempQuality.tempRangeMinC;
        r.tempSoftExtrapolated = false;
        r.tempHardExtrapolated = false;
        r.tempDistanceToRangeC = 0.0f;
        r.tempExtrapolationConfidence = 1.0f;
    }

    if (ctx.config) {
        r.frameReady = makeSensorToDeviceFrame(
            ctx.config->data.frame.sensorToDeviceValid,
            ctx.config->data.frame.sensorToDevice
        ).enabled;
        r.magDriver = ctx.config->data.magCal.driverEnabled;
        r.magCal = ctx.config->data.magCal.calibrationValid;
        r.magAxis = ctx.config->data.magCal.axisAlignmentValid;
        r.magYawApply = ctx.config->data.magYaw.applyEnabled;
    }

    r.runtimeBiasReady = r.gyroReady && r.accelReady && r.tempReady;
    r.runtimeBiasEnabled = ctx.runtimeBias && ctx.runtimeBias->enabled;

    if (ctx.tapRuntime) {
        const TapRuntimeStatus tap = ctx.tapRuntime->status();
        r.tapEnabled = tap.enabled;
        r.tapHardwareConfigured = tap.hardwareConfigured;
        r.tapReady = tap.enabled && tap.hardwareConfigured;
    }

    if (ctx.networkConfig) {
        const auto& n = ctx.networkConfig->data;
        r.wifiConfigured = n.credentialsValid && n.ssid[0] != '\0';
        r.wifiEnabled = n.wifiEnabled;
    }

    if (ctx.wifiManager) {
        const TrackerWifiManagerStatus st = ctx.wifiManager->status();
        r.wifiConnected = st.connected;
    }

    if (ctx.slimevrRuntime) {
        const SlimeVROutputRuntimeStatus st = ctx.slimevrRuntime->status();
        r.slimeEnabled = st.enabled;
        r.slimeServerFound = st.serverFound;
    }

    r.localOutputReady = ctx.config != nullptr && ctx.streamState != nullptr;
    r.runtimeWired = ctx.networkConfig != nullptr && ctx.wifiManager != nullptr && ctx.slimevrRuntime != nullptr;
    return r;
}

bool setupStoredTempModelReady(TrackerSerialCommandContext& ctx) {
    const float tempC = ctx.calibrationIo ? ctx.calibrationIo->latestTempC : 25.0f;
    if (ctx.gyroTempComp) {
        const GyroTempCompSnapshot s = ctx.gyroTempComp->snapshot(tempC);
        // For resume/skip decisions we care whether a usable model exists, not
        // whether the current temperature is outside the calibrated range.
        return s.valid && s.enabled && s.hasCalibratedRange && s.fitQuality > 0.0f;
    }
    if (ctx.config) {
        return ctx.config->data.gyroCal.tempCompValid &&
               ctx.config->data.gyroCal.tempCompEnabled &&
               ctx.config->data.gyroTempQuality.fitQuality > 0.0f &&
               ctx.config->data.gyroTempQuality.tempRangeMaxC >
                   ctx.config->data.gyroTempQuality.tempRangeMinC;
    }
    return false;
}

void printStep(Stream& s, const char* name, bool ready, const char* next) {
    s.print(name);
    s.print('=');
    s.println(readyWord(ready));
    if (!ready && next && next[0]) {
        s.print("  next: ");
        s.println(next);
    }
}

void printSetupGuide(Stream& s) {
    s.println("# SETUP GUIDE");
    s.println("# New tracker path:");
    s.println("#   1) setup wifi");
    s.println("#   2) setup calibration [resume|full] [nomag|6dof] [axis <bodyX> <bodyY> <bodyZ>]");
    s.println("#   3) setup status");
    s.println();
    s.println("setup wifi");
    s.println("  Interactive Wi-Fi provisioning: scan, choose network, enter password,");
    s.println("  connect, save to NVS, start SlimeVR discovery and enable autostart.");
    s.println();
    s.println("setup calibration [resume|full] [nomag|6dof] [axis <bodyX> <bodyY> <bodyZ>]");
    s.println("  Default/resume mode skips already valid stages and saves each completed");
    s.println("  missing stage to NVS immediately, so a failed mag stage does not force");
    s.println("  another 15-minute temperature calibration on the next run.");
    s.println("  full mode intentionally recalibrates every stage transactionally.");
    s.println("  Blocking guided production calibration. It services FIFO, magnetometer,");
    s.println("  Wi-Fi and SlimeVR while it performs rest gyro, gyro temperature model,");
    s.println("  auto-detected accel 6-position, a two-orientation sensor-to-device frame stage,");
    s.println("  optional mag hard/soft collection, mag axis alignment and production tracking enable/save.");
    s.println("  The frame stage reuses the first two accel captures in full calibration; resume mode");
    s.println("  asks for only two short positions when accel calibration already exists.");
    s.println("  Use nomag/6dof for a dead or absent magnetometer.");
    s.println();
    s.println("setup status");
    s.println("  Readiness checklist for tracking, frame alignment, mag-yaw, temperature model and SlimeVR.");
    s.println();
    s.println("setup frame status|calibrate");
    s.println("  Inspect or repeat only the two-position sensor-to-device frame stage.");
    s.println();
    s.println("# Low-level net/cal/mag commands remain available for service diagnostics,");
    s.println("# but normal first-run setup should use the two commands above.");
}

void printSetupStatus(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    const SetupReadiness r = readSetupReadiness(ctx);

    s.println("# SETUP STATUS");
    s.print("production_ready="); s.println(yesNo(r.production()));
    s.print("tracking_6dof_ready="); s.println(yesNo(r.tracking6dof()));
    s.print("mag_yaw_ready="); s.println(yesNo(r.magYaw()));
    s.print("temp_model_ready="); s.println(yesNo(r.tempQuality()));
    s.print("slimevr_ready="); s.println(yesNo(r.slimevr()));
    s.println();

    printStep(s, "config", r.configValid, "config print; config save");
    printStep(s, "wifi", r.network(), "setup wifi");
    s.print("wifi_connected="); s.println(yesNo(r.wifiConnected));
    printStep(s, "rest_gyro", r.gyroReady, "setup calibration");
    printStep(s, "accel_6pos", r.accelReady, "setup calibration");
    printStep(s, "sensor_to_device", r.frameReady, "setup calibration");
    if (r.magDriver) {
        printStep(s, "mag_driver", r.magDriver, "setup calibration nomag");
        printStep(s, "mag_hard_soft", r.magCal, "setup calibration");
        printStep(s, "mag_axis", r.magAxis, "setup calibration axis <bodyX> <bodyY> <bodyZ>");
    } else {
        s.println("mag_driver=disabled");
        s.println("mag_hard_soft=disabled");
        s.println("mag_axis=disabled");
    }
    printStep(s, "temperature_model", r.tempQuality(), "setup calibration");
    printStep(s, "runtime_bias", r.runtimeBias(), "setup calibration");
    s.print("runtime_bias_enabled="); s.println(yesNo(r.runtimeBiasEnabled));
    printStep(s, "tap_input", r.tapReady, "tap status; tap on");
    s.print("tap_enabled="); s.println(yesNo(r.tapEnabled));
    s.print("tap_hardware_configured="); s.println(yesNo(r.tapHardwareConfigured));
    s.print("temperature_range_current=");
    if (!r.tempReady) s.println("missing");
    else if (r.tempHardExtrapolated) s.println("hard_extrapolated");
    else if (r.tempSoftExtrapolated) s.println("soft_extrapolated");
    else if (!r.tempRangeValid) s.println("outside");
    else s.println("in_range");
    s.print("temperature_range_distance_c="); s.println(r.tempDistanceToRangeC, 3);
    s.print("temperature_extrapolation_confidence="); s.println(r.tempExtrapolationConfidence, 3);
    printStep(s, "slimevr_runtime", r.slimevr(), "setup wifi; slime status");

    s.println();
    s.print("rest_calibration_sent_to_slimevr=");
    s.println(yesNo(r.gyroReady));
    s.print("slimevr_server_found="); s.println(yesNo(r.slimeServerFound));
    s.print("mag_yaw_apply_enabled="); s.println(yesNo(r.magYawApply));
    s.print("runtime_bias_enabled="); s.println(yesNo(r.runtimeBiasEnabled));

    if (!r.production()) {
        if (r.tracking6dof() && r.slimevr() && !r.magDriver) {
            s.println("# 6DoF setup is ready; mag is disabled/optional, so full production_ready remains no.");
        } else {
            s.println("# Use setup guide for the full first-run sequence.");
        }
    }
}

void dispatchNetwork(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    trackerSerialDispatchNetworkCommand(ctx, argc, argv);
}

void dispatchCal(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    trackerSerialDispatchCalibrationCommand(ctx, argc, argv);
}

void dispatchMag(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    trackerSerialDispatchMagCommand(ctx, argc, argv);
}

bool readSetupLine(TrackerSerialCommandContext& ctx, const char* prompt, char* buf, size_t cap, uint32_t timeoutMs);
bool serviceSetupRuntime(TrackerSerialCommandContext& ctx);

struct SetupCalibrationTransaction {
    TrackerConfig configSnapshot;
    ImuCalibration imuSnapshot;
    bool haveConfig = false;
    bool haveImu = false;
    RuntimeGyroBiasEstimator runtimeBiasSnapshot;
    bool haveRuntimeBias = false;
    bool originalMagDriverEnabled = false;
    bool originalMagYawApplyEnabled = false;
    bool committed = false;

    SetupCalibrationTransaction() = default;

    void begin(TrackerSerialCommandContext& ctx) {
        configSnapshot = TrackerConfig{};
        imuSnapshot = ImuCalibration{};
        runtimeBiasSnapshot = RuntimeGyroBiasEstimator{};
        haveConfig = false;
        haveImu = false;
        haveRuntimeBias = false;
        originalMagDriverEnabled = false;
        originalMagYawApplyEnabled = false;
        committed = false;

        if (ctx.config) {
            configSnapshot = *ctx.config;
            haveConfig = true;
            originalMagDriverEnabled = ctx.config->data.magCal.driverEnabled;
            originalMagYawApplyEnabled = ctx.config->data.magYaw.applyEnabled;
        }
        if (ctx.imuCal) {
            imuSnapshot = *ctx.imuCal;
            haveImu = true;
        }
        if (ctx.runtimeBias) {
            runtimeBiasSnapshot = *ctx.runtimeBias;
            haveRuntimeBias = true;
        }
    }

    void rollback(TrackerSerialCommandContext& ctx, const char* reason) {
        Stream& s = out(ctx);
        if (committed) return;

        if (ctx.stopMagCalibration) ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
        if (ctx.gyroTempCapture && ctx.gyroTempCapture->active()) ctx.gyroTempCapture->stop(millis());

        if (haveConfig && ctx.config) {
            *ctx.config = configSnapshot;
            ctx.config->sanitize();
            ctx.config->updateCrc();
        }
        if (haveImu && ctx.imuCal) {
            *ctx.imuCal = imuSnapshot;
        } else if (ctx.config && ctx.imuCal) {
            ctx.config->applyToImuCalibration(*ctx.imuCal);
        }
        if (ctx.config && ctx.gyroTempComp) {
            ctx.config->applyToGyroTempComp(*ctx.gyroTempComp);
        }

        if (ctx.accelCalRunner) ctx.accelCalRunner->reset();
        if (haveRuntimeBias && ctx.runtimeBias) {
            *ctx.runtimeBias = runtimeBiasSnapshot;
        } else if (ctx.resetRuntimeGyroBiasEstimator) {
            ctx.resetRuntimeGyroBiasEstimator(ctx.resetRuntimeGyroBiasEstimatorUser);
        }
        if (ctx.resetMagCalibration) ctx.resetMagCalibration(ctx.resetMagCalibrationUser);
        if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
        if (ctx.resetMagYawCorrection) ctx.resetMagYawCorrection(ctx.resetMagYawCorrectionUser);

        if (ctx.setMagRuntimeEnabled) {
            (void)ctx.setMagRuntimeEnabled(originalMagDriverEnabled, false, ctx.setMagRuntimeEnabledUser);
        }
        if (ctx.setMagYawCorrectionApplyEnabled) {
            (void)ctx.setMagYawCorrectionApplyEnabled(originalMagYawApplyEnabled, false, ctx.setMagYawCorrectionApplyEnabledUser);
        }
        if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();

        s.print("# SETUP CALIBRATION ROLLBACK");
        if (reason && reason[0]) {
            s.print(" reason=");
            s.print(reason);
        }
        s.println();
        s.println("# Previous RAM calibration/config restored; NVS was not changed by the failed setup calibration.");
    }

    bool commit(TrackerSerialCommandContext& ctx) {
        Stream& s = out(ctx);
        if (!ctx.config || !ctx.configStore) {
            tracker_serial_detail::printErr(s, "setup calibration commit failed: config store not available");
            return false;
        }
        trackerSerialCaptureRuntimeToConfig(ctx);
        ctx.config->sanitize();
        ctx.config->updateCrc();
        if (!ctx.configStore->save(*ctx.config)) {
            s.print("# ERR setup calibration commit save failed: ");
            s.println(ctx.configStore->lastErrorName());
            return false;
        }
        committed = true;
        tracker_serial_detail::printOk(s, "setup calibration transaction committed to NVS");
        if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();
        return true;
    }
};

// The guided calibration command runs inside Arduino's loopTask, whose stack is
// small on ESP32-C3.  Keep the heavy transaction snapshot and mag-axis sample
// buffers in static storage instead of the command stack frame.
static SetupCalibrationTransaction g_setupCalibrationTx;

struct SetupFrameObservations {
    bool topValid = false;
    bool forwardValid = false;
    Vec3 topScaledMeanG = Vec3::zero();
    Vec3 forwardScaledMeanG = Vec3::zero();

    void reset() {
        *this = SetupFrameObservations{};
    }

    bool complete() const {
        return topValid && forwardValid;
    }
};

static SetupFrameObservations g_setupFrameObservations;

struct SetupMagAxisFaceSample {
    bool valid = false;
    Accel6PosCalibration::Face face = Accel6PosCalibration::Face::Invalid;
    Vec3 accelMeanG = Vec3::zero();
    Vec3 magRawMean = Vec3::zero();
    uint32_t magSamples = 0;
};

struct SetupMagAxisAutoCollector {
    SetupMagAxisFaceSample samples[6];
    uint8_t count = 0;

    void reset() {
        for (auto& s : samples) s = SetupMagAxisFaceSample{};
        count = 0;
    }

    bool add(Accel6PosCalibration::Face face,
             const Vec3& accelMeanG,
             const Vec3& magRawMean,
             uint32_t magSamples) {
        const uint8_t idx = static_cast<uint8_t>(face);
        if (idx >= 6 || !accelMeanG.isFinite() || !magRawMean.isFinite() || magSamples == 0) return false;
        if (!samples[idx].valid) count++;
        samples[idx].valid = true;
        samples[idx].face = face;
        samples[idx].accelMeanG = accelMeanG;
        samples[idx].magRawMean = magRawMean;
        samples[idx].magSamples = magSamples;
        return true;
    }
};

static SetupMagAxisAutoCollector g_setupMagAxisAutoCollector;

struct SetupMagAxisAutoResult {
    bool valid = false;
    Mat3 magToImu = Mat3::identity();
    float score = 0.0f;
    float secondBestScore = 0.0f;
    float inclinationMean = 0.0f;
    float inclinationStddev = 0.0f;
    uint8_t usedSamples = 0;
};

struct SetupMagAxisDynamicInterval {
    Vec3 gyroImuRadS = Vec3::zero();
    Vec3 mag0Raw = Vec3::zero();
    Vec3 mag1Raw = Vec3::zero();
    float dtS = 0.0f;
};

struct SetupMagAxisDynamicResult {
    bool valid = false;
    Mat3 magToImu = Mat3::identity();
    float score = 0.0f;
    float secondBestScore = 0.0f;
    float meanDirectionError = 0.0f;
    float meanMagnitudeError = 0.0f;
    uint16_t usedIntervals = 0;
};

// The dynamic solver rejects candidates with fewer than ten usable
// gyro+mag intervals. Collect a little more than the hard minimum so the
// later quality gates can survive noisy intervals instead of falling through
// to the manual axis prompt.
constexpr uint16_t kSetupMagAxisDynamicSolverMinIntervals = 10;
constexpr uint16_t kSetupMagAxisDynamicTargetIntervals = 18;

struct SetupMagAxisDynamicCollector {
    static constexpr uint16_t kMaxIntervals = 160;

    SetupMagAxisDynamicInterval intervals[kMaxIntervals];
    uint16_t intervalCount = 0;
    uint32_t droppedIntervals = 0;

    uint32_t lastImuSeq = 0;
    uint32_t lastMagSeq = 0;
    Vec3 gyroSumRadS = Vec3::zero();
    uint16_t gyroSamples = 0;

    bool havePrevMag = false;
    Vec3 prevMagRaw = Vec3::zero();
    uint64_t prevMagUs = 0;

    float gyroNormMaxDps = 0.0f;
    uint32_t imuSamplesSeen = 0;
    uint32_t magSamplesSeen = 0;

    void reset() {
        *this = SetupMagAxisDynamicCollector{};
    }

    void pushInterval(const Vec3& gyroImuRadS, const Vec3& mag0Raw, const Vec3& mag1Raw, float dtS) {
        if (intervalCount >= kMaxIntervals) {
            droppedIntervals++;
            return;
        }
        intervals[intervalCount++] = SetupMagAxisDynamicInterval{gyroImuRadS, mag0Raw, mag1Raw, dtS};
    }

    void update(TrackerSerialCommandContext& ctx) {
        if (!ctx.lastCalibratedSample || !ctx.lastImuSampleSequence || !ctx.lastMagProcessed) return;

        const uint32_t imuSeq = *ctx.lastImuSampleSequence;
        if (imuSeq != 0u && imuSeq != lastImuSeq) {
            lastImuSeq = imuSeq;
            Vec3 gyro = ctx.lastCalibratedSample->gyro_rad_s;
            if (ctx.config) {
                const SensorToDeviceFrame frame = makeSensorToDeviceFrame(
                    ctx.config->data.frame.sensorToDeviceValid,
                    ctx.config->data.frame.sensorToDevice
                );
                gyro = frame.inverseApply(gyro);
            }
            const float gyroNormDps = gyro.norm() * MATH_RAD_TO_DEG;
            if (gyro.isFinite()) {
                imuSamplesSeen++;
                if (gyroNormDps > gyroNormMaxDps) gyroNormMaxDps = gyroNormDps;
                // The setup loop observes the latest pipeline sample, not every
                // FIFO sample.  Keep low-rate gyro accumulation permissive so a
                // 60 Hz mag interval can still produce a usable dynamic-axis
                // interval even when serviceSetupRuntime() is called at 5 ms cadence.
                if (gyroNormDps >= 2.0f && gyroNormDps <= 720.0f && gyroSamples < 2000u) {
                    gyroSumRadS += gyro;
                    gyroSamples++;
                }
            }
        }

        const MagProcessedSample& mag = *ctx.lastMagProcessed;
        if (mag.seq == 0u || mag.seq == lastMagSeq || !mag.raw.isFinite() || mag.rawNorm <= 1.0e-6f) {
            return;
        }
        lastMagSeq = mag.seq;
        magSamplesSeen++;

        if (havePrevMag && gyroSamples >= 1u) {
            float dtS = 0.0f;
            if (mag.t_us > prevMagUs && prevMagUs != 0u) {
                dtS = static_cast<float>(mag.t_us - prevMagUs) * 1.0e-6f;
            }
            if (dtS >= 0.004f && dtS <= 0.200f) {
                const Vec3 avgGyro = gyroSumRadS / static_cast<float>(gyroSamples);
                const Vec3 m0 = prevMagRaw.normalized();
                const Vec3 m1 = mag.raw.normalized();
                const float angle = std::acos(clampf(dot(m0, m1), -1.0f, 1.0f));
                const float gyroNormDps = avgGyro.norm() * MATH_RAD_TO_DEG;
                if (angle >= 0.0015f && gyroNormDps >= 3.0f && gyroNormDps <= 540.0f) {
                    pushInterval(avgGyro, prevMagRaw, mag.raw, dtS);
                }
            }
        }

        gyroSumRadS = Vec3::zero();
        gyroSamples = 0;
        prevMagRaw = mag.raw;
        prevMagUs = mag.t_us;
        havePrevMag = true;
    }
};

static SetupMagAxisDynamicCollector g_setupMagAxisDynamicCollector;

Vec3 normalizeSafe(const Vec3& v) {
    const float n = v.norm();
    if (!v.isFinite() || n <= 1.0e-6f) return Vec3::zero();
    return v / n;
}

float setupMagAxisScore(const SetupMagAxisAutoCollector& c,
                        const TrackerConfig& config,
                        const ImuCalibration& imuCal,
                        const Mat3& m,
                        float& meanOut,
                        float& stddevOut,
                        uint8_t& usedOut) {
    float values[6] = {};
    uint8_t used = 0;
    for (const auto& s : c.samples) {
        if (!s.valid) continue;
        const Vec3 accelBody = normalizeSafe(imuCal.accelCalValid ? imuCal.applyAccel(s.accelMeanG) : s.accelMeanG);
        const Vec3 magCal = config.data.magCal.softIron * (s.magRawMean - config.data.magCal.hardIron);
        const Vec3 magBody = normalizeSafe(m * magCal);
        if (!accelBody.isFinite() || !magBody.isFinite() || accelBody.norm() <= 1.0e-6f || magBody.norm() <= 1.0e-6f) continue;
        values[used++] = dot(accelBody, magBody);
    }

    usedOut = used;
    if (used < 4) {
        meanOut = 0.0f;
        stddevOut = 999.0f;
        return 999.0f;
    }

    float mean = 0.0f;
    for (uint8_t i = 0; i < used; ++i) mean += values[i];
    mean /= static_cast<float>(used);

    float var = 0.0f;
    for (uint8_t i = 0; i < used; ++i) {
        const float d = values[i] - mean;
        var += d * d;
    }
    var /= static_cast<float>(used);

    meanOut = mean;
    stddevOut = std::sqrt(var > 0.0f ? var : 0.0f);
    return stddevOut;
}

Mat3 setupPermutationMatrix(uint8_t ax0, float s0, uint8_t ax1, float s1, uint8_t ax2, float s2) {
    Mat3 m = Mat3::zero();
    m.m[0][ax0] = s0;
    m.m[1][ax1] = s1;
    m.m[2][ax2] = s2;
    return m;
}

bool setupAutoSolveMagAxis(const SetupMagAxisAutoCollector& c,
                           const TrackerConfig& config,
                           const ImuCalibration& imuCal,
                           SetupMagAxisAutoResult& result) {
    result = SetupMagAxisAutoResult{};
    if (c.count < 4 || !config.data.magCal.calibrationValid) return false;

    const uint8_t perms[6][3] = {
        {0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0},
    };
    float best = 999.0f;
    float second = 999.0f;
    Mat3 bestM = Mat3::identity();
    float bestMean = 0.0f;
    float bestStd = 999.0f;
    uint8_t bestUsed = 0;

    for (const auto& p : perms) {
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) {
                for (int sz = -1; sz <= 1; sz += 2) {
                    const Mat3 m = setupPermutationMatrix(p[0], static_cast<float>(sx),
                                                          p[1], static_cast<float>(sy),
                                                          p[2], static_cast<float>(sz));
                    float mean = 0.0f;
                    float stddev = 999.0f;
                    uint8_t used = 0;
                    const float score = setupMagAxisScore(c, config, imuCal, m, mean, stddev, used);
                    if (score < best) {
                        second = best;
                        best = score;
                        bestM = m;
                        bestMean = mean;
                        bestStd = stddev;
                        bestUsed = used;
                    } else if (score < second) {
                        second = score;
                    }
                }
            }
        }
    }

    result.magToImu = bestM;
    result.score = best;
    result.secondBestScore = second;
    result.inclinationMean = bestMean;
    result.inclinationStddev = bestStd;
    result.usedSamples = bestUsed;

    // Static accel-face samples constrain the magnetic inclination.  This is a
    // deliberate quality gate: when the result is ambiguous, do not enable mag yaw.
    const float separation = second - best;
    result.valid = bestUsed >= 4 && best < 0.12f && separation > 0.03f;
    return result.valid;
}

float setupDynamicAxisScore(const SetupMagAxisDynamicCollector& c,
                            const TrackerConfig& config,
                            const Mat3& candidate,
                            float& dirErrOut,
                            float& magErrOut,
                            uint16_t& usedOut) {
    float weightedErrorSum = 0.0f;
    float weightSum = 0.0f;
    float dirErrSum = 0.0f;
    float magErrSum = 0.0f;
    uint16_t used = 0;

    if (!config.data.magCal.calibrationValid) {
        dirErrOut = 999.0f;
        magErrOut = 999.0f;
        usedOut = 0;
        return 999.0f;
    }

    for (uint16_t i = 0; i < c.intervalCount; ++i) {
        const auto& in = c.intervals[i];
        if (!in.gyroImuRadS.isFinite() || !in.mag0Raw.isFinite() || !in.mag1Raw.isFinite() ||
            in.dtS <= 0.0f || !tracker::isFinite(in.dtS)) {
            continue;
        }

        const Vec3 mag0Cal = config.data.magCal.softIron * (in.mag0Raw - config.data.magCal.hardIron);
        const Vec3 mag1Cal = config.data.magCal.softIron * (in.mag1Raw - config.data.magCal.hardIron);
        Vec3 m0 = candidate * mag0Cal;
        Vec3 m1 = candidate * mag1Cal;
        if (!m0.normalizeInPlace() || !m1.normalizeInPlace()) continue;

        const Vec3 observed = (m1 - m0) / in.dtS;
        const Vec3 predicted = -cross(in.gyroImuRadS, m0);
        const float observedNorm = observed.norm();
        const float predictedNorm = predicted.norm();
        if (observedNorm < 0.02f || predictedNorm < 0.02f ||
            !tracker::isFinite(observedNorm) || !tracker::isFinite(predictedNorm)) {
            continue;
        }

        const float directionAgreement = clampf(dot(observed / observedNorm, predicted / predictedNorm), -1.0f, 1.0f);
        const float dirErr = 1.0f - directionAgreement;
        const float magErr = std::fabs(observedNorm - predictedNorm) / (predictedNorm + 0.05f);
        const float weight = clampf(predictedNorm, 0.05f, 4.0f);
        const float intervalScore = dirErr + 0.25f * clampf(magErr, 0.0f, 2.0f);

        weightedErrorSum += intervalScore * weight;
        weightSum += weight;
        dirErrSum += dirErr;
        magErrSum += magErr;
        used++;
    }

    usedOut = used;
    if (used < 10 || weightSum <= 0.0f) {
        dirErrOut = 999.0f;
        magErrOut = 999.0f;
        return 999.0f;
    }

    dirErrOut = dirErrSum / static_cast<float>(used);
    magErrOut = magErrSum / static_cast<float>(used);
    return weightedErrorSum / weightSum;
}

bool setupAutoSolveMagAxisDynamic(const SetupMagAxisDynamicCollector& c,
                                  const TrackerConfig& config,
                                  SetupMagAxisDynamicResult& result) {
    result = SetupMagAxisDynamicResult{};
    if (c.intervalCount < kSetupMagAxisDynamicSolverMinIntervals || !config.data.magCal.calibrationValid) return false;

    const uint8_t perms[6][3] = {
        {0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0},
    };

    float best = 999.0f;
    float second = 999.0f;
    Mat3 bestM = Mat3::identity();
    float bestDir = 999.0f;
    float bestMag = 999.0f;
    uint16_t bestUsed = 0;

    for (const auto& p : perms) {
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) {
                for (int sz = -1; sz <= 1; sz += 2) {
                    const Mat3 m = setupPermutationMatrix(p[0], static_cast<float>(sx),
                                                          p[1], static_cast<float>(sy),
                                                          p[2], static_cast<float>(sz));
                    float dir = 999.0f;
                    float mag = 999.0f;
                    uint16_t used = 0;
                    const float score = setupDynamicAxisScore(c, config, m, dir, mag, used);
                    if (score < best) {
                        second = best;
                        best = score;
                        bestM = m;
                        bestDir = dir;
                        bestMag = mag;
                        bestUsed = used;
                    } else if (score < second) {
                        second = score;
                    }
                }
            }
        }
    }

    result.magToImu = bestM;
    result.score = best;
    result.secondBestScore = second;
    result.meanDirectionError = bestDir;
    result.meanMagnitudeError = bestMag;
    result.usedIntervals = bestUsed;

    const float separation = second - best;
    result.valid = bestUsed >= kSetupMagAxisDynamicSolverMinIntervals && best < 0.75f && bestDir < 0.55f && separation > 0.08f;
    return result.valid;
}

bool setupAxisMatricesEqual(const Mat3& a, const Mat3& b) {
    for (uint8_t r = 0; r < 3; ++r) {
        for (uint8_t c = 0; c < 3; ++c) {
            if (std::fabs(a.m[r][c] - b.m[r][c]) > 0.25f) return false;
        }
    }
    return true;
}

void setupPrintMagAxisToken(Stream& s, const Mat3& m, uint8_t row) {
    uint8_t axis = 0;
    uint8_t nonZero = 0;
    float sign = 1.0f;
    for (uint8_t col = 0; col < 3; ++col) {
        const float v = m.m[row][col];
        if (std::fabs(v) > 0.5f) {
            nonZero++;
            axis = col;
            sign = v >= 0.0f ? 1.0f : -1.0f;
        }
    }
    if (nonZero != 1) {
        s.print("?");
        return;
    }
    s.print(sign >= 0.0f ? "+" : "-");
    s.print(axis == 0 ? "x" : (axis == 1 ? "y" : "z"));
}

void setupPrintMagAxisMapping(Stream& s, const Mat3& m) {
    setupPrintMagAxisToken(s, m, 0);
    s.print(' ');
    setupPrintMagAxisToken(s, m, 1);
    s.print(' ');
    setupPrintMagAxisToken(s, m, 2);
}

bool setupCaptureMagAxisFaceSample(TrackerSerialCommandContext& ctx,
                                   SetupMagAxisAutoCollector& collector,
                                   Accel6PosCalibration::Face face) {
    Stream& s = out(ctx);
    if (!ctx.lastMagProcessed || !ctx.accelCalRunner) return false;
    const auto& faceData = ctx.accelCalRunner->calibration().faceData(face);
    if (!faceData.valid) return false;

    Vec3 sum = Vec3::zero();
    uint32_t count = 0;
    uint32_t lastSeq = ctx.lastMagProcessed->seq;
    const uint32_t startMs = millis();
    while (millis() - startMs < 2500UL) {
        serviceSetupRuntime(ctx);
        const MagProcessedSample& mag = *ctx.lastMagProcessed;
        if (mag.seq != 0 && mag.seq != lastSeq && mag.raw.isFinite() && mag.rawNorm > 1.0e-6f) {
            lastSeq = mag.seq;
            sum += mag.raw;
            count++;
        }
        delay(5);
    }

    if (count < 3) {
        s.print("# WARN mag axis auto: too few mag samples for face ");
        s.println(Accel6PosCalibration::faceName(face));
        return false;
    }

    const Vec3 mean = sum / static_cast<float>(count);
    collector.add(face, faceData.meanG, mean, count);
    s.print("# setup mag axis auto face=");
    s.print(Accel6PosCalibration::faceName(face));
    s.print(" mag_samples=");
    s.println(count);
    return true;
}


void printSetupWifiList(Stream& s, const WifiScanResult* results, uint8_t count) {
    s.println("# idx rssi_dbm ch auth ssid");
    for (uint8_t i = 0; i < count; ++i) {
        const WifiScanResult& r = results[i];
        s.print(static_cast<unsigned int>(i + 1));
        s.print(' ');
        s.print(r.rssiDbm);
        s.print(' ');
        s.print(static_cast<unsigned int>(r.channel));
        s.print(' ');
        s.print(wifiAuthTypeName(r.authType));
        s.print(' ');
        s.println(r.ssid);
    }
}

bool setupWaitWifiConnected(TrackerSerialCommandContext& ctx, uint32_t timeoutMs) {
    Stream& s = out(ctx);
    const uint32_t startMs = millis();
    uint32_t lastPrintMs = 0;
    while (millis() - startMs < timeoutMs) {
        serviceSetupRuntime(ctx);
        if (ctx.wifiManager && ctx.wifiManager->connected()) return true;
        const uint32_t nowMs = millis();
        if (nowMs - lastPrintMs >= 1000UL) {
            lastPrintMs = nowMs;
            if (ctx.wifiManager) {
                const TrackerWifiManagerStatus st = ctx.wifiManager->status();
                s.print("# setup wifi connecting state=");
                s.print(trackerWifiStateName(st.state));
                s.print(" link=");
                s.print(wifiLinkStatusName(st.linkStatus));
                s.print(" elapsed_s=");
                s.println((nowMs - startMs) / 1000UL);
            }
        }
        delay(5);
    }
    return ctx.wifiManager && ctx.wifiManager->connected();
}

bool setupWaitSlimeServerFound(TrackerSerialCommandContext& ctx, uint32_t timeoutMs) {
    Stream& s = out(ctx);
    const uint32_t startMs = millis();
    uint32_t lastPrintMs = 0;
    while (millis() - startMs < timeoutMs) {
        serviceSetupRuntime(ctx);
        if (ctx.slimevrRuntime && ctx.slimevrRuntime->serverFound()) return true;
        const uint32_t nowMs = millis();
        if (nowMs - lastPrintMs >= 2000UL) {
            lastPrintMs = nowMs;
            if (ctx.slimevrRuntime) {
                const SlimeVROutputRuntimeStatus st = ctx.slimevrRuntime->status();
                s.print("# setup wifi slimevr state=");
                s.print(slimevrOutputStateName(st.state));
                s.print(" discovery_responses=");
                s.print(st.discoveryResponses);
                s.print(" elapsed_s=");
                s.println((nowMs - startMs) / 1000UL);
            }
        }
        delay(5);
    }
    return ctx.slimevrRuntime && ctx.slimevrRuntime->serverFound();
}

void setupStartSlimeRuntime(TrackerSerialCommandContext& ctx) {
    char* start[] = { const_cast<char*>("slime"), const_cast<char*>("start") };
    trackerSerialDispatchSlimeVRCommand(ctx, 2, start);
}

void cmdSetupWifi(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& s = out(ctx);
    if (argc >= 3 && (is(argv[2], "help") || is(argv[2], "?"))) {
        s.println("setup wifi");
        s.println("  Interactive Wi-Fi provisioning: scan, choose SSID, enter password,");
        s.println("  connect, save to NVS, start SlimeVR discovery and enable autostart.");
        return;
    }
    if (argc >= 3) {
        tracker_serial_detail::printErr(s, "usage: setup wifi  (interactive); use low-level net/slime commands for manual diagnostics");
        return;
    }

    if (!ctx.networkConfig || !ctx.networkConfigStore || !ctx.wifiManager) {
        tracker_serial_detail::printErr(s, "setup wifi failed: network runtime/config is not available");
        return;
    }

    static WifiScanResult results[16];
    s.println("# SETUP WIFI");
    s.println("# Scanning visible 2.4 GHz networks. Keep SlimeVR Server open on the same LAN.");
    const int16_t seen = ctx.wifiManager->scanNetworks(results, 16, false);
    trackerConsoleSuppressTrackingMessagesFor(TRACKER_SERIAL_COMMAND_RECOVERY_SUPPRESS_MS);
    if (seen < 0) {
        s.print("# ERR setup wifi scan failed: ");
        s.println(seen);
        ctx.wifiManager->clearScanResults();
        return;
    }

    const uint8_t shown = static_cast<uint8_t>(seen < 16 ? seen : 16);
    if (shown == 0) {
        ctx.wifiManager->clearScanResults();
        tracker_serial_detail::printErr(s, "no visible Wi-Fi networks found; use low-level net set commands for hidden SSIDs");
        return;
    }
    printSetupWifiList(s, results, shown);

    char line[96] = {};
    uint32_t choice = 0;
    while (choice == 0 || choice > shown) {
        if (!readSetupLine(ctx, "# Choose network number, or q to abort:", line, sizeof(line), 120000UL)) {
            ctx.wifiManager->clearScanResults();
            tracker_serial_detail::printErr(s, "setup wifi aborted: selection timeout");
            return;
        }
        if (is(line, "q") || is(line, "quit") || is(line, "abort")) {
            ctx.wifiManager->clearScanResults();
            tracker_serial_detail::printErr(s, "setup wifi aborted");
            return;
        }
        if (!tracker_serial_detail::parseU32(line, choice) || choice == 0 || choice > shown) {
            choice = 0;
            s.println("# Please enter a valid number from the list.");
        }
    }

    const WifiScanResult selected = results[choice - 1];
    ctx.wifiManager->clearScanResults();
    if (selected.ssid[0] == '\0') {
        tracker_serial_detail::printErr(s, "selected network has an empty/hidden SSID; use low-level net set commands");
        return;
    }

    s.print("# Selected SSID: ");
    s.println(selected.ssid);
    s.println("# Enter Wi-Fi password. Leave blank only for an open network.");
    char password[65] = {};
    if (!readSetupLine(ctx, "# password> ", password, sizeof(password), 180000UL)) {
        tracker_serial_detail::printErr(s, "setup wifi aborted: password timeout");
        return;
    }
    if (selected.authType != WifiAuthType::Open && password[0] == '\0') {
        tracker_serial_detail::printErr(s, "password is required for the selected secured network");
        return;
    }

    TrackerNetworkConfig oldConfig = *ctx.networkConfig;
    copySetupCString(ctx.networkConfig->data.ssid, sizeof(ctx.networkConfig->data.ssid), selected.ssid);
    copySetupCString(ctx.networkConfig->data.password, sizeof(ctx.networkConfig->data.password), password);
    ctx.networkConfig->data.credentialsValid = true;
    ctx.networkConfig->data.wifiEnabled = true;
    ctx.networkConfig->data.discoveryEnabled = true;
    ctx.networkConfig->sanitize();

    s.println("# Trying Wi-Fi connection. Credentials are not saved until connection succeeds.");
    ctx.wifiManager->reset();
    applySetupWifiConfig(ctx);
    if (!setupWaitWifiConnected(ctx, 30000UL)) {
        *ctx.networkConfig = oldConfig;
        ctx.networkConfig->sanitize();
        ctx.wifiManager->reset();
        applySetupWifiConfig(ctx);
        tracker_serial_detail::printErr(s, "setup wifi failed: could not connect; old network config restored in RAM and NVS was not changed");
        return;
    }

    tracker_serial_detail::printOk(s, "Wi-Fi connected");
    if (!saveSetupNetworkConfig(ctx)) {
        s.println("# WARN Wi-Fi works in RAM, but autostart was not persisted");
        return;
    }

    s.println("# Starting SlimeVR UDP discovery. Make sure SlimeVR Server is running on the same network.");
    setupStartSlimeRuntime(ctx);
    if (setupWaitSlimeServerFound(ctx, 30000UL)) {
        tracker_serial_detail::printOk(s, "SlimeVR Server found; Wi-Fi/SlimeVR autostart is enabled");
    } else {
        s.println("# WARN SlimeVR Server was not found within 30s.");
        s.println("# Wi-Fi was saved and autostart remains enabled; discovery will continue during normal runtime.");
        s.println("# Check that SlimeVR Server is running, firewall allows UDP 6969, and PC/tracker are on the same LAN.");
    }

    printSetupStatus(ctx);
}


bool serviceSetupRuntime(TrackerSerialCommandContext& ctx) {
    if (ctx.serviceNonCliRuntime) {
        return ctx.serviceNonCliRuntime(ctx.serviceNonCliRuntimeUser);
    }
    delay(5);
    return true;
}

void drainSetupInput(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    while (s.available() > 0) {
        (void)s.read();
        serviceSetupRuntime(ctx);
    }
}

bool readSetupLine(TrackerSerialCommandContext& ctx,
                   const char* prompt,
                   char* buf,
                   size_t cap,
                   uint32_t timeoutMs) {
    Stream& s = out(ctx);
    if (!buf || cap == 0) return false;
    buf[0] = '\0';

    drainSetupInput(ctx);
    if (prompt && prompt[0]) s.println(prompt);

    const uint32_t startMs = millis();
    size_t n = 0;
    while (millis() - startMs < timeoutMs) {
        serviceSetupRuntime(ctx);
        while (s.available() > 0) {
            const int c = s.read();
            if (c < 0) break;
            if (c == '\r') continue;
            if (c == '\n') {
                buf[n] = '\0';
                return true;
            }
            if (n + 1 < cap) {
                buf[n++] = static_cast<char>(c);
            }
        }
        delay(5);
    }

    buf[n] = '\0';
    return false;
}

bool waitSetupEnter(TrackerSerialCommandContext& ctx, const char* prompt, uint32_t timeoutMs) {
    char line[8];
    return readSetupLine(ctx, prompt, line, sizeof(line), timeoutMs);
}

bool setupPrepareCalibrationRuntime(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);

    // A full setup calibration must measure the physical gyro bias/temperature
    // model, not the residual left after the previous runtime trim.  Pause and
    // clear the transient runtime-bias estimator before collecting rest/temp
    // samples.  The transaction snapshot restores it on failure; successful
    // setup re-enables it after the new calibrated base model is committed.
    if (ctx.runtimeBias) {
        ctx.runtimeBias->enabled = false;
        ctx.runtimeBias->runtimeTrimRadS = Vec3::zero();
        ctx.runtimeBias->resetCounters();
        s.println("# setup calibration: runtime gyro bias estimator paused and transient trim cleared");
        return true;
    }

    if (ctx.setRuntimeGyroBiasEnabled) {
        (void)ctx.setRuntimeGyroBiasEnabled(false, ctx.setRuntimeGyroBiasEnabledUser);
    }
    if (ctx.resetRuntimeGyroBiasEstimator) {
        ctx.resetRuntimeGyroBiasEstimator(ctx.resetRuntimeGyroBiasEstimatorUser);
        s.println("# setup calibration: runtime gyro bias estimator reset through hook");
    }
    return true;
}

bool setupMaybeStartWifiHeating(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    if (!ctx.networkConfig) {
        s.println("# WARN Wi-Fi config is not wired; temperature calibration will use IMU/self heating only");
        return true;
    }

    const auto& n = ctx.networkConfig->data;
    if (!n.credentialsValid || n.ssid[0] == '\0') {
        s.println("# WARN Wi-Fi credentials are missing; run setup wifi before production calibration");
        s.println("# WARN continuing without Wi-Fi heat source; temp fit may fail if temperature range is too small");
        return true;
    }

    if (ctx.wifiManager && ctx.wifiManager->connected()) {
        // Do not force a reconnect here. A remote TCP console user is connected
        // through the same Wi-Fi link, and the old unconditional `net reconnect`
        // dropped telnet/nc at the beginning of the temperature stage.
        s.println("# setup calibration: Wi-Fi already connected; keeping current link for heat source");
        return true;
    }

    s.println("# setup calibration: enabling Wi-Fi during static warm-up for realistic tracker heating");
    char* enable[] = { const_cast<char*>("net"), const_cast<char*>("enable") };
    dispatchNetwork(ctx, 2, enable);
    char* reconnect[] = { const_cast<char*>("net"), const_cast<char*>("reconnect") };
    dispatchNetwork(ctx, 2, reconnect);
    return true;
}

bool setupRunRestGyro(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    s.println();
    s.println("# SETUP CALIBRATION STEP 1/7: REST/GYRO");
    s.println("# Put the tracker on a stable surface and do not touch it.");
    if (!waitSetupEnter(ctx, "# Press Enter when the tracker is completely still.", 120000UL)) {
        tracker_serial_detail::printErr(s, "setup calibration aborted: rest confirmation timeout");
        return false;
    }

    char* gyro[] = { const_cast<char*>("cal"), const_cast<char*>("gyro") };
    dispatchCal(ctx, 2, gyro);
    if (!ctx.imuCal || !ctx.imuCal->gyroBiasValid) {
        tracker_serial_detail::printErr(s, "setup calibration failed: gyro/rest calibration is not valid");
        return false;
    }
    if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();
    return true;
}

bool setupRunTemperatureFit(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    s.println();
    s.println("# SETUP CALIBRATION STEP 2/7: GYRO TEMPERATURE MODEL");
    s.println("# Keep the tracker on a normal stable surface. The firmware stops at a relative temperature plateau.");
    s.println("# Brief touches pause collection and discard only the current short stationary window; accepted progress is kept.");
    s.println("# This uses a dedicated setup temperature capture, not the developer test static runner.");

    if (!ctx.gyroTempCapture || !ctx.fitGyroTempFromCaptureRam) {
        tracker_serial_detail::printErr(s, "setup calibration failed: gyro temperature capture RAM-fit hook is not available");
        return false;
    }

    setupMaybeStartWifiHeating(ctx);

    constexpr uint32_t kMinMs = 180000UL;
    constexpr uint32_t kMaxMs = 900000UL;
    constexpr uint32_t kPlateauWindowMs = 60000UL;
    constexpr float kPlateauDeltaC = 0.15f;
    constexpr float kMinTempRangeC = 3.0f;

    ctx.gyroTempCapture->start(millis(), kMaxMs);
    const uint32_t startMs = millis();
    float minTemp = ctx.calibrationIo ? ctx.calibrationIo->latestTempC : 25.0f;
    float maxTemp = minTemp;
    float windowStartTemp = minTemp;
    uint32_t windowStartMs = startMs;
    uint32_t lastPrintMs = 0;
    bool plateau = false;

    while (ctx.gyroTempCapture->active()) {
        serviceSetupRuntime(ctx);
        const uint32_t nowMs = millis();
        const float t = ctx.calibrationIo ? ctx.calibrationIo->latestTempC : minTemp;
        if (t < minTemp) minTemp = t;
        if (t > maxTemp) maxTemp = t;

        if (nowMs - lastPrintMs >= 5000UL) {
            lastPrintMs = nowMs;
            s.print("# setup temp elapsed_s="); s.print((nowMs - startMs) / 1000UL);
            s.print(" temp_c="); s.print(t, 3);
            s.print(" range_c="); s.print(maxTemp - minTemp, 3);
            const GyroTempCalibrationCaptureDiagnostics& d = ctx.gyroTempCapture->diagnostics();
            s.print(" usable_bins="); s.print(ctx.gyroTempCapture->usableTempBins());
            s.print(" samples="); s.print(ctx.gyroTempCapture->capture().samples);
            s.print(" accepted="); s.print(d.acceptedSamples);
            s.print(" rejected="); s.print(d.rejectedSamples);
            s.print(" window="); s.print(d.currentWindowSamples);
            s.print(" motion_resets="); s.println(d.motionWindowResets);
        }

        if (nowMs - windowStartMs >= kPlateauWindowMs) {
            const float windowDelta = std::fabs(t - windowStartTemp);
            const float totalRange = maxTemp - minTemp;
            if ((nowMs - startMs) >= kMinMs && totalRange >= kMinTempRangeC && windowDelta <= kPlateauDeltaC) {
                plateau = true;
                break;
            }
            windowStartMs = nowMs;
            windowStartTemp = t;
        }
        delay(5);
    }

    if (ctx.gyroTempCapture->active()) {
        ctx.gyroTempCapture->stop(millis());
    }

    if (plateau) s.println("# setup temp: relative plateau detected; stopping capture");
    else s.println("# setup temp: max capture duration reached; trying fit with collected data");

    const StaticRuntimeTest& capture = ctx.gyroTempCapture->capture();
    const GyroTempCalibrationCaptureDiagnostics& captureDiag = ctx.gyroTempCapture->diagnostics();
    s.print("# setup temp accepted_samples="); s.println(captureDiag.acceptedSamples);
    s.print("# setup temp rejected_samples="); s.println(captureDiag.rejectedSamples);
    s.print("# setup temp accepted_windows="); s.println(captureDiag.acceptedWindows);
    s.print("# setup temp rejected_windows="); s.println(captureDiag.rejectedWindows);
    s.print("# setup temp motion_window_resets="); s.println(captureDiag.motionWindowResets);
    if (!ctx.fitGyroTempFromCaptureRam(&capture, s, ctx.fitGyroTempFromCaptureRamUser)) {
        tracker_serial_detail::printErr(s, "setup calibration failed: gyro temperature fit did not pass quality gates");
        s.println("# TIP: repeat setup calibration after a larger cold-to-warm temperature change");
        return false;
    }
    return true;
}

bool setupRunAccelFacesWithMagCollection(TrackerSerialCommandContext& ctx,
                                         SetupMagAxisAutoCollector& axisAuto,
                                         SetupFrameObservations& frameObservations,
                                         bool collectMagDuringAccel) {
    Stream& s = out(ctx);
    s.println();
    s.println(collectMagDuringAccel
        ? "# SETUP CALIBRATION STEP 3/7: ACCEL 6-POS + FRAME + MAG COLLECTION"
        : "# SETUP CALIBRATION STEP 3/7: ACCEL 6-POS + FRAME");
    s.println("# Device axes used by firmware: +X right, +Y forward, +Z top/outward.");
    s.println("# Use the same physical +Y edge on every tracker; the USB-connector edge is the recommended default.");
    s.println("# The first two captures identify the case frame and also count toward the normal six accel sides.");
    s.println("# You do NOT need to know the IMU axis labels.");
    s.println("# After the first two captures, place the tracker on any uncaptured physical side, let it fully settle, then press Enter.");
    s.println("# The firmware waits for a contiguous still window, detects which accel side is up, and rejects duplicates/diagonal positions.");
    s.println("# Slight IMU solder/board misalignment is handled later by the full 3x3 accel correction matrix.");

    bool magCollectionActive = false;
    if (collectMagDuringAccel) {
        if (ctx.setMagRuntimeEnabled) {
            (void)ctx.setMagRuntimeEnabled(true, false, ctx.setMagRuntimeEnabledUser);
        }
        if (ctx.config) {
            ctx.config->data.magCal.driverEnabled = true;
            ctx.config->updateCrc();
        }
        if (ctx.resetMagCalibration) ctx.resetMagCalibration(ctx.resetMagCalibrationUser);
        if (!ctx.startMagCalibration || !ctx.startMagCalibration(ctx.startMagCalibrationUser)) {
            tracker_serial_detail::printErr(s, "setup calibration failed: could not start magnetometer calibration");
            return false;
        }
        magCollectionActive = true;
    }

    char* clearAccel[] = { const_cast<char*>("cal"), const_cast<char*>("accel"), const_cast<char*>("clear") };
    dispatchCal(ctx, 3, clearAccel);
    frameObservations.reset();

    uint8_t captured = 0;
    uint8_t attempts = 0;
    while (captured < 6 && attempts < 18) {
        attempts++;
        s.println();
        s.print("# Accel face ");
        s.print(static_cast<unsigned int>(captured + 1));
        s.println("/6");
        s.print("# already captured:");
        bool any = false;
        if (ctx.accelCalRunner) {
            for (uint8_t i = 0; i < 6; ++i) {
                const auto face = static_cast<Accel6PosCalibration::Face>(i);
                if (ctx.accelCalRunner->calibration().hasFace(face)) {
                    s.print(' ');
                    s.print(Accel6PosCalibration::faceName(face));
                    any = true;
                }
            }
        }
        if (!any) s.print(" none");
        s.println();

        const char* capturePrompt = "# Move to a NEW side, wait until it stops wobbling, then press Enter.";
        if (captured == 0) {
            s.println("# FRAME TOP: place the tracker flat with its top/outward face pointing upward (+Z up).");
            capturePrompt = "# Wait until the tracker stops wobbling, then press Enter for frame top/+Z.";
        } else if (captured == 1) {
            s.println("# FRAME FORWARD: stand the tracker so its chosen +Y/forward edge points straight upward.");
            capturePrompt = "# Wait until the tracker stops wobbling, then press Enter for frame forward/+Y.";
        }
        if (!waitSetupEnter(ctx, capturePrompt, 300000UL)) {
            if (magCollectionActive && ctx.stopMagCalibration) ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
            tracker_serial_detail::printErr(s, "setup calibration aborted while waiting for accel face");
            return false;
        }

        FifoAccelAutoFaceCaptureResult faceResult;
        if (!ctx.accelCalRunner ||
            !ctx.accelCalRunner->captureAutoFace(*ctx.calibrationIo, faceResult, nullptr, nullptr)) {
            if (faceResult.duplicate) {
                s.print("# WARN duplicate accel face detected: ");
                s.println(Accel6PosCalibration::faceName(faceResult.detectedFace));
                s.println("# Keep the tracker on a different physical side for the next capture.");
                continue;
            }
            if (faceResult.ambiguous) {
                s.println("# WARN accel face was ambiguous/diagonal or not settled enough.");
                s.print("# detection norm_g="); s.print(faceResult.detection.normG, 4);
                s.print(" dominant_abs_g="); s.print(faceResult.detection.dominantAbsG, 4);
                s.print(" second_abs_g="); s.print(faceResult.detection.secondAbsG, 4);
                s.print(" margin_g="); s.println(faceResult.detection.dominanceMarginG, 4);
                s.println("# Put the tracker flat on one side, avoid holding it in your hand, and retry.");
                continue;
            }
            if (magCollectionActive && ctx.stopMagCalibration) ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
            tracker_serial_detail::printErr(s, "setup calibration failed: auto accel face capture failed");
            return false;
        }

        if (captured == 0) {
            frameObservations.topScaledMeanG = faceResult.meanG;
            frameObservations.topValid = true;
        } else if (captured == 1) {
            frameObservations.forwardScaledMeanG = faceResult.meanG;
            frameObservations.forwardValid = true;
        }
        captured++;
        s.print("# setup accel auto_face=");
        s.print(Accel6PosCalibration::faceName(faceResult.detectedFace));
        s.print(" samples="); s.print(faceResult.acceptedSamples);
        s.print(" rejected="); s.print(faceResult.rejectedSamples);
        s.print(" norm_g="); s.print(faceResult.meanNormG, 5);
        s.print(" dominance_margin_g="); s.println(faceResult.detection.dominanceMarginG, 5);

        if (collectMagDuringAccel) {
            (void)setupCaptureMagAxisFaceSample(ctx, axisAuto, faceResult.detectedFace);
        }
    }

    if (captured < 6) {
        if (magCollectionActive && ctx.stopMagCalibration) ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
        tracker_serial_detail::printErr(s, "setup calibration failed: could not collect six unique accel faces");
        return false;
    }

    char* compute[] = { const_cast<char*>("cal"), const_cast<char*>("accel"), const_cast<char*>("compute") };
    dispatchCal(ctx, 3, compute);
    if (!ctx.imuCal || !ctx.imuCal->accelCalValid) {
        if (magCollectionActive && ctx.stopMagCalibration) ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
        tracker_serial_detail::printErr(s, "setup calibration failed: accel 6-position quality gates rejected the result");
        return false;
    }
    return true;
}

bool setupApplySensorToDeviceAlignment(TrackerSerialCommandContext& ctx,
                                       const Vec3& topScaledSensorG,
                                       const Vec3& forwardScaledSensorG,
                                       bool separateAccelRotation) {
    Stream& s = out(ctx);
    if (!ctx.config || !ctx.imuCal || !ctx.imuCal->accelCalValid) {
        tracker_serial_detail::printErr(s, "setup frame failed: config or accel calibration is not available");
        return false;
    }

    SensorToDeviceAlignmentResult alignment;
    if (separateAccelRotation) {
        const SensorToDeviceCalibrationSeparationResult separated =
            separateSensorToDeviceFromAccelCalibration(
                topScaledSensorG,
                forwardScaledSensorG,
                ctx.imuCal->accelBiasG,
                ctx.imuCal->accelScale
            );
        s.print("# frame accel_rotation_det="); s.println(separated.extractedAccelRotation.determinant(), 6);
        s.print("# frame accel_reconstruction_error="); s.println(separated.reconstructionError, 8);
        if (!separated.valid) {
            tracker_serial_detail::printErr(s, "setup frame failed: accel calibration could not be separated from board rotation");
            return false;
        }
        ctx.imuCal->accelScale = separated.accelScaleSensorFrame;
        alignment = separated.alignment;
    } else {
        alignment = solveSensorToDeviceAlignment(
            ctx.imuCal->applyAccel(topScaledSensorG),
            ctx.imuCal->applyAccel(forwardScaledSensorG)
        );
    }

    s.print("# frame top_norm_g="); s.println(alignment.topNormG, 6);
    s.print("# frame forward_norm_g="); s.println(alignment.forwardNormG, 6);
    s.print("# frame observation_separation_deg="); s.println(alignment.observationSeparationDeg, 3);
    s.print("# frame mapped_top_error_deg="); s.println(alignment.mappedTopErrorDeg, 3);
    s.print("# frame mapped_forward_error_deg="); s.println(alignment.mappedForwardErrorDeg, 3);
    s.print("# frame determinant="); s.println(alignment.determinant, 6);
    if (!alignment.valid) {
        tracker_serial_detail::printErr(s, "setup frame failed: top and forward observations were not stable and perpendicular enough");
        s.println("# TIP: keep top/+Z and forward/+Y positions distinct; do not hold the tracker diagonally.");
        return false;
    }

    ctx.config->data.frame.sensorToDevice = alignment.sensorToDevice;
    ctx.config->data.frame.sensorToDeviceValid = true;
    ctx.config->updateCrc();
    if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
    tracker_serial_detail::printOk(s, "sensor-to-device frame aligned in RAM");
    return true;
}

bool setupRunSensorToDeviceAlignmentFromAccel(TrackerSerialCommandContext& ctx,
                                               const SetupFrameObservations& observations) {
    Stream& s = out(ctx);
    s.println();
    s.println("# SETUP CALIBRATION STEP 4/7: SENSOR-TO-DEVICE FRAME");
    s.println("# Reusing the top/+Z and forward/+Y observations already captured during accel 6-position calibration.");
    if (!ctx.imuCal || !ctx.imuCal->accelCalValid || !observations.complete()) {
        tracker_serial_detail::printErr(s, "setup frame failed: accel calibration or frame observations are missing");
        return false;
    }
    return setupApplySensorToDeviceAlignment(
        ctx,
        observations.topScaledMeanG,
        observations.forwardScaledMeanG,
        true
    );
}

bool setupCaptureSensorFrameObservation(TrackerSerialCommandContext& ctx,
                                        const char* instruction,
                                        const char* prompt,
                                        Vec3& meanAccelG) {
    Stream& s = out(ctx);
    if (!ctx.lastScaledSample || !ctx.lastImuSampleSequence || !ctx.imuCal || !ctx.imuCal->accelCalValid) {
        tracker_serial_detail::printErr(s, "setup frame failed: latest sensor sample or accel calibration is unavailable");
        return false;
    }

    s.println(instruction);
    if (!waitSetupEnter(ctx, prompt, 300000UL)) {
        tracker_serial_detail::printErr(s, "setup frame aborted while waiting for position confirmation");
        return false;
    }

    SensorToDeviceObservationCapture capture;
    uint32_t lastSeq = *ctx.lastImuSampleSequence;
    const uint32_t startMs = millis();
    uint32_t lastPrintMs = 0;
    while (!capture.complete() && millis() - startMs < 60000UL) {
        serviceSetupRuntime(ctx);
        const uint32_t seq = *ctx.lastImuSampleSequence;
        if (seq != 0u && seq != lastSeq) {
            lastSeq = seq;
            Lsm6dsv::Sample sensorSample = *ctx.lastScaledSample;
            if (ctx.imuCal->gyroBiasValid) sensorSample.gyro_rad_s = ctx.imuCal->applyGyro(sensorSample.gyro_rad_s);
            const Vec3 sourceAccelG = sensorSample.accel_g;
            sensorSample.accel_g = ctx.imuCal->applyAccel(sourceAccelG);
            (void)capture.push(sensorSample, sourceAccelG);
        }
        const uint32_t nowMs = millis();
        if (nowMs - lastPrintMs >= 2000UL) {
            lastPrintMs = nowMs;
            const SensorToDeviceObservationCaptureStatus st = capture.status();
            s.print("# setup frame accepted="); s.print(st.acceptedSamples);
            s.print('/'); s.print(capture.params().requiredSamples);
            s.print(" rejected="); s.print(st.rejectedSamples);
            s.print(" resets="); s.println(st.resetCount);
        }
        delay(5);
    }

    const SensorToDeviceObservationCaptureStatus st = capture.status();
    if (!st.complete) {
        tracker_serial_detail::printErr(s, "setup frame failed: no contiguous stable observation was captured");
        return false;
    }
    meanAccelG = st.meanSourceAccelG;
    s.print("# setup frame mean_g=");
    s.print(meanAccelG.x, 6); s.print(',');
    s.print(meanAccelG.y, 6); s.print(',');
    s.println(meanAccelG.z, 6);
    return true;
}

bool setupRunSensorToDeviceAlignmentStandalone(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    s.println();
    s.println("# SETUP CALIBRATION STEP 4/7: SENSOR-TO-DEVICE FRAME");
    s.println("# Device axes: +X right, +Y forward, +Z top/outward.");
    s.println("# Use the same +Y physical edge on every tracker; USB-connector edge is the recommended default.");

    Vec3 topSensorG = Vec3::zero();
    Vec3 forwardSensorG = Vec3::zero();
    if (!setupCaptureSensorFrameObservation(
            ctx,
            "# FRAME TOP: place the tracker flat with its top/outward face pointing upward (+Z up).",
            "# Wait until it stops wobbling, then press Enter.",
            topSensorG)) {
        return false;
    }
    if (!setupCaptureSensorFrameObservation(
            ctx,
            "# FRAME FORWARD: stand the tracker so its chosen +Y/forward edge points straight upward.",
            "# Wait until it stops wobbling, then press Enter.",
            forwardSensorG)) {
        return false;
    }
    const bool frameAlreadyValid = ctx.config && makeSensorToDeviceFrame(
        ctx.config->data.frame.sensorToDeviceValid,
        ctx.config->data.frame.sensorToDevice
    ).enabled;
    return setupApplySensorToDeviceAlignment(
        ctx,
        topSensorG,
        forwardSensorG,
        !frameAlreadyValid
    );
}

bool setupRunMagMotionAndApply(TrackerSerialCommandContext& ctx,
                                SetupMagAxisDynamicCollector& axisDynamic,
                                bool startFreshMagCalibration) {
    Stream& s = out(ctx);
    s.println();
    s.println("# SETUP CALIBRATION STEP 5/7: MAG HARD/SOFT MOTION + GYRO AXIS DATA");
    s.println("# Move slowly through as many orientations as possible. Avoid steel tables, speakers, chargers and magnets.");
    s.println("# Include several smooth rotations around different tracker axes; gyro+mag motion will be used to validate mag axis mapping.");

    if (startFreshMagCalibration) {
        if (ctx.setMagRuntimeEnabled) {
            (void)ctx.setMagRuntimeEnabled(true, false, ctx.setMagRuntimeEnabledUser);
        }
        if (ctx.config) {
            ctx.config->data.magCal.driverEnabled = true;
            ctx.config->updateCrc();
        }
        if (ctx.resetMagCalibration) ctx.resetMagCalibration(ctx.resetMagCalibrationUser);
        if (!ctx.startMagCalibration || !ctx.startMagCalibration(ctx.startMagCalibrationUser)) {
            tracker_serial_detail::printErr(s, "setup calibration failed: could not start magnetometer calibration");
            return false;
        }
    }

    drainSetupInput(ctx);
    axisDynamic.reset();
    bool magCalibrationApplied = false;

    for (uint8_t round = 1; round <= 4; ++round) {
        s.print("# setup mag motion round=");
        s.print(static_cast<unsigned int>(round));
        s.println("/4");
        s.println("# Rotate in full 3D: figure-eights, roll/pitch/yaw sweeps, and several smooth rotations around different tracker axes.");
        s.println("# Press Enter after good all-axis coverage, or let this round finish.");

        const uint32_t startMs = millis();
        uint32_t lastPrintMs = 0;
        while (millis() - startMs < 90000UL) {
            serviceSetupRuntime(ctx);
            axisDynamic.update(ctx);
            while (s.available() > 0) {
                const int c = s.read();
                if (c == '\n' || c == '\r') {
                    goto setup_mag_motion_round_done;
                }
            }
            const uint32_t nowMs = millis();
            if (nowMs - lastPrintMs >= 5000UL) {
                lastPrintMs = nowMs;
                s.print("# setup mag motion elapsed_s=");
                s.print((nowMs - startMs) / 1000UL);
                s.print(" axis_intervals=");
                s.print(axisDynamic.intervalCount);
                s.print(" target_intervals=");
                s.print(kSetupMagAxisDynamicTargetIntervals);
                s.print(" mag_samples=");
                s.print(axisDynamic.magSamplesSeen);
                s.print(" imu_samples=");
                s.print(axisDynamic.imuSamplesSeen);
                s.print(" gyro_max_dps=");
                s.println(axisDynamic.gyroNormMaxDps, 1);
            }
            delay(5);
        }

    setup_mag_motion_round_done:
        s.print("# setup mag dynamic_axis_intervals="); s.println(axisDynamic.intervalCount);
        s.print("# setup mag dynamic_axis_dropped="); s.println(axisDynamic.droppedIntervals);

        if (!magCalibrationApplied) {
            if (ctx.applyMagCalibration && ctx.applyMagCalibration(false, ctx.applyMagCalibrationUser)) {
                magCalibrationApplied = true;
                if (ctx.stopMagCalibration) ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
                tracker_serial_detail::printOk(s, "mag hard/soft calibration accepted in RAM");
            } else {
                if (ctx.printMagCalibrationStatus) {
                    ctx.printMagCalibrationStatus(s, ctx.printMagCalibrationStatusUser);
                }

                if (round >= 4) break;

                char line[8];
                if (!readSetupLine(ctx,
                                   "# Mag calibration still lacks enough valid coverage. Press Enter to continue collecting, or type q then Enter to abort.",
                                   line,
                                   sizeof(line),
                                   300000UL)) {
                    if (ctx.stopMagCalibration) ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
                    tracker_serial_detail::printErr(s, "setup calibration aborted while waiting to continue mag calibration");
                    return false;
                }
                if (line[0] == 'q' || line[0] == 'Q') {
                    if (ctx.stopMagCalibration) ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
                    tracker_serial_detail::printErr(s, "setup calibration aborted by user during mag calibration");
                    return false;
                }
                continue;
            }
        }

        if (axisDynamic.intervalCount >= kSetupMagAxisDynamicTargetIntervals) {
            return true;
        }

        if (round < 4) {
            s.print("# setup mag hard/soft is ready; continuing automatically for gyro-assisted axis intervals ");
            s.print(axisDynamic.intervalCount);
            s.print('/');
            s.println(kSetupMagAxisDynamicTargetIntervals);
            continue;
        }
    }

    if (magCalibrationApplied) {
        if (axisDynamic.intervalCount < kSetupMagAxisDynamicSolverMinIntervals) {
            s.print("# WARN mag hard/soft accepted, but gyro-assisted axis data is sparse: intervals=");
            s.print(axisDynamic.intervalCount);
            s.print(" solver_min=");
            s.println(kSetupMagAxisDynamicSolverMinIntervals);
        }
        return true;
    }

    if (ctx.stopMagCalibration) ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
    tracker_serial_detail::printErr(s, "setup calibration failed: mag hard/soft calibration did not pass quality gates");
    return false;
}

bool setupRunMagAxisMotionOnly(TrackerSerialCommandContext& ctx,
                               SetupMagAxisDynamicCollector& axisDynamic) {
    Stream& s = out(ctx);
    s.println();
    s.println("# SETUP CALIBRATION STEP 5b/7: MAG AXIS MOTION ONLY");
    s.println("# Existing hard/soft mag calibration is valid; rotate in full 3D so gyro+mag can infer axis mapping.");
    s.println("# Press Enter after several smooth rotations around different tracker axes, or let this finish.");

    if (ctx.setMagRuntimeEnabled) {
        (void)ctx.setMagRuntimeEnabled(true, false, ctx.setMagRuntimeEnabledUser);
    }

    drainSetupInput(ctx);
    axisDynamic.reset();
    const uint32_t startMs = millis();
    uint32_t lastPrintMs = 0;
    while (millis() - startMs < 60000UL) {
        serviceSetupRuntime(ctx);
        axisDynamic.update(ctx);
        while (s.available() > 0) {
            const int c = s.read();
            if (c == '\n' || c == '\r') {
                s.print("# setup mag axis motion intervals=");
                s.println(axisDynamic.intervalCount);
                if (axisDynamic.intervalCount >= kSetupMagAxisDynamicTargetIntervals) {
                    return true;
                }
                s.print("# WARN mag axis motion needs more gyro+mag intervals before auto-solve: ");
                s.print(axisDynamic.intervalCount);
                s.print('/');
                s.println(kSetupMagAxisDynamicTargetIntervals);
            }
        }
        const uint32_t nowMs = millis();
        if (nowMs - lastPrintMs >= 5000UL) {
            lastPrintMs = nowMs;
            s.print("# setup mag axis motion elapsed_s=");
            s.print((nowMs - startMs) / 1000UL);
            s.print(" intervals=");
            s.print(axisDynamic.intervalCount);
            s.print(" mag_samples=");
            s.print(axisDynamic.magSamplesSeen);
            s.print(" imu_samples=");
            s.print(axisDynamic.imuSamplesSeen);
            s.print(" gyro_max_dps=");
            s.println(axisDynamic.gyroNormMaxDps, 1);
        }
        delay(5);
    }
    s.print("# setup mag axis motion intervals=");
    s.println(axisDynamic.intervalCount);
    return axisDynamic.intervalCount >= kSetupMagAxisDynamicSolverMinIntervals;
}

bool setupSetAxisIdentity(TrackerSerialCommandContext& ctx) {
    char* identity[] = { const_cast<char*>("mag"), const_cast<char*>("axis"), const_cast<char*>("identity") };
    dispatchMag(ctx, 3, identity);
    return ctx.config && ctx.config->data.magCal.axisAlignmentValid;
}

bool setupSetAxisMapping(TrackerSerialCommandContext& ctx, const char* x, const char* y, const char* z) {
    char* setAxis[] = {
        const_cast<char*>("mag"), const_cast<char*>("axis"), const_cast<char*>("set"),
        const_cast<char*>(x), const_cast<char*>(y), const_cast<char*>(z)
    };
    dispatchMag(ctx, 6, setAxis);
    return ctx.config && ctx.config->data.magCal.axisAlignmentValid;
}

bool setupApplyAxisMatrix(TrackerSerialCommandContext& ctx, const Mat3& m) {
    if (!ctx.config) return false;
    ctx.config->data.magCal.magToImu = m;
    ctx.config->data.magCal.axisAlignmentValid = true;
    ctx.config->updateCrc();
    if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
    return true;
}

bool setupRunAxisAlignment(TrackerSerialCommandContext& ctx,
                           const SetupMagAxisAutoCollector& axisAuto,
                           const SetupMagAxisDynamicCollector& axisDynamic,
                           const char* axisX,
                           const char* axisY,
                           const char* axisZ) {
    Stream& s = out(ctx);
    s.println();
    s.println("# SETUP CALIBRATION STEP 6/7: MAG AXIS ALIGNMENT");

    if (axisX && axisY && axisZ) {
        s.println("# Manual axis mapping was provided; applying it instead of auto-detection.");
        return setupSetAxisMapping(ctx, axisX, axisY, axisZ);
    }

    if (ctx.config && ctx.imuCal) {
        SetupMagAxisAutoResult staticAxis;
        const bool staticOk = setupAutoSolveMagAxis(axisAuto, *ctx.config, *ctx.imuCal, staticAxis);

        SetupMagAxisDynamicResult dynamicAxis;
        const bool dynamicOk = setupAutoSolveMagAxisDynamic(axisDynamic, *ctx.config, dynamicAxis);

        if (dynamicOk) {
            s.print("# gyro_mag_axis_mapping=");
            setupPrintMagAxisMapping(s, dynamicAxis.magToImu);
            s.println();
            s.print("# gyro_mag_axis_score="); s.println(dynamicAxis.score, 6);
            s.print("# gyro_mag_axis_second_best="); s.println(dynamicAxis.secondBestScore, 6);
            s.print("# gyro_mag_axis_direction_error="); s.println(dynamicAxis.meanDirectionError, 6);
            s.print("# gyro_mag_axis_magnitude_error="); s.println(dynamicAxis.meanMagnitudeError, 6);
            s.print("# gyro_mag_axis_intervals="); s.println(static_cast<unsigned int>(dynamicAxis.usedIntervals));

            if (staticOk) {
                s.print("# static_mag_axis_mapping=");
                setupPrintMagAxisMapping(s, staticAxis.magToImu);
                s.println();
                s.print("# static_mag_axis_score="); s.println(staticAxis.score, 6);
                if (!setupAxisMatricesEqual(dynamicAxis.magToImu, staticAxis.magToImu)) {
                    s.println("# WARN static inclination axis check disagrees with gyro-assisted axis check; using gyro-assisted result");
                }
            } else {
                s.println("# WARN static inclination axis check was inconclusive; using gyro-assisted result");
            }

            if (setupApplyAxisMatrix(ctx, dynamicAxis.magToImu)) {
                tracker_serial_detail::printOk(s, "mag axis alignment gyro-assisted auto-detected in RAM");
                return true;
            }
        }

        if (staticOk) {
            s.print("# static_mag_axis_mapping=");
            setupPrintMagAxisMapping(s, staticAxis.magToImu);
            s.println();
            s.print("# static_mag_axis_score="); s.println(staticAxis.score, 6);
            s.print("# static_mag_axis_second_best="); s.println(staticAxis.secondBestScore, 6);
            s.print("# static_mag_axis_inclination_mean="); s.println(staticAxis.inclinationMean, 6);
            s.print("# static_mag_axis_inclination_stddev="); s.println(staticAxis.inclinationStddev, 6);
            s.print("# static_mag_axis_samples="); s.println(static_cast<unsigned int>(staticAxis.usedSamples));
            s.println("# WARN gyro-assisted axis check was inconclusive; falling back to static inclination check");
            if (setupApplyAxisMatrix(ctx, staticAxis.magToImu)) {
                tracker_serial_detail::printOk(s, "mag axis alignment auto-detected in RAM");
                return true;
            }
        }

        s.print("# WARN automatic mag axis alignment failed. dynamic_intervals=");
        s.print(axisDynamic.intervalCount);
        s.print(" solver_min=");
        s.print(kSetupMagAxisDynamicSolverMinIntervals);
        s.print(" target=");
        s.print(kSetupMagAxisDynamicTargetIntervals);
        s.print(" static_faces=");
        s.println(axisAuto.count);
    }

    char line[48] = {};
    s.println("# Enter mag axis mapping as three tokens for IMU/body X Y Z, for example: +x +y +z");
    s.println("# Leave blank only if this board's magnetometer axes are already known to match IMU axes.");
    if (!readSetupLine(ctx, "# axis mapping> ", line, sizeof(line), 300000UL)) {
        tracker_serial_detail::printErr(s, "setup calibration aborted: axis mapping timeout");
        return false;
    }
    if (line[0] == '\0') {
        return setupSetAxisIdentity(ctx);
    }

    char* tokens[3] = {};
    uint8_t count = 0;
    for (char* p = strtok(line, " \t"); p != nullptr && count < 3; p = strtok(nullptr, " \t")) {
        tokens[count++] = p;
    }
    if (count != 3) {
        tracker_serial_detail::printErr(s, "setup calibration failed: expected three axis tokens, for example +x +y +z");
        return false;
    }

    return setupSetAxisMapping(ctx, tokens[0], tokens[1], tokens[2]);
}

bool setupDisableMagFor6Dof(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    if (!ctx.config) {
        tracker_serial_detail::printErr(s, "setup calibration failed: config is not available");
        return false;
    }

    if (ctx.stopMagCalibration) ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
    if (ctx.setMagRuntimeEnabled) {
        (void)ctx.setMagRuntimeEnabled(false, false, ctx.setMagRuntimeEnabledUser);
    }
    if (ctx.setMagYawCorrectionApplyEnabled) {
        (void)ctx.setMagYawCorrectionApplyEnabled(false, false, ctx.setMagYawCorrectionApplyEnabledUser);
    }
    if (ctx.resetMagYawCorrection) ctx.resetMagYawCorrection(ctx.resetMagYawCorrectionUser);

    ctx.config->data.magCal.driverEnabled = false;
    ctx.config->data.magYaw.applyEnabled = false;
    ctx.config->updateCrc();
    if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();
    s.println("# setup calibration: magnetometer disabled for 6DoF-only tracker");
    return true;
}

bool setupEnableProductionTracking(TrackerSerialCommandContext& ctx, bool noMag) {
    Stream& s = out(ctx);
    s.println();
    s.println(noMag
        ? "# SETUP CALIBRATION STEP 7/7: ENABLE 6DOF TRACKING FEATURES"
        : "# SETUP CALIBRATION STEP 7/7: ENABLE PRODUCTION TRACKING FEATURES");

    if (!ctx.config) {
        tracker_serial_detail::printErr(s, "setup calibration failed: config is not available");
        return false;
    }

    ctx.config->data.ahrs.useAccelCorrection = true;
    ctx.config->data.ahrsRuntime.accelCorrectionEnabled = true;
    ctx.config->data.ahrsRuntime.adaptiveAccelCorrection = true;
    ctx.config->data.ahrsRuntime.reserved |=
        tracker_config_detail::AHRS_RUNTIME_FLAG_RUNTIME_BIAS_ENABLED;

    if (ctx.gyroTempComp) ctx.gyroTempComp->setEnabled(true);
    if (ctx.setRuntimeGyroBiasEnabled) {
        (void)ctx.setRuntimeGyroBiasEnabled(true, ctx.setRuntimeGyroBiasEnabledUser);
    }
    if (noMag) {
        if (!setupDisableMagFor6Dof(ctx)) return false;
    } else if (ctx.setMagYawCorrectionApplyEnabled) {
        if (!ctx.setMagYawCorrectionApplyEnabled(true, false, ctx.setMagYawCorrectionApplyEnabledUser)) {
            tracker_serial_detail::printErr(s, "setup calibration failed: mag yaw apply enable failed");
            return false;
        }
    } else {
        ctx.config->data.magYaw.applyEnabled = true;
    }

    trackerSerialCaptureRuntimeToConfig(ctx);
    if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);

    const SetupReadiness r = readSetupReadiness(ctx);
    const bool ready = noMag
        ? (r.tracking6dof() && r.tempQuality() && r.runtimeBias())
        : r.production();
    if (!ready) {
        s.println(noMag
            ? "# WARN setup calibration finished in RAM, but 6dof_ready is still no; run setup status for missing items"
            : "# WARN setup calibration finished in RAM, but production_ready is still no; run setup status for missing items");
    } else {
        tracker_serial_detail::printOk(s, noMag
            ? "6DoF tracking features enabled in RAM"
            : "setup calibration features enabled in RAM");
    }
    if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();
    printSetupStatus(ctx);
    return ready;
}

struct SetupCalibrationOptions {
    bool full = false;
    bool noMag = false;
    const char* axisX = nullptr;
    const char* axisY = nullptr;
    const char* axisZ = nullptr;
};

bool parseSetupCalibrationOptions(TrackerSerialCommandContext& ctx,
                                  int argc,
                                  char** argv,
                                  SetupCalibrationOptions& opt) {
    Stream& s = out(ctx);
    for (int i = 2; i < argc; ++i) {
        if (is(argv[i], "axis")) {
            if (i + 3 >= argc) {
                tracker_serial_detail::printErr(s, "usage: setup calibration [resume|full] [nomag|6dof] [axis <bodyX> <bodyY> <bodyZ>]");
                return false;
            }
            opt.axisX = argv[i + 1];
            opt.axisY = argv[i + 2];
            opt.axisZ = argv[i + 3];
            i += 3;
        } else if (is(argv[i], "full") || is(argv[i], "force") || is(argv[i], "restart") || is(argv[i], "all")) {
            opt.full = true;
        } else if (is(argv[i], "resume") || is(argv[i], "continue") || is(argv[i], "missing") || is(argv[i], "auto")) {
            opt.full = false;
        } else if (is(argv[i], "nomag") || is(argv[i], "no-mag") || is(argv[i], "6dof") ||
                   is(argv[i], "sixdof") || is(argv[i], "without-mag")) {
            opt.noMag = true;
        } else if (is(argv[i], "help") || is(argv[i], "?")) {
            s.println("setup calibration [resume|full] [nomag|6dof] [axis <bodyX> <bodyY> <bodyZ>]");
            s.println("  resume/default: skip valid stages and save every newly completed stage to NVS");
            s.println("  full: recalibrate all stages transactionally and save once at the end");
            s.println("  nomag/6dof: skip mag hard/soft + mag axis, disable mag yaw, save 6DoF config");
            return false;
        } else {
            tracker_serial_detail::printErr(s, "usage: setup calibration [resume|full] [nomag|6dof] [axis <bodyX> <bodyY> <bodyZ>]");
            return false;
        }
    }
    return true;
}

bool setupCheckpointCommit(TrackerSerialCommandContext& ctx, const char* stageName) {
    Stream& s = out(ctx);
    if (!g_setupCalibrationTx.commit(ctx)) return false;
    s.print("# setup calibration checkpoint saved stage=");
    s.println(stageName ? stageName : "unknown");
    return true;
}

void cmdSetupCalibrationFull(TrackerSerialCommandContext& ctx,
                             bool noMag,
                             const char* axisX,
                             const char* axisY,
                             const char* axisZ) {
    Stream& s = out(ctx);
    s.println(noMag ? "# SETUP CALIBRATION FULL 6DOF/NOMAG" : "# SETUP CALIBRATION FULL");
    s.println("# Full mode recalibrates every stage transactionally and writes NVS only once at the end.");
    s.println("# Use plain 'setup calibration' to resume missing stages and skip already saved work.");

    if (!ctx.calibrationIo || !ctx.imuCal || !ctx.accelCalRunner || !ctx.config || !ctx.configStore) {
        tracker_serial_detail::printErr(s, "setup calibration failed: required calibration dependencies are not available");
        return;
    }

    SetupCalibrationTransaction& tx = g_setupCalibrationTx;
    tx.begin(ctx);
    setupPrepareCalibrationRuntime(ctx);
    auto fail = [&](const char* reason) {
        tx.rollback(ctx, reason);
    };

    if (!setupRunRestGyro(ctx)) { fail("rest_gyro"); return; }
    if (!setupRunTemperatureFit(ctx)) { fail("gyro_temperature"); return; }

    SetupMagAxisAutoCollector& axisAuto = g_setupMagAxisAutoCollector;
    axisAuto.reset();
    SetupMagAxisDynamicCollector& axisDynamic = g_setupMagAxisDynamicCollector;
    axisDynamic.reset();
    SetupFrameObservations& frameObservations = g_setupFrameObservations;
    frameObservations.reset();
    if (!setupRunAccelFacesWithMagCollection(ctx, axisAuto, frameObservations, !noMag)) { fail(noMag ? "accel_6pos" : "accel_mag_faces"); return; }
    if (!setupRunSensorToDeviceAlignmentFromAccel(ctx, frameObservations)) { fail("sensor_to_device"); return; }
    if (!noMag) {
        if (!setupRunMagMotionAndApply(ctx, axisDynamic, false)) { fail("mag_hard_soft"); return; }
        if (!setupRunAxisAlignment(ctx, axisAuto, axisDynamic, axisX, axisY, axisZ)) { fail("mag_axis"); return; }
    } else {
        s.println("# skip mag_hard_soft: nomag/6dof requested");
        s.println("# skip mag_axis: nomag/6dof requested");
    }
    if (!setupEnableProductionTracking(ctx, noMag)) { fail("enable_tracking"); return; }

    if (!tx.commit(ctx)) { fail("commit_save"); return; }

    tracker_serial_detail::printOk(s, noMag ? "setup calibration complete: 6DoF tracker calibration saved to NVS" : "setup calibration complete: tracker calibration saved to NVS");
    printSetupStatus(ctx);
}

void cmdSetupCalibrationResume(TrackerSerialCommandContext& ctx,
                               bool noMag,
                               const char* axisX,
                               const char* axisY,
                               const char* axisZ) {
    Stream& s = out(ctx);
    s.println(noMag ? "# SETUP CALIBRATION RESUME 6DOF/NOMAG" : "# SETUP CALIBRATION RESUME");
    s.println("# Resume mode skips stages that are already valid and saves each newly completed stage to NVS immediately.");
    if (noMag) s.println("# nomag/6dof: mag hard/soft and mag axis stages will be skipped and mag yaw will be disabled.");
    s.println("# Use 'setup calibration full' to intentionally recalibrate everything from scratch.");

    if (!ctx.calibrationIo || !ctx.imuCal || !ctx.accelCalRunner || !ctx.config || !ctx.configStore) {
        tracker_serial_detail::printErr(s, "setup calibration failed: required calibration dependencies are not available");
        return;
    }

    SetupMagAxisAutoCollector& axisAuto = g_setupMagAxisAutoCollector;
    SetupMagAxisDynamicCollector& axisDynamic = g_setupMagAxisDynamicCollector;
    SetupFrameObservations& frameObservations = g_setupFrameObservations;
    axisAuto.reset();
    axisDynamic.reset();
    frameObservations.reset();

    SetupReadiness r = readSetupReadiness(ctx);
    const bool restReadyAtStart = r.gyroReady;
    const bool tempReadyAtStart = setupStoredTempModelReady(ctx);
    if (restReadyAtStart) {
        s.println("# skip rest_gyro: already valid in RAM/NVS");
    } else {
        SetupCalibrationTransaction& tx = g_setupCalibrationTx;
        tx.begin(ctx);
        setupPrepareCalibrationRuntime(ctx);
        if (!setupRunRestGyro(ctx)) { tx.rollback(ctx, "rest_gyro"); return; }
        if (!setupCheckpointCommit(ctx, "rest_gyro")) { tx.rollback(ctx, "rest_gyro_commit"); return; }
    }

    if (tempReadyAtStart) {
        s.println("# skip gyro_temperature: temperature model already valid");
    } else {
        SetupCalibrationTransaction& tx = g_setupCalibrationTx;
        tx.begin(ctx);
        setupPrepareCalibrationRuntime(ctx);
        if (!setupRunTemperatureFit(ctx)) { tx.rollback(ctx, "gyro_temperature"); return; }
        if (!setupCheckpointCommit(ctx, "gyro_temperature")) { tx.rollback(ctx, "gyro_temperature_commit"); return; }
    }

    // Re-read readiness after rest/temp commits because they may update runtime/config state.
    r = readSetupReadiness(ctx);
    const bool needMagCollection = !noMag && !r.magCal;
    const bool needAxisAssist = !noMag && (!r.magAxis || (axisX && axisY && axisZ));
    bool magCollectionStartedDuringAccel = false;

    if (r.accelReady) {
        s.println("# skip accel_6pos: already valid in RAM/NVS");
        if (r.frameReady) {
            s.println("# skip sensor_to_device: already valid in RAM/NVS");
        } else {
            SetupCalibrationTransaction& tx = g_setupCalibrationTx;
            tx.begin(ctx);
            if (!setupRunSensorToDeviceAlignmentStandalone(ctx)) { tx.rollback(ctx, "sensor_to_device"); return; }
            if (!setupCheckpointCommit(ctx, "sensor_to_device")) { tx.rollback(ctx, "sensor_to_device_commit"); return; }
        }
    } else {
        SetupCalibrationTransaction& tx = g_setupCalibrationTx;
        tx.begin(ctx);
        const bool collectMagDuringAccel = needMagCollection || needAxisAssist;
        magCollectionStartedDuringAccel = collectMagDuringAccel;
        if (!setupRunAccelFacesWithMagCollection(ctx, axisAuto, frameObservations, collectMagDuringAccel)) {
            tx.rollback(ctx, "accel_6pos");
            return;
        }
        if (!setupRunSensorToDeviceAlignmentFromAccel(ctx, frameObservations)) {
            tx.rollback(ctx, "sensor_to_device");
            return;
        }
        if (!setupCheckpointCommit(ctx, "accel_6pos_and_frame")) { tx.rollback(ctx, "accel_frame_commit"); return; }
    }

    if (noMag) {
        s.println("# skip mag_hard_soft: nomag/6dof requested");
        s.println("# skip mag_axis: nomag/6dof requested");
    } else {
        r = readSetupReadiness(ctx);
        if (r.magCal) {
            s.println("# skip mag_hard_soft: already valid in RAM/NVS");
        } else {
            SetupCalibrationTransaction& tx = g_setupCalibrationTx;
            tx.begin(ctx);
            axisDynamic.reset();
            const bool startFreshMagCalibration = !magCollectionStartedDuringAccel;
            if (!setupRunMagMotionAndApply(ctx, axisDynamic, startFreshMagCalibration)) { tx.rollback(ctx, "mag_hard_soft"); return; }
            if (!setupCheckpointCommit(ctx, "mag_hard_soft")) { tx.rollback(ctx, "mag_hard_soft_commit"); return; }
        }

        r = readSetupReadiness(ctx);
        if (r.magAxis && !(axisX && axisY && axisZ)) {
            s.println("# skip mag_axis: already valid in RAM/NVS");
        } else {
            SetupCalibrationTransaction& tx = g_setupCalibrationTx;
            tx.begin(ctx);
            if (axisDynamic.intervalCount < kSetupMagAxisDynamicSolverMinIntervals && !axisAuto.count && !(axisX && axisY && axisZ)) {
                (void)setupRunMagAxisMotionOnly(ctx, axisDynamic);
            }
            if (!setupRunAxisAlignment(ctx, axisAuto, axisDynamic, axisX, axisY, axisZ)) { tx.rollback(ctx, "mag_axis"); return; }
            if (!setupCheckpointCommit(ctx, "mag_axis")) { tx.rollback(ctx, "mag_axis_commit"); return; }
        }
    }

    // Always refresh feature toggles in RAM/NVS. This is quick and makes resume
    // useful after low-level diagnostics changed a flag manually.
    {
        SetupCalibrationTransaction& tx = g_setupCalibrationTx;
        tx.begin(ctx);
        if (!setupEnableProductionTracking(ctx, noMag)) { tx.rollback(ctx, "enable_tracking"); return; }
        if (!setupCheckpointCommit(ctx, "enable_tracking")) { tx.rollback(ctx, "enable_tracking_commit"); return; }
    }

    tracker_serial_detail::printOk(s, noMag ? "setup calibration resume complete: 6DoF stages saved to NVS" : "setup calibration resume complete: all currently missing stages are saved to NVS");
    printSetupStatus(ctx);
}

void printSetupFrameStatus(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    s.println("# SENSOR-TO-DEVICE FRAME STATUS");
    s.println("# convention: +X right, +Y forward, +Z top/outward");
    if (!ctx.config) {
        s.println("valid=no");
        return;
    }
    const SensorToDeviceFrame frame = makeSensorToDeviceFrame(
        ctx.config->data.frame.sensorToDeviceValid,
        ctx.config->data.frame.sensorToDevice
    );
    s.print("valid="); s.println(yesNo(frame.enabled));
    s.print("determinant="); s.println(ctx.config->data.frame.sensorToDevice.determinant(), 6);
    for (uint8_t row = 0; row < 3; ++row) {
        s.print("row"); s.print(row); s.print('=');
        s.print(ctx.config->data.frame.sensorToDevice.m[row][0], 6); s.print(',');
        s.print(ctx.config->data.frame.sensorToDevice.m[row][1], 6); s.print(',');
        s.println(ctx.config->data.frame.sensorToDevice.m[row][2], 6);
    }
}

void cmdSetupFrame(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& s = out(ctx);
    if (argc < 3 || is(argv[2], "status")) {
        printSetupFrameStatus(ctx);
        return;
    }
    if (!is(argv[2], "calibrate") && !is(argv[2], "align")) {
        tracker_serial_detail::printErr(s, "usage: setup frame status|calibrate");
        return;
    }
    if (!ctx.config || !ctx.configStore || !ctx.imuCal || !ctx.imuCal->accelCalValid) {
        tracker_serial_detail::printErr(s, "setup frame calibrate requires saved accel calibration and config store");
        return;
    }

    SetupCalibrationTransaction& tx = g_setupCalibrationTx;
    tx.begin(ctx);
    if (!setupRunSensorToDeviceAlignmentStandalone(ctx)) {
        tx.rollback(ctx, "sensor_to_device");
        return;
    }
    if (!tx.commit(ctx)) {
        tx.rollback(ctx, "sensor_to_device_commit");
        return;
    }
    tracker_serial_detail::printOk(s, "sensor-to-device frame calibration saved to NVS");
    printSetupFrameStatus(ctx);
}

void cmdSetupCalibration(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    SetupCalibrationOptions opt;
    if (!parseSetupCalibrationOptions(ctx, argc, argv, opt)) return;

    if (opt.full) {
        cmdSetupCalibrationFull(ctx, opt.noMag, opt.axisX, opt.axisY, opt.axisZ);
    } else {
        cmdSetupCalibrationResume(ctx, opt.noMag, opt.axisX, opt.axisY, opt.axisZ);
    }
}

} // namespace

void trackerSerialDispatchSetupCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& s = out(ctx);

    if (argc < 2 || is(argv[1], "guide") || is(argv[1], "help") || is(argv[1], "?")) {
        printSetupGuide(s);
        return;
    }

    if (is(argv[1], "status")) {
        printSetupStatus(ctx);
        return;
    }

    if (is(argv[1], "calibration") || is(argv[1], "calibrate")) {
        cmdSetupCalibration(ctx, argc, argv);
        return;
    }

    if (is(argv[1], "frame")) {
        cmdSetupFrame(ctx, argc, argv);
        return;
    }

    if (is(argv[1], "wifi")) {
        cmdSetupWifi(ctx, argc, argv);
        return;
    }

    tracker_serial_detail::printErr(s, "usage: setup guide|status|wifi|frame status|frame calibrate|calibration [resume|full] [axis <bodyX> <bodyY> <bodyZ>]");
}

} // namespace tracker
