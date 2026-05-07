#include <Arduino.h>
#include <SPI.h>
#include <cmath>

#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "connection/lsm6dsv_sensorhub.hpp"
#include "sensor/qmc6309.hpp"
#include "sensor/mag_calibration.hpp"
#include "sensor/mag_runtime.hpp"
#include "sensor/mag_heading.hpp"
#include "sensor/mag_yaw_correction.hpp"
#include "sensor/calibration.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/imu_quality.hpp"
#include "sensor/fifo_calibrations.hpp"
#include "config/tracker_config.hpp"
#include "serial/tracker_serial_commands.hpp"

using namespace tracker;

// ============================================================
// ESP32-C3 + LSM6DSV command-based tracker firmware
// ============================================================
// Features:
//   - FIFO v2 continuous IMU pipeline
//   - hardware FIFO timestamps
//   - FIFO temperature tags
//   - quality/dropped/saturation monitor
//   - gyro/accel calibration via Serial commands
//   - config save/load via NVS
//   - command-based runtime diagnostics
//   - optional stream raw/scaled/quaternion
//   - optional non-blocking static test via command:
//       test static <seconds>
//       test stop
//       test status
//
// Important:
//   This is not the long blocking 30-min test main.
//   It is a normal command-driven runtime main.
// ============================================================

// Hardware constants. Keep pins compile-time for now.
static constexpr int PIN_LSM_SCK  = 3;
static constexpr int PIN_LSM_MISO = 0;
static constexpr int PIN_LSM_MOSI = 2;
static constexpr int PIN_LSM_CS   = 1;
static constexpr int PIN_LSM_INT1 = 10;

static constexpr uint32_t SERIAL_BAUD_DEFAULT = 921600;
static constexpr uint32_t SPI_HZ_DEFAULT = 1000000;
static constexpr uint8_t SPI_MODE_DEFAULT = SPI_MODE0;

static constexpr size_t FIFO_RAW_BUFFER_CAPACITY = 160;
static constexpr size_t MAG_RAW_BUFFER_CAPACITY = 48;
static constexpr float MAG_HUB_ODR_HZ = 60.0f;
static constexpr float MAG_HUB_PERIOD_US = 1000000.0f / MAG_HUB_ODR_HZ;
static constexpr uint16_t FIFO_MAX_WORDS_PER_DRAIN_DEFAULT = 384;
static constexpr uint8_t MAX_DRAIN_ROUNDS_PER_EVENT_DEFAULT = 6;
static constexpr uint32_t FIFO_WAIT_TIMEOUT_MS = 1000;
static constexpr uint32_t HEARTBEAT_PERIOD_MS = 30000;

// ============================================================
// Global runtime objects
// ============================================================

ArduinoLsm6dsvSpiTransport lsmBus(SPI, PIN_LSM_CS, SPI_HZ_DEFAULT, SPI_MODE_DEFAULT);
Lsm6dsv lsm(lsmBus);
Lsm6dsvFifoReader lsmFifo(lsmBus, lsm);
Lsm6dsvSensorHub lsmHub(lsmBus);
Qmc6309 qmc(lsmHub);

TrackerConfig g_config;
TrackerConfigStore g_configStore;

ImuCalibration g_imuCal;
GyroTempCompensator g_gyroTempComp;
ImuQualityMonitor g_quality;
Ahrs6Dof g_ahrs6dof;

TrackerSerialStreamState g_streamState;
TrackerSerialCommandContext g_cmdCtx;
TrackerSerialCommandInterface<> g_cli;

FifoCalibrationIo g_calIo;
FifoAccel6PosCalibrationRunner g_accelCalRunner;

Lsm6dsv::RawSample g_fifoRaw[FIFO_RAW_BUFFER_CAPACITY];
Lsm6dsvFifoReader::MagRawSample g_magRaw[MAG_RAW_BUFFER_CAPACITY];

volatile uint32_t g_fifoIntCount = 0;
volatile uint32_t g_fifoLastIrqUs = 0;

static uint32_t g_lastHandledFifoIntCount = 0;
static uint32_t g_fifoIntMissed = 0;
static uint32_t g_fifoStatusFallbackEvents = 0;
static uint32_t g_fifoWaitTimeouts = 0;
static uint64_t g_lastSampleTimestampUs = 0;
static uint32_t g_runtimeSamples = 0;
static float g_latestTempC = 25.0f;
static bool g_configLoadedFromNvs = false;
static uint32_t g_lastHeartbeatMs = 0;
static float g_lastOutputConfidence = 0.0f;

static constexpr uint32_t TRACKING_RECOVERY_STABLE_SAMPLES = 128;
static bool g_trackingRecovering = false;
static uint32_t g_trackingRecoveryStableSamples = 0;
static uint32_t g_trackingRecoveryEnterCount = 0;
static uint32_t g_trackingRecoveryLastFlags = 0;
static uint64_t g_trackingRecoveryLastTimestampUs = 0;

struct MagRuntimeState {
    bool hubInitialized = false;
    bool runtimeEnabled = false;
    bool fifoArmed = false;
    bool lastInitOk = false;
    uint32_t samples = 0;
    uint32_t queuePops = 0;
    uint32_t lastSampleMs = 0;
    uint32_t lastEnableMs = 0;
    uint32_t enableFailures = 0;
    uint32_t nacksSeen = 0;
    Lsm6dsvFifoReader::MagRawSample lastRaw;
    float lastNormRaw = 0.0f;
};

static MagRuntimeState g_magState;
static MagCalibrationCollector g_magCalCollector;
static MagRuntimeProcessor g_magProcessor;
static MagProcessedSample g_lastMagProcessed;

static MagHeadingEstimator g_magHeading;
static MagHeadingSample g_lastMagHeading;

static MagYawCorrectionController g_magYawCorrection;
static MagYawCorrectionOutput g_lastMagYawCorrection;

struct MagHeadingReferenceState {
    bool valid = false;
    float worldYawRad = 0.0f;
    uint32_t setMs = 0;
    uint32_t magSeq = 0;
    uint64_t magTimestampUs = 0;

    void clear() {
        valid = false;
        worldYawRad = 0.0f;
        setMs = 0;
        magSeq = 0;
        magTimestampUs = 0;
    }

    float worldYawDeg() const {
        return worldYawRad * MATH_RAD_TO_DEG;
    }
};

static MagHeadingReferenceState g_magHeadingRef;

struct MagHeadingAutoReferenceState {
    bool enabled = true;
    bool done = false;

    uint32_t stableSinceMs = 0;
    uint32_t setCount = 0;
    uint32_t lastSetMs = 0;
    uint32_t lastRejectFlags = 0;

    float lastGyroNormDps = 0.0f;
    float lastAccelTrust = 0.0f;
    float lastHorizontalTrust = 0.0f;

    void resetCandidate() {
        stableSinceMs = 0;
        lastRejectFlags = 0;
        lastGyroNormDps = 0.0f;
        lastAccelTrust = 0.0f;
        lastHorizontalTrust = 0.0f;
    }

    void resetAll() {
        enabled = true;
        done = false;
        stableSinceMs = 0;
        setCount = 0;
        lastSetMs = 0;
        lastRejectFlags = 0;
        lastGyroNormDps = 0.0f;
        lastAccelTrust = 0.0f;
        lastHorizontalTrust = 0.0f;
    }
};

static MagHeadingAutoReferenceState g_magHeadingAutoRef;

// ============================================================
// Small stats for command-driven static test
// ============================================================

struct ScalarStats {
    uint32_t count = 0;
    double sum = 0.0;
    double sumSq = 0.0;
    float minValue = 0.0f;
    float maxValue = 0.0f;

    void reset() {
        count = 0;
        sum = 0.0;
        sumSq = 0.0;
        minValue = 0.0f;
        maxValue = 0.0f;
    }

    void push(float x) {
        if (!std::isfinite(x)) return;
        if (count == 0) {
            minValue = x;
            maxValue = x;
        } else {
            if (x < minValue) minValue = x;
            if (x > maxValue) maxValue = x;
        }
        count++;
        sum += static_cast<double>(x);
        sumSq += static_cast<double>(x) * static_cast<double>(x);
    }

    float mean() const {
        return count == 0 ? 0.0f : static_cast<float>(sum / static_cast<double>(count));
    }

    float stddev() const {
        if (count < 2) return 0.0f;
        const double n = static_cast<double>(count);
        const double m = sum / n;
        const double v = (sumSq / n) - (m * m);
        return static_cast<float>(v > 0.0 ? std::sqrt(v) : 0.0);
    }
};

struct Vec3Stats {
    uint32_t count = 0;
    Vec3 sum = Vec3::zero();
    Vec3 sumSq = Vec3::zero();
    Vec3 minValue = Vec3::zero();
    Vec3 maxValue = Vec3::zero();

    void reset() {
        count = 0;
        sum = Vec3::zero();
        sumSq = Vec3::zero();
        minValue = Vec3::zero();
        maxValue = Vec3::zero();
    }

    void push(const Vec3& v) {
        if (!v.isFinite()) return;

        if (count == 0) {
            minValue = v;
            maxValue = v;
        } else {
            if (v.x < minValue.x) minValue.x = v.x;
            if (v.y < minValue.y) minValue.y = v.y;
            if (v.z < minValue.z) minValue.z = v.z;

            if (v.x > maxValue.x) maxValue.x = v.x;
            if (v.y > maxValue.y) maxValue.y = v.y;
            if (v.z > maxValue.z) maxValue.z = v.z;
        }

        count++;
        sum += v;
        sumSq += hadamard(v, v);
    }

    Vec3 mean() const {
        return count == 0 ? Vec3::zero() : sum / static_cast<float>(count);
    }

    Vec3 stddev() const {
        if (count < 2) return Vec3::zero();

        const Vec3 m = mean();
        const Vec3 meanSq = sumSq / static_cast<float>(count);
        Vec3 v = meanSq - hadamard(m, m);

        if (v.x < 0.0f) v.x = 0.0f;
        if (v.y < 0.0f) v.y = 0.0f;
        if (v.z < 0.0f) v.z = 0.0f;

        return Vec3(std::sqrt(v.x), std::sqrt(v.y), std::sqrt(v.z));
    }
};

struct StaticRuntimeTest {
    bool active = false;
    bool stopRequested = false;
    uint32_t durationMs = 0;
    uint32_t startMs = 0;
    uint32_t lastProgressMs = 0;

    float tempStartC = 0.0f;
    float tempEndC = 0.0f;
    bool tempCaptured = false;

    uint32_t samples = 0;
    uint32_t hwTs = 0;
    uint32_t fallbackTs = 0;
    uint32_t badTs = 0;
    uint32_t droppedEstimate = 0;
    uint32_t recoveryRequests = 0;
    uint32_t ahrsSkipped = 0;
    uint32_t accelDisabled = 0;

    uint32_t fifoOverrunAtStart = 0;
    uint32_t fifoFullAtStart = 0;
    uint32_t fifoUnknownAtStart = 0;
    uint32_t fifoHwTsAtStart = 0;
    uint32_t fifoFbTsAtStart = 0;

    // Magnetometer / yaw correction snapshots.
    bool magEnabledAtStart = false;
    bool magYawApplyEnabledAtStart = false;
    bool magRefValidAtStart = false;

    float magErrorStartDeg = 0.0f;
    float magErrorEndDeg = 0.0f;
    float magErrorAbsMaxDeg = 0.0f;
    double magErrorAbsSumDeg = 0.0;
    uint32_t magErrorSamples = 0;

    uint32_t magTrustedAtStart = 0;
    uint32_t magRejectedAtStart = 0;

    uint32_t magHeadingValidAtStart = 0;
    uint32_t magHeadingRejectedAtStart = 0;

    uint32_t magYawUpdatesAtStart = 0;
    uint32_t magYawGateOpenAtStart = 0;
    uint32_t magYawGateClosedAtStart = 0;
    uint32_t magYawApplyAllowedAtStart = 0;
    uint32_t magYawAppliedAtStart = 0;

    uint32_t magYawRejectNoReferenceAtStart = 0;
    uint32_t magYawRejectHeadingInvalidAtStart = 0;
    uint32_t magYawRejectMagNotTrustedAtStart = 0;
    uint32_t magYawRejectMagStaleAtStart = 0;
    uint32_t magYawRejectHorizontalBadAtStart = 0;
    uint32_t magYawRejectInnovationTooLargeAtStart = 0;
    uint32_t magYawRejectGyroMovingAtStart = 0;
    uint32_t magYawRejectAccelNotTrustedAtStart = 0;

    ScalarStats dtUs;
    ScalarStats accelNormG;
    ScalarStats accelTrust;
    ScalarStats tempC;
    Vec3Stats gyroAfterRadS;

    ScalarStats magHeadingHorizontalNorm;
    ScalarStats magYawCombinedTrust;
    ScalarStats magYawCorrectionRateDegS;
    ScalarStats magYawCorrectionStepDeg;

    bool poseCaptured = false;
    Vec3 eulerStartDeg = Vec3::zero();
    Vec3 eulerEndDeg = Vec3::zero();
    Quat qStart = Quat::identity();
    Quat qEnd = Quat::identity();

    void reset() {
        active = false;
        stopRequested = false;
        durationMs = 0;
        startMs = 0;
        lastProgressMs = 0;
        tempStartC = 0.0f;
        tempEndC = 0.0f;
        tempCaptured = false;
        samples = 0;
        hwTs = 0;
        fallbackTs = 0;
        badTs = 0;
        droppedEstimate = 0;
        recoveryRequests = 0;
        ahrsSkipped = 0;
        accelDisabled = 0;
        fifoOverrunAtStart = 0;
        fifoFullAtStart = 0;
        fifoUnknownAtStart = 0;
        fifoHwTsAtStart = 0;
        fifoFbTsAtStart = 0;

        magEnabledAtStart = false;
        magYawApplyEnabledAtStart = false;
        magRefValidAtStart = false;

        magErrorStartDeg = 0.0f;
        magErrorEndDeg = 0.0f;
        magErrorAbsMaxDeg = 0.0f;
        magErrorAbsSumDeg = 0.0;
        magErrorSamples = 0;

        magTrustedAtStart = 0;
        magRejectedAtStart = 0;

        magHeadingValidAtStart = 0;
        magHeadingRejectedAtStart = 0;

        magYawUpdatesAtStart = 0;
        magYawGateOpenAtStart = 0;
        magYawGateClosedAtStart = 0;
        magYawApplyAllowedAtStart = 0;
        magYawAppliedAtStart = 0;

        magYawRejectNoReferenceAtStart = 0;
        magYawRejectHeadingInvalidAtStart = 0;
        magYawRejectMagNotTrustedAtStart = 0;
        magYawRejectMagStaleAtStart = 0;
        magYawRejectHorizontalBadAtStart = 0;
        magYawRejectInnovationTooLargeAtStart = 0;
        magYawRejectGyroMovingAtStart = 0;
        magYawRejectAccelNotTrustedAtStart = 0;

        dtUs.reset();
        accelNormG.reset();
        accelTrust.reset();
        tempC.reset();
        gyroAfterRadS.reset();
        magHeadingHorizontalNorm.reset();
        magYawCombinedTrust.reset();
        magYawCorrectionRateDegS.reset();
        magYawCorrectionStepDeg.reset();
        poseCaptured = false;
        eulerStartDeg = Vec3::zero();
        eulerEndDeg = Vec3::zero();
        qStart = Quat::identity();
        qEnd = Quat::identity();
    }
};

static StaticRuntimeTest g_staticTest;
static StaticRuntimeTest g_lastCompletedStaticTest;
static bool g_lastCompletedStaticTestValid = false;
static uint32_t g_lastCompletedStaticTestFinishedMs = 0;

// ============================================================
// Utility
// ============================================================

void IRAM_ATTR onFifoInt1() {
    g_fifoLastIrqUs = micros();
    g_fifoIntCount++;
}

static const char* errorName(Lsm6dsv::Error e) {
    switch (e) {
        case Lsm6dsv::Error::None:           return "None";
        case Lsm6dsv::Error::BusReadFailed:  return "BusReadFailed";
        case Lsm6dsv::Error::BusWriteFailed: return "BusWriteFailed";
        case Lsm6dsv::Error::WrongWhoAmI:    return "WrongWhoAmI";
        case Lsm6dsv::Error::ResetTimeout:   return "ResetTimeout";
        case Lsm6dsv::Error::InvalidConfig:  return "InvalidConfig";
    }
    return "Unknown";
}

static float angleDiffDeg(float a, float b) {
    return wrapPi((b - a) * MATH_DEG_TO_RAD) * MATH_RAD_TO_DEG;
}

static void resetFifoRuntimeCounters() {
    noInterrupts();
    g_fifoIntCount = 0;
    g_fifoLastIrqUs = micros();
    interrupts();

    g_lastHandledFifoIntCount = 0;
    g_fifoIntMissed = 0;
    g_fifoStatusFallbackEvents = 0;
    g_fifoWaitTimeouts = 0;
    g_lastSampleTimestampUs = 0;
}

static bool consumeFifoInterruptEvent(uint32_t timeoutMs) {
    const uint32_t startMs = millis();

    do {
        noInterrupts();
        const uint32_t current = g_fifoIntCount;
        interrupts();

        if (current != g_lastHandledFifoIntCount) {
            const uint32_t delta = current - g_lastHandledFifoIntCount;
            if (delta > 1) g_fifoIntMissed += delta - 1;
            g_lastHandledFifoIntCount = current;
            return true;
        }

        if (timeoutMs == 0) break;
        yield();
    } while (millis() - startMs < timeoutMs);

    // Lightweight fallback: only check FIFO_STATUS after timeout or explicit nonblocking call.
    Lsm6dsvFifoReader::Status st;
    if (lsmFifo.readStatus(st)) {
        if (st.unreadWords >= g_config.data.fifo.watermarkWords || st.overrun || st.full || st.overrunLatched) {
            g_fifoStatusFallbackEvents++;
            return true;
        }
    }

    if (timeoutMs > 0) g_fifoWaitTimeouts++;
    return false;
}

static bool waitFifoEventForCalibration(uint32_t timeoutMs, void* user) {
    (void)user;
    return consumeFifoInterruptEvent(timeoutMs);
}

static void updateLatestTemperatureFromFifo() {
    const auto& fs = lsmFifo.stats();
    if (fs.latestTempValid) {
        g_latestTempC = fs.latestTempC;
    }
}

static Lsm6dsv::Sample makeCalibratedSample(const Lsm6dsv::Sample& scaled) {
    Lsm6dsv::Sample calibrated = scaled;
    calibrated.temp_c = g_latestTempC;

    if (g_gyroTempComp.valid()) {
        calibrated.gyro_rad_s = g_gyroTempComp.correctedGyro(scaled.gyro_rad_s, g_latestTempC);
    } else if (g_imuCal.gyroBiasValid) {
        calibrated.gyro_rad_s = scaled.gyro_rad_s - g_imuCal.gyroBiasRadS;
    }

    if (g_imuCal.accelCalValid) {
        calibrated.accel_g = g_imuCal.applyAccel(scaled.accel_g);
    }

    return calibrated;
}

// ============================================================
// Config / init
// ============================================================

static void enforceProductCalibrationValidityAtBoot() {
    // Production rule: never seed a universal accel calibration into a new
    // device. A missing accel calibration must remain explicit so mag yaw
    // correction cannot silently run with another device's tilt calibration.
    if (!g_config.data.accelCal.valid) {
        g_config.data.accelCal.biasG = Vec3::zero();
        g_config.data.accelCal.scale = Mat3::identity();
        g_config.data.magYaw.applyEnabled = false;
    }

    if (!g_config.data.accelCal.valid || !g_config.data.magCal.calibrationValid) {
        g_config.data.magYaw.applyEnabled = false;
    }

    g_config.updateCrc();
}

static bool loadConfigAndApplyRuntime() {
    g_configStore.loadOrDefaults(g_config, &g_configLoadedFromNvs);
    g_config.sanitize();
    enforceProductCalibrationValidityAtBoot();

    g_config.applyToImuCalibration(g_imuCal);
    g_config.applyToGyroTempComp(g_gyroTempComp);
    g_ahrs6dof.setConfig(g_config.makeAhrsConfig());

    g_quality.setConfig(g_config.makeQualityConfig());
    g_quality.reset();

    g_streamState.rateHz = g_config.data.output.outputRateHz;
    g_streamState.mode = g_config.data.output.quaternionOutputEnabled ? TrackerStreamMode::Quat : TrackerStreamMode::Off;
    return true;
}

static bool initLsm() {
    SPI.begin(PIN_LSM_SCK, PIN_LSM_MISO, PIN_LSM_MOSI, PIN_LSM_CS);
    lsmBus.begin();

    Lsm6dsv::Config cfg = g_config.makeLsmConfig();

    if (!lsm.begin(cfg)) {
        Serial.print("# ERR LSM6DSV init failed error=");
        Serial.print(errorName(lsm.lastError()));
        Serial.print(" who=0x");
        Serial.println(lsm.lastWhoAmI(), HEX);
        return false;
    }

    uint8_t who = 0;
    lsm.readWhoAmI(who);

    Serial.println("# OK LSM6DSV init");
    Serial.print("# WHO_AM_I=0x"); Serial.println(who, HEX);
    Serial.print("# ODR_Hz="); Serial.println(Lsm6dsv::odrHz(g_config.data.imu.imuOdr), 3);
    return true;
}

static bool initFifo() {
    Lsm6dsvFifoReader::Config fifoCfg = g_config.makeFifoConfig();
    fifoCfg.enableSensorHubSlave0 = g_config.data.magCal.driverEnabled;
    fifoCfg.sensorHubSlave0PeriodUs = g_config.data.magCal.driverEnabled ? MAG_HUB_PERIOD_US : 0.0f;

    if (!lsmFifo.configure(fifoCfg)) {
        Serial.println("# ERR FIFO configure failed");
        return false;
    }

    Lsm6dsvFifoReader::Status st;
    lsmFifo.readStatus(st);

    const auto& fs = lsmFifo.stats();
    Serial.println("# OK FIFO v2 init");
    Serial.print("# fifo_watermark_words="); Serial.println(g_config.data.fifo.watermarkWords);
    Serial.print("# fifo_initial_unread_words="); Serial.println(st.unreadWords);
    Serial.print("# timestamp_tick_us="); Serial.println(fs.timestampTickUs, 6);
    Serial.print("# sample_period_us="); Serial.println(fs.samplePeriodUs, 3);
    Serial.print("# mag_fifo_parser="); Serial.println(fifoCfg.enableSensorHubSlave0 ? "on" : "off");
    Serial.print("# internal_freq_fine="); Serial.println(fs.internalFreqFine);
    return true;
}

static bool initSensorHubForMag() {
    if (g_magState.hubInitialized) return true;

    Lsm6dsvSensorHub::Config hubCfg;
    hubCfg.resetMasterOnBegin = true;
    hubCfg.enableInternalShubPullups = true;
    hubCfg.forceDisablePrimaryI2cI3c = true;
    hubCfg.transactionOdr = Lsm6dsvSensorHub::ShubOdr::Hz120;
    hubCfg.transactionTimeoutMs = 150;

    if (!lsmHub.begin(hubCfg)) {
        Serial.print("# ERR sensor hub init failed error=");
        Serial.println(lsmHub.lastErrorName());
        g_magState.hubInitialized = false;
        return false;
    }

    g_magState.hubInitialized = true;
    Serial.println("# OK LSM6DSV sensor hub init for QMC6309");
    return true;
}

static void resetMagRuntimeCounters() {
    g_magState.samples = 0;
    g_magState.queuePops = 0;
    g_magState.lastSampleMs = 0;
    g_magState.nacksSeen = lsmFifo.stats().sensorHubNackWords;
    g_magState.lastRaw = Lsm6dsvFifoReader::MagRawSample{};
    g_magState.lastNormRaw = 0.0f;

    g_magProcessor.reset();
    g_lastMagProcessed = MagProcessedSample{};

    g_magHeading.reset();
    g_lastMagHeading = MagHeadingSample{};

    g_magYawCorrection.reset();
    g_lastMagYawCorrection = MagYawCorrectionOutput{};

    g_magHeadingRef.clear();
    g_magHeadingAutoRef.resetAll();
}

static bool reconfigureFifoForCurrentMagConfig(uint64_t keepTimestampUs) {
    Lsm6dsvFifoReader::Config fifoCfg = g_config.makeFifoConfig();
    fifoCfg.enableSensorHubSlave0 = g_config.data.magCal.driverEnabled;
    fifoCfg.sensorHubSlave0PeriodUs = g_config.data.magCal.driverEnabled ? MAG_HUB_PERIOD_US : 0.0f;

    if (!lsmFifo.configure(fifoCfg)) {
        Serial.println("# ERR FIFO reconfigure for mag failed");
        return false;
    }

    lsmFifo.resetTimestampReconstruction(keepTimestampUs);
    if (g_quality.counters().samples > 0) {
        g_quality.reset();
        g_quality.syncFifoStats(lsmFifo.stats());
    }
    resetFifoRuntimeCounters();
    return true;
}

static bool setMagRuntimeEnabledHook(bool enabled, bool persist, void* user) {
    (void)user;

    const uint64_t keepTs = lsmFifo.stats().lastAssignedTimestampUs != 0
        ? lsmFifo.stats().lastAssignedTimestampUs
        : g_lastSampleTimestampUs;

    if (!enabled) {
        g_config.data.magCal.driverEnabled = false;
        g_config.updateCrc();
        lsmHub.stopMaster();
        g_magState.runtimeEnabled = false;
        g_magState.fifoArmed = false;
        g_magState.lastInitOk = true;
        reconfigureFifoForCurrentMagConfig(keepTs);
        resetMagRuntimeCounters();

        if (persist && !g_configStore.save(g_config)) {
            Serial.print("# ERR mag disable save failed: ");
            Serial.println(g_configStore.lastErrorName());
            return false;
        }
        return true;
    }

    g_config.data.magCal.driverEnabled = true;
    g_config.updateCrc();

    if (!initSensorHubForMag()) {
        g_magState.lastInitOk = false;
        g_magState.enableFailures++;
        return false;
    }

    if (!qmc.configureNormal100Hz()) {
        Serial.print("# ERR QMC6309 init100 failed qmcErr=");
        Serial.print(qmc.lastErrorName());
        Serial.print(" hubErr=");
        Serial.println(lsmHub.lastErrorName());
        g_magState.lastInitOk = false;
        g_magState.enableFailures++;
        return false;
    }

    if (!reconfigureFifoForCurrentMagConfig(keepTs)) {
        g_magState.lastInitOk = false;
        g_magState.enableFailures++;
        return false;
    }

    if (!qmc.armHubFifoRead(Lsm6dsvSensorHub::ShubOdr::Hz60)) {
        Serial.print("# ERR QMC6309 arm FIFO failed qmcErr=");
        Serial.print(qmc.lastErrorName());
        Serial.print(" hubErr=");
        Serial.println(lsmHub.lastErrorName());
        g_magState.lastInitOk = false;
        g_magState.enableFailures++;
        return false;
    }

    g_magState.runtimeEnabled = true;
    g_magState.fifoArmed = true;
    g_magState.lastInitOk = true;
    g_magState.lastEnableMs = millis();
    resetMagRuntimeCounters();

    if (persist && !g_configStore.save(g_config)) {
        Serial.print("# ERR mag enable save failed: ");
        Serial.println(g_configStore.lastErrorName());
        return false;
    }

    Serial.println("# OK QMC6309 -> LSM6DSV FIFO enabled: SLV0 0x01..0x06 @60Hz");
    return true;
}

static float magHeadingErrorToReferenceRad(const MagHeadingSample& heading) {
    if (!g_magHeadingRef.valid || !heading.valid) {
        return 0.0f;
    }
    return wrapPi(heading.magneticNorthWorldYawRad - g_magHeadingRef.worldYawRad);
}

static float magHeadingErrorToReferenceDeg(const MagHeadingSample& heading) {
    return magHeadingErrorToReferenceRad(heading) * MATH_RAD_TO_DEG;
}

static MagYawCorrectionConfig makeMagYawCorrectionConfig() {
    const auto& y = g_config.data.magYaw;

    MagYawCorrectionConfig c;
    const bool accelCalReady = g_imuCal.accelCalValid;
    const bool recoverySafe = !g_trackingRecovering;

    c.enabled = y.controllerEnabled &&
                g_config.data.magCal.driverEnabled &&
                g_config.data.magCal.calibrationValid &&
                accelCalReady &&
                recoverySafe;
    c.applyEnabled = y.applyEnabled && accelCalReady && recoverySafe;

    c.maxInnovationDeg = y.maxInnovationDeg;
    c.maxMagAgeMs = y.maxMagAgeMs;

    c.horizontalNormGood = y.horizontalNormGood;
    c.horizontalNormBad = y.horizontalNormBad;

    c.gyroNormGoodDps = y.gyroNormGoodDps;
    c.gyroNormBadDps = y.gyroNormBadDps;

    c.accelTrustGood = y.accelTrustGood;
    c.accelTrustBad = y.accelTrustBad;
    c.requireAccelTrusted = y.requireAccelTrusted;

    c.timeConstantS = y.timeConstantS;
    c.maxCorrectionRateDegS = y.maxCorrectionRateDegS;
    c.maxCorrectionStepDeg = y.maxCorrectionStepDeg;
    c.fallbackDtS = 1.0f / 60.0f;

    c.gyroMovingCooldownMs = 1000;
    c.accelBadCooldownMs = 750;
    c.magDisturbanceCooldownMs = 3000;

    return c;
}

static MagHeadingConfig makeMagHeadingConfig() {
    MagHeadingConfig c;
    c.requireTrustedMag = true;
    c.minHorizontalNorm = 1.0e-6f;
    return c;
}

static MagRuntimeConfig makeMagRuntimeConfig() {
    MagRuntimeConfig c;
    c.enabled = g_config.data.magCal.driverEnabled;
    c.calibrationValid = g_config.data.magCal.calibrationValid;
    c.axisAlignmentValid = g_config.data.magCal.axisAlignmentValid;

    c.hardIron = g_config.data.magCal.hardIron;
    c.softIron = g_config.data.magCal.softIron;
    c.magToImu = g_config.data.magCal.magToImu;

    c.expectedFieldNorm = g_config.data.magCal.expectedFieldNorm;
    c.minTrustNorm = g_config.data.magCal.minTrustNorm;
    c.maxTrustNorm = g_config.data.magCal.maxTrustNorm;

    c.maxSampleAgeMs = 250;
    c.minUsableNorm = 1.0e-6f;
    return c;
}

static float magRawNorm(const Lsm6dsvFifoReader::MagRawSample& m) {
    return std::sqrt(static_cast<float>(m.x) * static_cast<float>(m.x) +
                     static_cast<float>(m.y) * static_cast<float>(m.y) +
                     static_cast<float>(m.z) * static_cast<float>(m.z));
}

static float magYawRampUpLocal(float x, float bad, float good) {
    if (x >= good) return 1.0f;
    if (x <= bad) return 0.0f;
    if (good <= bad) return 0.0f;
    return (x - bad) / (good - bad);
}

static void resetMagYawCorrectionRuntime() {
    g_magYawCorrection.reset();
    g_lastMagYawCorrection = MagYawCorrectionOutput{};
}

static void resetOrientationDependentState(const char* reason, uint64_t timestampUs, bool rebaseAhrsTimebase) {
    g_magHeading.reset();
    g_lastMagHeading = MagHeadingSample{};

    g_magHeadingRef.clear();
    g_magHeadingAutoRef.done = false;
    g_magHeadingAutoRef.resetCandidate();

    resetMagYawCorrectionRuntime();

    if (rebaseAhrsTimebase && timestampUs != 0) {
        g_ahrs6dof.rebaseTimestamp(timestampUs);
    }

    Serial.print("# TRACKING orientation-dependent state reset");
    if (reason && reason[0] != '\0') {
        Serial.print(" reason=");
        Serial.print(reason);
    }
    if (timestampUs != 0) {
        Serial.print(" t_us=");
        Serial.print(static_cast<unsigned long>(timestampUs));
    }
    Serial.println();
}

static void enterTrackingRecovery(uint32_t reasonFlags, const char* reason, uint64_t timestampUs) {
    g_trackingRecovering = true;
    g_trackingRecoveryStableSamples = 0;
    g_trackingRecoveryEnterCount++;
    g_trackingRecoveryLastFlags = reasonFlags;
    g_trackingRecoveryLastTimestampUs = timestampUs;

    resetOrientationDependentState(reason, timestampUs, true);

    Serial.print("# TRACKING state=RECOVERING");
    if (reason && reason[0] != '\0') {
        Serial.print(" reason=");
        Serial.print(reason);
    }
    Serial.print(" flags=0x");
    Serial.println(reasonFlags, HEX);
}

static void updateTrackingRecoveryState(const ImuQualityResult& quality) {
    if (!g_trackingRecovering) return;

    const bool stable = quality.shouldUpdateAhrs &&
                        !quality.shouldRequestFifoRecovery &&
                        !quality.has(imu_quality_flags::TIMESTAMP_ZERO) &&
                        !quality.has(imu_quality_flags::TIMESTAMP_NON_MONOTONIC) &&
                        !quality.has(imu_quality_flags::TIMESTAMP_BACKWARDS) &&
                        !quality.has(imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW) &&
                        !quality.has(imu_quality_flags::TIMESTAMP_LARGE_GAP) &&
                        !quality.has(imu_quality_flags::FIFO_OVERRUN) &&
                        !quality.has(imu_quality_flags::FIFO_FULL) &&
                        !quality.has(imu_quality_flags::FIFO_UNKNOWN_TAG);

    if (!stable) {
        g_trackingRecoveryStableSamples = 0;
        return;
    }

    g_trackingRecoveryStableSamples++;
    if (g_trackingRecoveryStableSamples >= TRACKING_RECOVERY_STABLE_SAMPLES) {
        g_trackingRecovering = false;
        g_trackingRecoveryStableSamples = 0;
        Serial.println("# TRACKING state=TRACKING_6DOF reason=recovery_stable");
    }
}

static const char* trackingStateName() {
    if (!g_imuCal.accelCalValid || !g_imuCal.gyroBiasValid) return "CALIBRATION_REQUIRED";
    if (g_trackingRecovering) return "RECOVERING";
    if (g_quality.recoveryRequested()) return "DEGRADED_TIMING";
    if (!g_ahrs6dof.initialized()) return "STARTUP_CONVERGENCE";
    if (g_magHeadingRef.valid && g_lastMagYawCorrection.applied) return "TRACKING_6DOF_MAG_YAW";
    return "TRACKING_6DOF";
}

static bool setMagHeadingReferenceInternal(const char* reason, bool verbose) {
    if (!g_lastMagHeading.valid) {
        if (verbose) Serial.println("# ERR cannot set mag heading reference: last heading is invalid");
        return false;
    }

    if (!MagRuntimeProcessor::trustedForUse(g_lastMagProcessed, makeMagRuntimeConfig(), millis())) {
        if (verbose) Serial.println("# ERR cannot set mag heading reference: last mag is not trusted");
        return false;
    }

    g_magHeadingRef.valid = true;
    g_magHeadingRef.worldYawRad = g_lastMagHeading.magneticNorthWorldYawRad;
    g_magHeadingRef.setMs = millis();
    g_magHeadingRef.magSeq = g_lastMagHeading.magSeq;
    g_magHeadingRef.magTimestampUs = g_lastMagHeading.magTimestampUs;

    g_magHeadingAutoRef.done = true;
    g_magHeadingAutoRef.setCount++;
    g_magHeadingAutoRef.lastSetMs = millis();
    g_magHeadingAutoRef.resetCandidate();

    // Important: a new reference invalidates previous controller timing/stats.
    resetMagYawCorrectionRuntime();

    if (verbose) {
        Serial.print("# OK mag heading ref");
        if (reason && reason[0] != '\0') {
            Serial.print(" reason=");
            Serial.print(reason);
        }
        Serial.print(" world_yaw_deg=");
        Serial.println(g_magHeadingRef.worldYawDeg(), 6);
    }

    return true;
}

static void updateMagHeadingAutoReference(uint32_t nowMs,
                                          float gyroNormDps,
                                          float accelTrust,
                                          bool magTrustedForUse) {
    if (!g_magHeadingAutoRef.enabled) return;
    if (g_trackingRecovering) {
        g_magHeadingAutoRef.resetCandidate();
        return;
    }

    // Already have a reference for this AHRS world-frame.
    if (g_magHeadingRef.valid) {
        g_magHeadingAutoRef.done = true;
        return;
    }

    const MagYawCorrectionConfig yawCfg = makeMagYawCorrectionConfig();

    uint32_t reject = 0;

    if (!yawCfg.enabled || !yawCfg.applyEnabled) {
        reject |= 1u << 0;
    }

    if (!g_lastMagHeading.valid) {
        reject |= 1u << 1;
    }

    if (!magTrustedForUse) {
        reject |= 1u << 2;
    }

    if (!tracker::isFinite(gyroNormDps) || gyroNormDps > 1.5f) {
        reject |= 1u << 3;
    }

    if (!tracker::isFinite(accelTrust) || accelTrust < 0.90f) {
        reject |= 1u << 4;
    }

    const float hTrust = magYawRampUpLocal(
        g_lastMagHeading.horizontalNorm,
        yawCfg.horizontalNormBad,
        yawCfg.horizontalNormGood
    );

    // Auto-ref only needs heading to be usable, not perfect.
    // Continuous yaw correction still uses full combinedTrust ramp later.
    if (!tracker::isFinite(hTrust) || hTrust < 0.25f) {
        reject |= 1u << 5;
    }

    g_magHeadingAutoRef.lastRejectFlags = reject;
    g_magHeadingAutoRef.lastGyroNormDps = gyroNormDps;
    g_magHeadingAutoRef.lastAccelTrust = accelTrust;
    g_magHeadingAutoRef.lastHorizontalTrust = hTrust;

    if (reject != 0) {
        g_magHeadingAutoRef.stableSinceMs = 0;
        return;
    }

    if (g_magHeadingAutoRef.stableSinceMs == 0) {
        g_magHeadingAutoRef.stableSinceMs = nowMs;
        return;
    }

    const uint32_t stableMs = nowMs - g_magHeadingAutoRef.stableSinceMs;
    if (stableMs >= 3000) {
        setMagHeadingReferenceInternal("auto", true);
    }
}

static bool applyMagYawCorrectionToAhrs(const MagYawCorrectionOutput& yaw) {
    if (!yaw.applyAllowed || yaw.correctionStepRad == 0.0f) {
        return false;
    }

    if (!tracker::isFinite(yaw.correctionStepRad)) {
        return false;
    }

    const Vec3 worldAxis = g_ahrs6dof.config().worldUp.normalized();
    if (!worldAxis.isFinite() || worldAxis.normSq() < MATH_EPSILON) {
        return false;
    }

    const Vec3 correctionWorldRad = worldAxis * yaw.correctionStepRad;
    const Quat corrected = applyWorldCorrection(g_ahrs6dof.quaternion(), correctionWorldRad).withPositiveW();

    if (!corrected.isFinite()) {
        return false;
    }

    g_ahrs6dof.setQuaternion(corrected);
    return true;
}

static void processOneMagRawSample(const Lsm6dsvFifoReader::MagRawSample& mag) {
    g_magState.samples++;
    g_magState.queuePops++;
    g_magState.lastSampleMs = millis();
    g_magState.lastRaw = mag;
    g_magState.lastNormRaw = magRawNorm(mag);

    g_magCalCollector.push(mag, g_magState.lastNormRaw, millis());

    MagProcessedSample processed;
    g_magProcessor.process(mag, makeMagRuntimeConfig(), millis(), processed);
    g_lastMagProcessed = processed;

    MagHeadingSample heading;
    g_magHeading.update(
        g_lastMagProcessed,
        g_ahrs6dof.quaternionPositiveW(),
        makeMagHeadingConfig(),
        millis(),
        heading
    );
    g_lastMagHeading = heading;

    const uint32_t nowMs = millis();
    const MagRuntimeConfig magCfg = makeMagRuntimeConfig();
    const Ahrs6DofStats& ahrsStats = g_ahrs6dof.stats();

    const float gyroNormDps = ahrsStats.lastGyroRadS.norm() * MATH_RAD_TO_DEG;
    const float accelTrust = ahrsStats.lastAccelGate.trust;

    const bool magTrustedForUse =
        MagRuntimeProcessor::trustedForUse(g_lastMagProcessed, magCfg, nowMs);
    const uint32_t magRejectFlagsForUse =
        MagRuntimeProcessor::rejectFlagsForUse(g_lastMagProcessed, magCfg, nowMs);

    updateMagHeadingAutoReference(nowMs, gyroNormDps, accelTrust, magTrustedForUse);

    MagYawCorrectionInput yawIn;
    yawIn.mag = g_lastMagProcessed;
    yawIn.heading = g_lastMagHeading;
    yawIn.referenceValid = g_magHeadingRef.valid;
    yawIn.referenceWorldYawRad = g_magHeadingRef.worldYawRad;
    yawIn.magTrustedForUse = magTrustedForUse;
    yawIn.magRejectFlagsForUse = magRejectFlagsForUse;
    yawIn.gyroNormDps = gyroNormDps;
    yawIn.accelTrust = accelTrust;
    yawIn.nowMs = nowMs;

    MagYawCorrectionOutput yawOut;
    g_magYawCorrection.update(yawIn, makeMagYawCorrectionConfig(), yawOut);

    if (applyMagYawCorrectionToAhrs(yawOut)) {
        yawOut.applied = true;
        g_magYawCorrection.markApplied(yawOut.correctionStepDeg);
    }

    g_lastMagYawCorrection = yawOut;

    if (g_staticTest.active && g_magHeadingRef.valid && g_lastMagHeading.valid) {
        const float e = magHeadingErrorToReferenceDeg(g_lastMagHeading);
        if (std::isfinite(e)) {
            g_staticTest.magErrorEndDeg = e;

            const float ae = std::fabs(e);
            g_staticTest.magErrorAbsSumDeg += static_cast<double>(ae);
            g_staticTest.magErrorSamples++;

            if (g_staticTest.magErrorSamples == 1 || ae > g_staticTest.magErrorAbsMaxDeg) {
                g_staticTest.magErrorAbsMaxDeg = ae;
            }
        }
    }

    if (g_staticTest.active) {
        if (g_lastMagHeading.valid && std::isfinite(g_lastMagHeading.horizontalNorm)) {
            g_staticTest.magHeadingHorizontalNorm.push(g_lastMagHeading.horizontalNorm);
        }

        if (yawOut.valid) {
            if (std::isfinite(yawOut.combinedTrust)) {
                g_staticTest.magYawCombinedTrust.push(yawOut.combinedTrust);
            }

            if (std::isfinite(yawOut.correctionRateDegS)) {
                g_staticTest.magYawCorrectionRateDegS.push(yawOut.correctionRateDegS);
            }

            if (std::isfinite(yawOut.correctionStepDeg)) {
                g_staticTest.magYawCorrectionStepDeg.push(yawOut.correctionStepDeg);
            }
        }
    }

    const uint32_t nacks = lsmFifo.stats().sensorHubNackWords;
    if (nacks != g_magState.nacksSeen) {
        g_magState.nacksSeen = nacks;
    }
}

static void printMagRuntimeStatus(Stream& out, void* user) {
    (void)user;
    out.print("mag_runtime_enabled="); out.println(g_magState.runtimeEnabled ? "yes" : "no");
    out.print("mag_hub_initialized="); out.println(g_magState.hubInitialized ? "yes" : "no");
    out.print("mag_fifo_armed="); out.println(g_magState.fifoArmed ? "yes" : "no");
    out.print("mag_last_init_ok="); out.println(g_magState.lastInitOk ? "yes" : "no");
    out.print("mag_enable_failures="); out.println(g_magState.enableFailures);
    out.print("mag_runtime_samples="); out.println(g_magState.samples);
    out.print("mag_last_age_ms="); out.println(g_magState.lastSampleMs == 0 ? 0UL : millis() - g_magState.lastSampleMs);
    out.print("mag_last_t_us="); out.println(static_cast<unsigned long>(g_magState.lastRaw.t_us));
    out.print("mag_last_xyz=");
    out.print(g_magState.lastRaw.x); out.print(',');
    out.print(g_magState.lastRaw.y); out.print(',');
    out.println(g_magState.lastRaw.z);
    out.print("mag_last_norm_raw="); out.println(g_magState.lastNormRaw, 3);
    out.print("mag_last_flags=0x"); out.println(g_magState.lastRaw.flags, HEX);
    out.print("mag_qmc_error="); out.println(qmc.lastErrorName());
    out.print("mag_hub_error="); out.println(lsmHub.lastErrorName());
}

static void printMagProcessedStatus(Stream& out, void* user) {
    (void)user;

    const MagProcessedSample& m = g_lastMagProcessed;
    const MagRuntimeStats& s = g_magProcessor.stats();
    const MagRuntimeConfig cfg = makeMagRuntimeConfig();

    const uint32_t nowMs = millis();
    const uint32_t ageMs = MagRuntimeProcessor::ageMsForUse(m, nowMs);
    const uint32_t rejectFlagsNow = MagRuntimeProcessor::rejectFlagsForUse(m, cfg, nowMs);
    const bool trustedNow = MagRuntimeProcessor::trustedForUse(m, cfg, nowMs);

    out.println("# MAG PROCESSED");

    out.print("processed_valid="); out.println(m.valid ? "yes" : "no");
    out.print("trusted="); out.println(trustedNow ? "yes" : "no");
    out.print("reject_flags=0x"); out.println(rejectFlagsNow, HEX);
    out.print("process_reject_flags=0x"); out.println(m.rejectFlags, HEX);
    out.print("seq="); out.println(m.seq);
    out.print("age_ms="); out.println(ageMs);
    out.print("received_ms="); out.println(m.receivedMs);
    out.print("t_us="); out.println(static_cast<unsigned long>(m.t_us));

    out.print("raw=");
    out.print(m.raw.x, 3); out.print(',');
    out.print(m.raw.y, 3); out.print(',');
    out.println(m.raw.z, 3);

    out.print("calibrated_mag_frame=");
    out.print(m.calibratedMagFrame.x, 6); out.print(',');
    out.print(m.calibratedMagFrame.y, 6); out.print(',');
    out.println(m.calibratedMagFrame.z, 6);

    out.print("body=");
    out.print(m.body.x, 6); out.print(',');
    out.print(m.body.y, 6); out.print(',');
    out.println(m.body.z, 6);

    out.print("norms_raw_cal_body=");
    out.print(m.rawNorm, 6); out.print(',');
    out.print(m.calibratedNorm, 6); out.print(',');
    out.println(m.bodyNorm, 6);

    out.print("config_cal_valid="); out.println(g_config.data.magCal.calibrationValid ? "yes" : "no");
    out.print("config_axis_valid="); out.println(g_config.data.magCal.axisAlignmentValid ? "yes" : "no");

    out.print("trust_norm_min_max=");
    out.print(g_config.data.magCal.minTrustNorm, 6); out.print(',');
    out.println(g_config.data.magCal.maxTrustNorm, 6);

    out.println("# MAG TRUST STATS");
    out.print("raw_samples="); out.println(s.rawSamples);
    out.print("processed_samples="); out.println(s.processedSamples);
    out.print("trusted_samples="); out.println(s.trustedSamples);
    out.print("rejected_samples="); out.println(s.rejectedSamples);

    out.print("raw_norm_min_mean_max=");
    out.print(s.rawNormMin, 6); out.print(',');
    out.print(s.rawNormMean(), 6); out.print(',');
    out.println(s.rawNormMax, 6);

    out.print("body_norm_min_mean_max=");
    out.print(s.bodyNormMin, 6); out.print(',');
    out.print(s.bodyNormMean(), 6); out.print(',');
    out.println(s.bodyNormMax, 6);

    out.print("trusted_body_norm_min_mean_max=");
    out.print(s.trustedBodyNormMin, 6); out.print(',');
    out.print(s.trustedBodyNormMean(), 6); out.print(',');
    out.println(s.trustedBodyNormMax, 6);

    out.print("reject_disabled="); out.println(s.rejectedDisabled);
    out.print("reject_raw_saturated="); out.println(s.rejectedRawSaturated);
    out.print("reject_raw_nonfinite="); out.println(s.rejectedRawNonfinite);
    out.print("reject_not_calibrated="); out.println(s.rejectedNotCalibrated);
    out.print("reject_axis_not_aligned="); out.println(s.rejectedAxisNotAligned);
    out.print("reject_norm_too_low="); out.println(s.rejectedNormTooLow);
    out.print("reject_norm_too_high="); out.println(s.rejectedNormTooHigh);
    out.print("reject_stale_process_time="); out.println(s.rejectedStale);
    out.print("reject_zero_norm="); out.println(s.rejectedZeroNorm);
}

static void printMagHeadingStatus(Stream& out, void* user) {
    (void)user;

    const MagHeadingSample& h = g_lastMagHeading;
    const MagHeadingStats& s = g_magHeading.stats();

    const float errorToRefRad = magHeadingErrorToReferenceRad(h);
    const float errorToRefDeg = errorToRefRad * MATH_RAD_TO_DEG;

    out.println("# MAG HEADING");

    out.print("heading_valid=");
    out.println(h.valid ? "yes" : "no");

    out.print("heading_reject_flags=0x");
    out.println(h.rejectFlags, HEX);

    out.print("mag_seq=");
    out.println(h.magSeq);

    out.print("mag_t_us=");
    out.println(static_cast<unsigned long>(h.magTimestampUs));

    out.print("mag_received_ms=");
    out.println(h.magReceivedMs);

    out.print("mag_body=");
    out.print(h.magBody.x, 6); out.print(',');
    out.print(h.magBody.y, 6); out.print(',');
    out.println(h.magBody.z, 6);

    out.print("mag_world=");
    out.print(h.magWorld.x, 6); out.print(',');
    out.print(h.magWorld.y, 6); out.print(',');
    out.println(h.magWorld.z, 6);

    out.print("mag_world_horizontal=");
    out.print(h.magWorldHorizontal.x, 6); out.print(',');
    out.print(h.magWorldHorizontal.y, 6); out.print(',');
    out.println(h.magWorldHorizontal.z, 6);

    out.print("norms_body_world_horizontal=");
    out.print(h.magBodyNorm, 6); out.print(',');
    out.print(h.magWorldNorm, 6); out.print(',');
    out.println(h.horizontalNorm, 6);

    out.print("magnetic_field_world_yaw_deg=");
    out.println(h.magneticFieldWorldYawDeg, 6);

    out.print("magnetic_north_world_yaw_deg=");
    out.println(h.magneticNorthWorldYawDeg, 6);

    out.print("ahrs_yaw_deg=");
    out.println(h.currentAhrsYawDeg, 6);

    // Old diagnostic, kept for visibility but do not use it for yaw correction.
    out.print("north_minus_ahrs_yaw_deg=");
    out.println(h.yawInnovationDeg, 6);

    out.println("# MAG HEADING REFERENCE");

    out.print("mag_ref_valid=");
    out.println(g_magHeadingRef.valid ? "yes" : "no");

    out.print("mag_ref_world_yaw_deg=");
    out.println(g_magHeadingRef.valid ? g_magHeadingRef.worldYawDeg() : 0.0f, 6);

    out.print("mag_ref_age_ms=");
    out.println(g_magHeadingRef.valid ? millis() - g_magHeadingRef.setMs : 0UL);

    out.print("mag_ref_seq=");
    out.println(g_magHeadingRef.magSeq);

    out.print("mag_error_to_ref_deg=");
    out.println(g_magHeadingRef.valid && h.valid ? errorToRefDeg : 0.0f, 6);

    out.print("mag_error_to_ref_rad=");
    out.println(g_magHeadingRef.valid && h.valid ? errorToRefRad : 0.0f, 9);

    out.println("# MAG HEADING AUTO REFERENCE");

    out.print("mag_auto_ref_enabled=");
    out.println(g_magHeadingAutoRef.enabled ? "yes" : "no");

    out.print("mag_auto_ref_done=");
    out.println(g_magHeadingAutoRef.done ? "yes" : "no");

    out.print("mag_auto_ref_stable_ms=");
    out.println(g_magHeadingAutoRef.stableSinceMs == 0 ? 0UL : millis() - g_magHeadingAutoRef.stableSinceMs);

    out.print("mag_auto_ref_set_count=");
    out.println(g_magHeadingAutoRef.setCount);

    out.print("mag_auto_ref_last_set_age_ms=");
    out.println(g_magHeadingAutoRef.lastSetMs == 0 ? 0UL : millis() - g_magHeadingAutoRef.lastSetMs);

    out.print("mag_auto_ref_last_reject_flags=0x");
    out.println(g_magHeadingAutoRef.lastRejectFlags, HEX);

    out.print("mag_auto_ref_last_gyro_norm_dps=");
    out.println(g_magHeadingAutoRef.lastGyroNormDps, 6);

    out.print("mag_auto_ref_last_accel_trust=");
    out.println(g_magHeadingAutoRef.lastAccelTrust, 6);

    out.print("mag_auto_ref_last_horizontal_trust=");
    out.println(g_magHeadingAutoRef.lastHorizontalTrust, 6);

    out.println("# MAG HEADING STATS");

    out.print("heading_attempts=");
    out.println(s.attempts);

    out.print("heading_valid_count=");
    out.println(s.valid);

    out.print("heading_rejected_count=");
    out.println(s.rejected);

    out.print("reject_mag_invalid=");
    out.println(s.rejectMagInvalid);

    out.print("reject_mag_not_trusted=");
    out.println(s.rejectMagNotTrusted);

    out.print("reject_quat_invalid=");
    out.println(s.rejectQuatInvalid);

    out.print("reject_world_nonfinite=");
    out.println(s.rejectWorldNonfinite);

    out.print("reject_horizontal_small=");
    out.println(s.rejectHorizontalSmall);

    out.print("last_valid_age_ms=");
    out.println(s.lastValidMs == 0 ? 0UL : millis() - s.lastValidMs);
}

static void printMagYawCorrectionStatus(Stream& out, void* user) {
    (void)user;

    const MagYawCorrectionConfig cfg = makeMagYawCorrectionConfig();
    const MagYawCorrectionOutput& y = g_lastMagYawCorrection;
    const MagYawCorrectionStats& s = g_magYawCorrection.stats();

    out.println("# MAG YAW CORRECTION");

    out.print("enabled=");
    out.println(cfg.enabled ? "yes" : "no");

    out.print("apply_enabled=");
    out.println(cfg.applyEnabled ? "yes" : "no");

    out.print("gate_open=");
    out.println(y.gateOpen ? "yes" : "no");

    out.print("apply_allowed=");
    out.println(y.applyAllowed ? "yes" : "no");

    out.print("applied_last=");
    out.println(y.applied ? "yes" : "no");

    out.print("reject_flags=0x");
    out.println(y.rejectFlags, HEX);

    out.print("cooldown_active=");
    out.println(y.cooldownActive ? "yes" : "no");

    out.print("cooldown_remaining_ms=");
    out.println(y.cooldownRemainingMs);

    out.print("cooldown_reason_flags=0x");
    out.println(y.cooldownReasonFlags, HEX);

    out.print("mag_seq=");
    out.println(y.magSeq);

    out.print("mag_t_us=");
    out.println(static_cast<unsigned long>(y.magTimestampUs));

    out.print("mag_age_ms=");
    out.println(y.magAgeMs);

    out.print("dt_ms=");
    out.println(y.dtMs);

    out.print("horizontal_norm=");
    out.println(y.horizontalNorm, 6);

    out.print("horizontal_trust=");
    out.println(y.horizontalTrust, 6);

    out.print("gyro_norm_dps=");
    out.println(y.gyroNormDps, 6);

    out.print("gyro_trust=");
    out.println(y.gyroTrust, 6);

    out.print("accel_trust=");
    out.println(y.accelTrust, 6);

    out.print("accel_gate_trust=");
    out.println(y.accelGateTrust, 6);

    out.print("combined_trust=");
    out.println(y.combinedTrust, 6);

    out.print("error_to_ref_deg=");
    out.println(y.errorDeg, 6);

    out.print("error_to_ref_rad=");
    out.println(y.errorRad, 9);

    out.print("correction_rate_deg_s=");
    out.println(y.correctionRateDegS, 6);

    out.print("correction_rate_rad_s=");
    out.println(y.correctionRateRadS, 9);

    out.print("correction_step_deg=");
    out.println(y.correctionStepDeg, 6);

    out.print("correction_step_rad=");
    out.println(y.correctionStepRad, 9);

    out.println("# CONFIG");

    out.print("max_innovation_deg=");
    out.println(cfg.maxInnovationDeg, 3);

    out.print("horizontal_norm_bad_good=");
    out.print(cfg.horizontalNormBad, 3);
    out.print(',');
    out.println(cfg.horizontalNormGood, 3);

    out.print("gyro_norm_good_bad_dps=");
    out.print(cfg.gyroNormGoodDps, 3);
    out.print(',');
    out.println(cfg.gyroNormBadDps, 3);

    out.print("accel_trust_bad_good=");
    out.print(cfg.accelTrustBad, 3);
    out.print(',');
    out.println(cfg.accelTrustGood, 3);

    out.print("max_mag_age_ms=");
    out.println(cfg.maxMagAgeMs);

    out.print("time_constant_s=");
    out.println(cfg.timeConstantS, 3);

    out.print("max_rate_deg_s=");
    out.println(cfg.maxCorrectionRateDegS, 3);

    out.print("max_step_deg=");
    out.println(cfg.maxCorrectionStepDeg, 3);

    out.println("# STATS");

    out.print("updates=");
    out.println(s.updates);

    out.print("gate_open_count=");
    out.println(s.gateOpenCount);

    out.print("gate_closed_count=");
    out.println(s.gateClosedCount);

    out.print("apply_allowed_count=");
    out.println(s.applyAllowedCount);

    out.print("applied_count=");
    out.println(s.appliedCount);

    out.print("last_abs_error_deg=");
    out.println(s.lastAbsErrorDeg, 6);

    out.print("mean_abs_error_deg=");
    out.println(s.meanAbsErrorDeg(), 6);

    out.print("max_abs_error_deg=");
    out.println(s.maxAbsErrorDeg, 6);

    out.print("last_correction_step_deg=");
    out.println(s.lastCorrectionStepDeg, 6);

    out.print("max_abs_correction_step_deg=");
    out.println(s.maxAbsCorrectionStepDeg, 6);

    out.print("reject_disabled=");
    out.println(s.rejectDisabled);

    out.print("reject_apply_disabled=");
    out.println(s.rejectApplyDisabled);

    out.print("reject_no_reference=");
    out.println(s.rejectNoReference);

    out.print("reject_heading_invalid=");
    out.println(s.rejectHeadingInvalid);

    out.print("reject_mag_not_trusted=");
    out.println(s.rejectMagNotTrusted);

    out.print("reject_mag_stale=");
    out.println(s.rejectMagStale);

    out.print("reject_horizontal_bad=");
    out.println(s.rejectHorizontalBad);

    out.print("reject_innovation_too_large=");
    out.println(s.rejectInnovationTooLarge);

    out.print("reject_gyro_moving=");
    out.println(s.rejectGyroMoving);

    out.print("reject_accel_not_trusted=");
    out.println(s.rejectAccelNotTrusted);

    out.print("reject_cooldown=");
    out.println(s.rejectCooldown);

    out.print("reject_dt_invalid=");
    out.println(s.rejectDtInvalid);

    out.print("reject_nonfinite=");
    out.println(s.rejectNonfinite);
}

static void resetMagYawCorrectionHook(void* user) {
    (void)user;
    resetMagYawCorrectionRuntime();
}

static bool setMagYawCorrectionApplyEnabledHook(bool enabled, bool persist, void* user) {
    (void)user;

    g_config.data.magYaw.applyEnabled = enabled;
    g_config.sanitize();
    g_config.updateCrc();

    // Reset controller timing/stats when changing mode.
    g_magYawCorrection.reset();
    g_lastMagYawCorrection = MagYawCorrectionOutput{};

    if (persist) {
        if (!g_configStore.save(g_config)) {
            Serial.print("# ERR mag yaw correction save failed: ");
            Serial.println(g_configStore.lastErrorName());
            return false;
        }
    }

    Serial.print("# OK mag yaw correction apply=");
    Serial.println(enabled ? "enabled" : "disabled");
    return true;
}

static bool setMagHeadingReferenceHook(void* user) {
    (void)user;
    return setMagHeadingReferenceInternal("manual", true);
}

static void clearMagHeadingReferenceHook(void* user) {
    (void)user;
    g_magHeadingRef.clear();
    g_magHeadingAutoRef.done = false;
    g_magHeadingAutoRef.resetCandidate();
    resetMagYawCorrectionRuntime();
}

static bool setMagHeadingAutoReferenceEnabledHook(bool enabled, void* user) {
    (void)user;

    g_magHeadingAutoRef.enabled = enabled;

    if (!enabled) {
        g_magHeadingAutoRef.resetCandidate();
    } else if (!g_magHeadingRef.valid) {
        g_magHeadingAutoRef.done = false;
        g_magHeadingAutoRef.resetCandidate();
    }

    Serial.print("# OK mag heading auto-ref ");
    Serial.println(enabled ? "enabled" : "disabled");
    return true;
}

static void printMagCalibrationStatus(Stream& out, void* user) {
    (void)user;

    MagCalibrationResult result;
    const bool canCompute = g_magCalCollector.compute(result);

    out.println("# MAG CAL");
    out.print("mag_cal_active="); out.println(g_magCalCollector.active() ? "yes" : "no");
    out.print("mag_cal_samples="); out.println(g_magCalCollector.samples());
    out.print("mag_cal_rejected="); out.println(g_magCalCollector.rejected());
    out.print("mag_cal_saturated="); out.println(g_magCalCollector.saturated());
    out.print("mag_cal_elapsed_s=");
    out.println(g_magCalCollector.startMs() == 0 ? 0UL : (millis() - g_magCalCollector.startMs()) / 1000UL);
    out.print("mag_cal_last_age_ms=");
    out.println(g_magCalCollector.lastSampleMs() == 0 ? 0UL : millis() - g_magCalCollector.lastSampleMs());

    out.print("min_xyz=");
    out.print(g_magCalCollector.minX(), 3); out.print(',');
    out.print(g_magCalCollector.minY(), 3); out.print(',');
    out.println(g_magCalCollector.minZ(), 3);

    out.print("max_xyz=");
    out.print(g_magCalCollector.maxX(), 3); out.print(',');
    out.print(g_magCalCollector.maxY(), 3); out.print(',');
    out.println(g_magCalCollector.maxZ(), 3);

    out.print("span_xyz=");
    out.print(g_magCalCollector.spanX(), 3); out.print(',');
    out.print(g_magCalCollector.spanY(), 3); out.print(',');
    out.println(g_magCalCollector.spanZ(), 3);

    out.print("mean_xyz=");
    out.print(g_magCalCollector.meanX(), 3); out.print(',');
    out.print(g_magCalCollector.meanY(), 3); out.print(',');
    out.println(g_magCalCollector.meanZ(), 3);

    out.print("norm_min_mean_max=");
    out.print(g_magCalCollector.normMin(), 3); out.print(',');
    out.print(g_magCalCollector.normMean(), 3); out.print(',');
    out.println(g_magCalCollector.normMax(), 3);

    out.print("can_compute=");
    out.println(canCompute ? "yes" : "no");

    if (canCompute) {
        out.print("computed_hard_iron=");
        out.print(result.hardIron.x, 6); out.print(',');
        out.print(result.hardIron.y, 6); out.print(',');
        out.println(result.hardIron.z, 6);

        out.print("computed_soft_iron_diag=");
        out.print(result.softIron.m[0][0], 6); out.print(',');
        out.print(result.softIron.m[1][1], 6); out.print(',');
        out.println(result.softIron.m[2][2], 6);

        out.print("computed_expected_norm=");
        out.println(result.expectedNorm, 6);

        out.print("computed_radius_xyz=");
        out.print(result.radiusX, 3); out.print(',');
        out.print(result.radiusY, 3); out.print(',');
        out.println(result.radiusZ, 3);

        out.print("suggested_trust_norm_min_max=");
        out.print(result.minTrustNorm, 6); out.print(',');
        out.println(result.maxTrustNorm, 6);
    }

    out.println("# Rotate the whole final assembly through all orientations.");
    out.println("# Use: mag cal apply save");
}

static bool startMagCalibrationHook(void* user) {
    (void)user;
    if (!g_magState.runtimeEnabled || !g_magState.fifoArmed) {
        return false;
    }
    g_magCalCollector.start(millis());
    return true;
}

static void stopMagCalibrationHook(void* user) {
    (void)user;
    g_magCalCollector.stop();
}

static void resetMagCalibrationHook(void* user) {
    (void)user;
    g_magCalCollector.reset();
}

static bool applyMagCalibrationHook(bool persist, void* user) {
    (void)user;

    MagCalibrationResult result;
    if (!g_magCalCollector.compute(result)) {
        Serial.println("# ERR mag calibration compute failed");
        Serial.println("# Need more samples and wider 3-axis rotation coverage");
        return false;
    }

    g_config.data.magCal.calibrationValid = true;
    g_config.data.magCal.hardIron = result.hardIron;
    g_config.data.magCal.softIron = result.softIron;
    g_config.data.magCal.expectedFieldNorm = result.expectedNorm;
    g_config.data.magCal.minTrustNorm = result.minTrustNorm;
    g_config.data.magCal.maxTrustNorm = result.maxTrustNorm;
    g_config.updateCrc();

    if (persist) {
        if (!g_configStore.save(g_config)) {
            Serial.print("# ERR mag calibration save failed: ");
            Serial.println(g_configStore.lastErrorName());
            return false;
        }
    }

    resetOrientationDependentState("mag_calibration_changed", lsmFifo.stats().lastAssignedTimestampUs, false);

    Serial.print("# OK mag hardIron=");
    Serial.print(result.hardIron.x, 6); Serial.print(',');
    Serial.print(result.hardIron.y, 6); Serial.print(',');
    Serial.println(result.hardIron.z, 6);

    Serial.print("# OK mag softIronDiag=");
    Serial.print(result.softIron.m[0][0], 6); Serial.print(',');
    Serial.print(result.softIron.m[1][1], 6); Serial.print(',');
    Serial.println(result.softIron.m[2][2], 6);

    Serial.print("# OK mag expectedNorm=");
    Serial.println(result.expectedNorm, 6);

    return true;
}

static void setupCalibrationIo() {
    g_calIo.lsm = &lsm;
    g_calIo.fifo = &lsmFifo;
    g_calIo.rawBuffer = g_fifoRaw;
    g_calIo.rawBufferCapacity = FIFO_RAW_BUFFER_CAPACITY;
    g_calIo.maxWordsPerDrain = g_config.data.fifo.maxWordsPerDrain;
    g_calIo.maxDrainRoundsPerEvent = g_config.data.fifo.maxDrainRoundsPerEvent;
    g_calIo.waitForFifoEvent = waitFifoEventForCalibration;
    g_calIo.waitUser = nullptr;
    g_calIo.latestTempC = g_latestTempC;
}

// ============================================================
// Command hooks
// ============================================================

static void hookResetFifoRuntime(void* user) {
    (void)user;
    resetFifoRuntimeCounters();
    g_lastSampleTimestampUs = 0;
    enterTrackingRecovery(imu_quality_flags::FIFO_RECOVERY_REQUESTED, "manual_fifo_reset", lsmFifo.stats().lastAssignedTimestampUs);
}

static void hookResetAhrsRuntime(void* user) {
    (void)user;
    g_lastSampleTimestampUs = 0;
    resetOrientationDependentState("ahrs_or_config_reset", lsmFifo.stats().lastAssignedTimestampUs, false);
}

static void printRuntimeStatus(Stream& out, void* user) {
    (void)user;
    out.print("uptime_ms="); out.println(millis());
    out.print("config_loaded_from_nvs="); out.println(g_configLoadedFromNvs ? "yes" : "no");
    out.print("runtime_samples="); out.println(g_runtimeSamples);
    out.print("fifo_int_count="); out.println(g_fifoIntCount);
    out.print("fifo_int_missed="); out.println(g_fifoIntMissed);
    out.print("fifo_status_fallback_events="); out.println(g_fifoStatusFallbackEvents);
    out.print("fifo_wait_timeouts="); out.println(g_fifoWaitTimeouts);
    out.print("latest_temp_c="); out.println(g_latestTempC, 3);
    out.print("mag_enabled="); out.println(g_config.data.magCal.driverEnabled ? "yes" : "no");
    out.print("mag_runtime_samples="); out.println(g_magState.samples);
    out.print("mag_fifo_armed="); out.println(g_magState.fifoArmed ? "yes" : "no");
    out.print("gyro_bias_valid="); out.println(g_imuCal.gyroBiasValid ? "yes" : "no");
    out.print("accel_cal_valid="); out.println(g_imuCal.accelCalValid ? "yes" : "no");
    out.print("tracking_state="); out.println(trackingStateName());
    out.print("tracking_recovery_active="); out.println(g_trackingRecovering ? "yes" : "no");
    out.print("tracking_recovery_enter_count="); out.println(g_trackingRecoveryEnterCount);
    out.print("tracking_recovery_last_flags=0x"); out.println(g_trackingRecoveryLastFlags, HEX);
    out.print("last_output_confidence="); out.println(g_lastOutputConfidence, 6);
    out.print("quality_recovery_requested="); out.println(g_quality.recoveryRequested() ? "yes" : "no");

    const Ahrs6DofConfig& acfg = g_ahrs6dof.config();
    const Ahrs6DofStats& ast = g_ahrs6dof.stats();
    out.print("ahrs_accel_enabled="); out.println(acfg.accelCorrectionEnabled ? "yes" : "no");
    out.print("ahrs_adaptive_accel="); out.println(acfg.adaptiveAccelCorrection ? "yes" : "no");
    out.print("ahrs_accel_kp="); out.println(acfg.accelKp, 6);
    out.print("ahrs_accel_trust="); out.println(ast.lastAccelGate.trust, 6);
    out.print("ahrs_accel_variance_trust="); out.println(ast.lastAccelNormVarianceTrust, 6);
    out.print("ahrs_gyro_motion_trust="); out.println(ast.lastGyroMotionTrust, 6);
    out.print("ahrs_accel_norm_variance_g2="); out.println(ast.accelNormVarianceG2, 9);
    out.print("stream_mode="); out.println(g_streamState.mode == TrackerStreamMode::Off ? "off" :
                                        g_streamState.mode == TrackerStreamMode::Raw ? "raw" :
                                        g_streamState.mode == TrackerStreamMode::Scaled ? "scaled" :
                                        g_streamState.mode == TrackerStreamMode::Quat ? "quat" :
                                        g_streamState.mode == TrackerStreamMode::Heartbeat ? "heartbeat" : "debug");
    out.print("stream_rate_hz="); out.println(g_streamState.rateHz);

    const Vec3 e = g_ahrs6dof.eulerDeg();
    const Quat q = g_ahrs6dof.quaternionPositiveW();
    out.print("quat=");
    out.print(q.w, 7); out.print(',');
    out.print(q.x, 7); out.print(',');
    out.print(q.y, 7); out.print(',');
    out.println(q.z, 7);
    out.print("euler_deg=");
    out.print(e.x, 3); out.print(',');
    out.print(e.y, 3); out.print(',');
    out.println(e.z, 3);
}

static void printRuntimeHealth(Stream& out, void* user) {
    (void)user;
    printRuntimeStatus(out, nullptr);
    out.println("# FIFO SUMMARY");
    const auto& fs = lsmFifo.stats();
    out.print("fifo_words_read="); out.println(fs.fifoWordsRead);
    out.print("imu_samples_produced="); out.println(fs.imuSamplesProduced);
    out.print("gyro_words="); out.println(fs.gyroWords);
    out.print("accel_words="); out.println(fs.accelWords);
    out.print("timestamp_words="); out.println(fs.timestampWords);
    out.print("temperature_words="); out.println(fs.tempWords);
    out.print("sensorhub0_words="); out.println(fs.sensorHubSlave0Words);
    out.print("sensorhub_nack_words="); out.println(fs.sensorHubNackWords);
    out.print("mag_samples_produced="); out.println(fs.magSamplesProduced);
    out.print("mag_queue_overflow="); out.println(fs.magQueueOverflow);

    const auto& ms = g_magProcessor.stats();
    out.print("mag_processed_samples="); out.println(ms.processedSamples);
    out.print("mag_trusted_samples="); out.println(ms.trustedSamples);
    out.print("mag_rejected_samples="); out.println(ms.rejectedSamples);
    out.print("mag_last_reject_flags=0x"); out.println(g_lastMagProcessed.rejectFlags, HEX);
    out.print("mag_last_body_norm="); out.println(g_lastMagProcessed.bodyNorm, 6);
    out.print("mag_last_trusted="); out.println(g_lastMagProcessed.trusted ? "yes" : "no");

    const auto& hs = g_magHeading.stats();
    out.print("mag_heading_valid="); out.println(g_lastMagHeading.valid ? "yes" : "no");
    out.print("mag_heading_reject_flags=0x"); out.println(g_lastMagHeading.rejectFlags, HEX);
    out.print("mag_heading_valid_count="); out.println(hs.valid);
    out.print("mag_heading_north_minus_ahrs_yaw_deg="); out.println(g_lastMagHeading.yawInnovationDeg, 6);
    out.print("mag_heading_ref_valid="); out.println(g_magHeadingRef.valid ? "yes" : "no");
    out.print("mag_heading_auto_ref_done="); out.println(g_magHeadingAutoRef.done ? "yes" : "no");
    out.print("mag_heading_auto_ref_reject_flags=0x"); out.println(g_magHeadingAutoRef.lastRejectFlags, HEX);
    out.print("mag_heading_error_to_ref_deg=");
    out.println(g_magHeadingRef.valid && g_lastMagHeading.valid ? magHeadingErrorToReferenceDeg(g_lastMagHeading) : 0.0f, 6);
    out.print("mag_yaw_gate_open=");
    out.println(g_lastMagYawCorrection.gateOpen ? "yes" : "no");
    out.print("mag_yaw_reject_flags=0x");
    out.println(g_lastMagYawCorrection.rejectFlags, HEX);
    out.print("mag_yaw_cooldown_active=");
    out.println(g_lastMagYawCorrection.cooldownActive ? "yes" : "no");

    out.print("mag_yaw_cooldown_remaining_ms=");
    out.println(g_lastMagYawCorrection.cooldownRemainingMs);
    out.print("mag_yaw_error_deg=");
    out.println(g_lastMagYawCorrection.errorDeg, 6);
    out.print("mag_yaw_correction_rate_deg_s=");
    out.println(g_lastMagYawCorrection.correctionRateDegS, 6);
    out.print("mag_yaw_correction_step_deg=");
    out.println(g_lastMagYawCorrection.correctionStepDeg, 6);

    out.print("mag_yaw_apply_enabled=");
    out.println(g_config.data.magYaw.applyEnabled ? "yes" : "no");
    out.print("mag_yaw_applied_last=");
    out.println(g_lastMagYawCorrection.applied ? "yes" : "no");

    out.print("unknown_words="); out.println(fs.unknownWords);
    out.print("overrun_events="); out.println(fs.overrunEvents);
    out.print("full_events="); out.println(fs.fullEvents);
    out.print("tag_counter_jumps="); out.println(fs.tagCounterJumps);
    out.print("hw_ts_assigned="); out.println(fs.hwTimestampAssigned);
    out.print("fb_ts_assigned="); out.println(fs.fallbackTimestampAssigned);

    out.println("# QUALITY SUMMARY");
    const auto& qc = g_quality.counters();
    out.print("quality_samples="); out.println(qc.samples);
    out.print("estimated_dropped_samples="); out.println(qc.estimatedDroppedSamples);
    out.print("large_gap_samples="); out.println(qc.largeGapSamples);
    out.print("gyro_saturated_samples="); out.println(qc.gyroSaturatedSamples);
    out.print("accel_saturated_samples="); out.println(qc.accelSaturatedSamples);
    out.print("fifo_recovery_requests="); out.println(qc.fifoRecoveryRequests);
    out.print("mean_dt_us="); out.println(qc.meanDtUs(), 6);
}

static bool startStaticTestHook(uint32_t durationMs, void* user) {
    (void)user;
    if (g_staticTest.active) return false;

    g_staticTest.reset();
    g_staticTest.active = true;
    g_staticTest.durationMs = durationMs;
    g_staticTest.startMs = millis();
    g_staticTest.lastProgressMs = g_staticTest.startMs;

    const auto& fs = lsmFifo.stats();
    g_staticTest.fifoOverrunAtStart = fs.overrunEvents;
    g_staticTest.fifoFullAtStart = fs.fullEvents;
    g_staticTest.fifoUnknownAtStart = fs.unknownWords;
    g_staticTest.fifoHwTsAtStart = fs.hwTimestampAssigned;
    g_staticTest.fifoFbTsAtStart = fs.fallbackTimestampAssigned;

    const auto& ms = g_magProcessor.stats();
    const auto& hs = g_magHeading.stats();
    const auto& ys = g_magYawCorrection.stats();

    g_staticTest.magEnabledAtStart = g_config.data.magCal.driverEnabled;
    g_staticTest.magYawApplyEnabledAtStart = g_config.data.magYaw.applyEnabled;
    g_staticTest.magRefValidAtStart = g_magHeadingRef.valid;

    g_staticTest.magErrorStartDeg =
        (g_magHeadingRef.valid && g_lastMagHeading.valid)
            ? magHeadingErrorToReferenceDeg(g_lastMagHeading)
            : 0.0f;
    g_staticTest.magErrorEndDeg = g_staticTest.magErrorStartDeg;

    g_staticTest.magTrustedAtStart = ms.trustedSamples;
    g_staticTest.magRejectedAtStart = ms.rejectedSamples;

    g_staticTest.magHeadingValidAtStart = hs.valid;
    g_staticTest.magHeadingRejectedAtStart = hs.rejected;

    g_staticTest.magYawUpdatesAtStart = ys.updates;
    g_staticTest.magYawGateOpenAtStart = ys.gateOpenCount;
    g_staticTest.magYawGateClosedAtStart = ys.gateClosedCount;
    g_staticTest.magYawApplyAllowedAtStart = ys.applyAllowedCount;
    g_staticTest.magYawAppliedAtStart = ys.appliedCount;

    g_staticTest.magYawRejectNoReferenceAtStart = ys.rejectNoReference;
    g_staticTest.magYawRejectHeadingInvalidAtStart = ys.rejectHeadingInvalid;
    g_staticTest.magYawRejectMagNotTrustedAtStart = ys.rejectMagNotTrusted;
    g_staticTest.magYawRejectMagStaleAtStart = ys.rejectMagStale;
    g_staticTest.magYawRejectHorizontalBadAtStart = ys.rejectHorizontalBad;
    g_staticTest.magYawRejectInnovationTooLargeAtStart = ys.rejectInnovationTooLarge;
    g_staticTest.magYawRejectGyroMovingAtStart = ys.rejectGyroMoving;
    g_staticTest.magYawRejectAccelNotTrustedAtStart = ys.rejectAccelNotTrusted;

    Serial.println("# STATIC TEST STARTED");
    Serial.print("# duration_s="); Serial.println(durationMs / 1000UL);
    Serial.println("# stop with: test stop");
    return true;
}

static bool stopStaticTestHook(void* user) {
    (void)user;
    if (!g_staticTest.active) return false;
    g_staticTest.stopRequested = true;
    return true;
}

static void printStaticTestStatus(Stream& out, void* user) {
    (void)user;
    out.print("test_active="); out.println(g_staticTest.active ? "yes" : "no");
    out.print("last_completed_valid="); out.println(g_lastCompletedStaticTestValid ? "yes" : "no");
    if (!g_staticTest.active) {
        if (g_lastCompletedStaticTestValid) {
            out.print("last_completed_age_s="); out.println((millis() - g_lastCompletedStaticTestFinishedMs) / 1000UL);
            out.print("last_completed_samples="); out.println(g_lastCompletedStaticTest.samples);
            out.print("last_completed_temp_mean_c="); out.println(g_lastCompletedStaticTest.tempC.mean(), 3);
            out.print("last_completed_gyro_mean_dps_norm=");
            out.println((g_lastCompletedStaticTest.gyroAfterRadS.mean() * MATH_RAD_TO_DEG).norm(), 6);
        }
        return;
    }
    const uint32_t elapsed = millis() - g_staticTest.startMs;
    out.print("elapsed_s="); out.println(elapsed / 1000UL);
    out.print("duration_s="); out.println(g_staticTest.durationMs / 1000UL);
    out.print("samples="); out.println(g_staticTest.samples);
    out.print("hw_ts="); out.println(g_staticTest.hwTs);
    out.print("fallback_ts="); out.println(g_staticTest.fallbackTs);
    out.print("bad_ts="); out.println(g_staticTest.badTs);
    out.print("estimated_dropped="); out.println(g_staticTest.droppedEstimate);
    out.print("accel_norm_mean="); out.println(g_staticTest.accelNormG.mean(), 6);
    out.print("gyro_after_mean_dps_norm="); out.println((g_staticTest.gyroAfterRadS.mean() * MATH_RAD_TO_DEG).norm(), 6);
}

static bool fitGyroTempFromLastStaticHook(bool persist, Stream& out, void* user);
static void setupCommandInterface() {
    g_cmdCtx.io = &Serial;
    g_cmdCtx.config = &g_config;
    g_cmdCtx.configStore = &g_configStore;
    g_cmdCtx.lsm = &lsm;
    g_cmdCtx.fifo = &lsmFifo;
    g_cmdCtx.sensorHub = &lsmHub;
    g_cmdCtx.mag = &qmc;
    g_cmdCtx.imuCal = &g_imuCal;
    g_cmdCtx.gyroTempComp = &g_gyroTempComp;
    g_cmdCtx.quality = &g_quality;
    g_cmdCtx.ahrs = &g_ahrs6dof;
    g_cmdCtx.calibrationIo = &g_calIo;
    g_cmdCtx.accelCalRunner = &g_accelCalRunner;
    g_cmdCtx.streamState = &g_streamState;

    g_cmdCtx.resetFifoRuntime = hookResetFifoRuntime;
    g_cmdCtx.resetFifoRuntimeUser = nullptr;
    g_cmdCtx.resetAhrsRuntime = hookResetAhrsRuntime;
    g_cmdCtx.resetAhrsRuntimeUser = nullptr;
    g_cmdCtx.printRuntimeStatus = printRuntimeStatus;
    g_cmdCtx.printRuntimeStatusUser = nullptr;
    g_cmdCtx.printRuntimeHealth = printRuntimeHealth;
    g_cmdCtx.printRuntimeHealthUser = nullptr;
    g_cmdCtx.startStaticTest = startStaticTestHook;
    g_cmdCtx.startStaticTestUser = nullptr;
    g_cmdCtx.stopStaticTest = stopStaticTestHook;
    g_cmdCtx.stopStaticTestUser = nullptr;
    g_cmdCtx.printStaticTestStatus = printStaticTestStatus;
    g_cmdCtx.printStaticTestStatusUser = nullptr;
    g_cmdCtx.setMagRuntimeEnabled = setMagRuntimeEnabledHook;
    g_cmdCtx.setMagRuntimeEnabledUser = nullptr;
    g_cmdCtx.printMagRuntimeStatus = printMagRuntimeStatus;
    g_cmdCtx.printMagRuntimeStatusUser = nullptr;
    g_cmdCtx.printMagProcessedStatus = printMagProcessedStatus;
    g_cmdCtx.printMagProcessedStatusUser = nullptr;
    g_cmdCtx.printMagHeadingStatus = printMagHeadingStatus;
    g_cmdCtx.printMagHeadingStatusUser = nullptr;
    g_cmdCtx.setMagHeadingReference = setMagHeadingReferenceHook;
    g_cmdCtx.setMagHeadingReferenceUser = nullptr;
    g_cmdCtx.clearMagHeadingReference = clearMagHeadingReferenceHook;
    g_cmdCtx.clearMagHeadingReferenceUser = nullptr;
    g_cmdCtx.setMagHeadingAutoReferenceEnabled = setMagHeadingAutoReferenceEnabledHook;
    g_cmdCtx.setMagHeadingAutoReferenceEnabledUser = nullptr;
    g_cmdCtx.printMagYawCorrectionStatus = printMagYawCorrectionStatus;
    g_cmdCtx.printMagYawCorrectionStatusUser = nullptr;
    g_cmdCtx.resetMagYawCorrection = resetMagYawCorrectionHook;
    g_cmdCtx.resetMagYawCorrectionUser = nullptr;
    g_cmdCtx.setMagYawCorrectionApplyEnabled = setMagYawCorrectionApplyEnabledHook;
    g_cmdCtx.setMagYawCorrectionApplyEnabledUser = nullptr;
    g_cmdCtx.startMagCalibration = startMagCalibrationHook;
    g_cmdCtx.startMagCalibrationUser = nullptr;
    g_cmdCtx.stopMagCalibration = stopMagCalibrationHook;
    g_cmdCtx.stopMagCalibrationUser = nullptr;
    g_cmdCtx.resetMagCalibration = resetMagCalibrationHook;
    g_cmdCtx.resetMagCalibrationUser = nullptr;
    g_cmdCtx.applyMagCalibration = applyMagCalibrationHook;
    g_cmdCtx.applyMagCalibrationUser = nullptr;
    g_cmdCtx.printMagCalibrationStatus = printMagCalibrationStatus;
    g_cmdCtx.printMagCalibrationStatusUser = nullptr;
    g_cmdCtx.fitGyroTempFromLastStatic = fitGyroTempFromLastStaticHook;
    g_cmdCtx.fitGyroTempFromLastStaticUser = nullptr;

    g_cli.begin(g_cmdCtx);
}

// ============================================================
// Runtime processing
// ============================================================

static void maybeRecoverFifo(const ImuQualityResult& quality, const Lsm6dsv::RawSample& raw) {
    if (!quality.shouldRequestFifoRecovery) return;

    Serial.print("# WARN FIFO recovery requested quality_flags=0x");
    Serial.println(quality.flags, HEX);

    const uint64_t ts = raw.t_us != 0 ? raw.t_us : lsmFifo.stats().lastAssignedTimestampUs;
    enterTrackingRecovery(quality.flags, "fifo_recovery", ts);
    lsmFifo.resetFifo();
    lsmFifo.resetTimestampReconstruction(ts);
    g_quality.clearRecoveryRequest();
    resetFifoRuntimeCounters();
}

static void finishStaticTest() {
    if (!g_staticTest.active) return;

    const auto& fs = lsmFifo.stats();
    const uint32_t elapsedMs = millis() - g_staticTest.startMs;
    const float durationS = static_cast<float>(elapsedMs) / 1000.0f;
    const float sampleRate = durationS > 0.0f ? static_cast<float>(g_staticTest.samples) / durationS : 0.0f;

    const Vec3 gyroAfterMeanDps = g_staticTest.gyroAfterRadS.mean() * MATH_RAD_TO_DEG;
    const Vec3 gyroAfterStdDps = g_staticTest.gyroAfterRadS.stddev() * MATH_RAD_TO_DEG;
    const Vec3 gyroAfterMinDps = g_staticTest.gyroAfterRadS.minValue * MATH_RAD_TO_DEG;
    const Vec3 gyroAfterMaxDps = g_staticTest.gyroAfterRadS.maxValue * MATH_RAD_TO_DEG;

    const float dRoll = angleDiffDeg(g_staticTest.eulerStartDeg.x, g_staticTest.eulerEndDeg.x);
    const float dPitch = angleDiffDeg(g_staticTest.eulerStartDeg.y, g_staticTest.eulerEndDeg.y);
    const float dYaw = angleDiffDeg(g_staticTest.eulerStartDeg.z, g_staticTest.eulerEndDeg.z);

    const float durationMin = durationS > 0.0f ? durationS / 60.0f : 0.0f;
    const float yawDriftDegPerMin = durationMin > 0.0f ? dYaw / durationMin : 0.0f;

    Serial.println();
    Serial.println("==============================================================================");
    Serial.println("COMMAND STATIC TEST REPORT");
    Serial.println("==============================================================================");
    Serial.print("stopped_by_command: "); Serial.println(g_staticTest.stopRequested ? "yes" : "no");
    Serial.print("duration_s: "); Serial.println(durationS, 3);
    Serial.print("samples: "); Serial.println(g_staticTest.samples);
    Serial.print("sample_rate_hz: "); Serial.println(sampleRate, 3);
    Serial.print("hw_timestamp_samples: "); Serial.println(g_staticTest.hwTs);
    Serial.print("fallback_timestamp_samples: "); Serial.println(g_staticTest.fallbackTs);
    Serial.print("bad_timestamp_samples: "); Serial.println(g_staticTest.badTs);
    Serial.print("estimated_dropped_samples: "); Serial.println(g_staticTest.droppedEstimate);
    Serial.print("recovery_requests: "); Serial.println(g_staticTest.recoveryRequests);
    Serial.print("ahrs_skipped_samples: "); Serial.println(g_staticTest.ahrsSkipped);
    Serial.print("accel_correction_disabled_samples: "); Serial.println(g_staticTest.accelDisabled);

    Serial.println("------------------------------------------------------------------------------");
    Serial.println("FIFO delta during test");
    Serial.print("fifo_overrun_delta: "); Serial.println(fs.overrunEvents - g_staticTest.fifoOverrunAtStart);
    Serial.print("fifo_full_delta: "); Serial.println(fs.fullEvents - g_staticTest.fifoFullAtStart);
    Serial.print("fifo_unknown_delta: "); Serial.println(fs.unknownWords - g_staticTest.fifoUnknownAtStart);
    Serial.print("fifo_hw_ts_delta: "); Serial.println(fs.hwTimestampAssigned - g_staticTest.fifoHwTsAtStart);
    Serial.print("fifo_fb_ts_delta: "); Serial.println(fs.fallbackTimestampAssigned - g_staticTest.fifoFbTsAtStart);

    Serial.println("------------------------------------------------------------------------------");
    Serial.println("Timing / motion stats");
    Serial.print("dt_mean_us: "); Serial.println(g_staticTest.dtUs.mean(), 6);
    Serial.print("dt_min_us: "); Serial.println(g_staticTest.dtUs.minValue, 6);
    Serial.print("dt_max_us: "); Serial.println(g_staticTest.dtUs.maxValue, 6);
    Serial.print("dt_std_us: "); Serial.println(g_staticTest.dtUs.stddev(), 6);
    Serial.print("accel_norm_mean_g: "); Serial.println(g_staticTest.accelNormG.mean(), 6);
    Serial.print("accel_norm_min_g: "); Serial.println(g_staticTest.accelNormG.minValue, 6);
    Serial.print("accel_norm_max_g: "); Serial.println(g_staticTest.accelNormG.maxValue, 6);
    Serial.print("accel_norm_std_g: "); Serial.println(g_staticTest.accelNormG.stddev(), 8);
    Serial.print("gyro_after_mean_dps_norm: ");
    Serial.println(gyroAfterMeanDps.norm(), 6);

    Serial.print("gyro_after_mean_dps_xyz: ");
    Serial.print(gyroAfterMeanDps.x, 8); Serial.print(',');
    Serial.print(gyroAfterMeanDps.y, 8); Serial.print(',');
    Serial.println(gyroAfterMeanDps.z, 8);

    Serial.print("gyro_after_std_dps_xyz: ");
    Serial.print(gyroAfterStdDps.x, 8); Serial.print(',');
    Serial.print(gyroAfterStdDps.y, 8); Serial.print(',');
    Serial.println(gyroAfterStdDps.z, 8);

    Serial.print("gyro_after_min_dps_xyz: ");
    Serial.print(gyroAfterMinDps.x, 8); Serial.print(',');
    Serial.print(gyroAfterMinDps.y, 8); Serial.print(',');
    Serial.println(gyroAfterMinDps.z, 8);

    Serial.print("gyro_after_max_dps_xyz: ");
    Serial.print(gyroAfterMaxDps.x, 8); Serial.print(',');
    Serial.print(gyroAfterMaxDps.y, 8); Serial.print(',');
    Serial.println(gyroAfterMaxDps.z, 8);

    Serial.print("temp_start_c: ");
    Serial.println(g_staticTest.tempStartC, 3);

    Serial.print("temp_end_c: ");
    Serial.println(g_staticTest.tempEndC, 3);

    Serial.print("temp_delta_c: ");
    Serial.println(g_staticTest.tempEndC - g_staticTest.tempStartC, 3);

    Serial.print("temp_mean_c: ");
    Serial.println(g_staticTest.tempC.mean(), 3);

    Serial.print("temp_min_c: ");
    Serial.println(g_staticTest.tempC.minValue, 3);

    Serial.print("temp_max_c: ");
    Serial.println(g_staticTest.tempC.maxValue, 3);

    Serial.println("------------------------------------------------------------------------------");
    Serial.println("Orientation");
    Serial.print("euler_start_deg: ");
    Serial.print(g_staticTest.eulerStartDeg.x, 3); Serial.print(',');
    Serial.print(g_staticTest.eulerStartDeg.y, 3); Serial.print(',');
    Serial.println(g_staticTest.eulerStartDeg.z, 3);
    Serial.print("euler_end_deg: ");
    Serial.print(g_staticTest.eulerEndDeg.x, 3); Serial.print(',');
    Serial.print(g_staticTest.eulerEndDeg.y, 3); Serial.print(',');
    Serial.println(g_staticTest.eulerEndDeg.z, 3);
    Serial.print("euler_delta_deg: roll="); Serial.print(dRoll, 3);
    Serial.print(" pitch="); Serial.print(dPitch, 3);
    Serial.print(" yaw="); Serial.println(dYaw, 3);
    Serial.print("yaw_drift_rate_deg_min: ");
    Serial.println(yawDriftDegPerMin, 6);

    Serial.println("------------------------------------------------------------------------------");
    Serial.println("MAG / YAW CORRECTION");

    const auto& ms = g_magProcessor.stats();
    const auto& hs = g_magHeading.stats();
    const auto& ys = g_magYawCorrection.stats();

    const float magErrorAbsMeanDeg =
        g_staticTest.magErrorSamples > 0
            ? static_cast<float>(g_staticTest.magErrorAbsSumDeg / static_cast<double>(g_staticTest.magErrorSamples))
            : 0.0f;

    Serial.print("mag_enabled_start: ");
    Serial.println(g_staticTest.magEnabledAtStart ? "yes" : "no");

    Serial.print("mag_yaw_apply_enabled_start: ");
    Serial.println(g_staticTest.magYawApplyEnabledAtStart ? "yes" : "no");

    Serial.print("mag_ref_valid_start: ");
    Serial.println(g_staticTest.magRefValidAtStart ? "yes" : "no");

    Serial.print("mag_ref_valid_end: ");
    Serial.println(g_magHeadingRef.valid ? "yes" : "no");

    Serial.print("mag_error_start_deg: ");
    Serial.println(g_staticTest.magErrorStartDeg, 6);

    Serial.print("mag_error_end_deg: ");
    Serial.println(g_staticTest.magErrorEndDeg, 6);

    Serial.print("mag_error_delta_deg: ");
    Serial.println(g_staticTest.magErrorEndDeg - g_staticTest.magErrorStartDeg, 6);

    Serial.print("mag_error_abs_mean_deg: ");
    Serial.println(magErrorAbsMeanDeg, 6);

    Serial.print("mag_error_abs_max_deg: ");
    Serial.println(g_staticTest.magErrorAbsMaxDeg, 6);

    Serial.print("mag_error_samples: ");
    Serial.println(g_staticTest.magErrorSamples);

    Serial.print("mag_heading_horizontal_norm_mean: ");
    Serial.println(g_staticTest.magHeadingHorizontalNorm.mean(), 6);

    Serial.print("mag_heading_horizontal_norm_min: ");
    Serial.println(g_staticTest.magHeadingHorizontalNorm.minValue, 6);

    Serial.print("mag_heading_horizontal_norm_max: ");
    Serial.println(g_staticTest.magHeadingHorizontalNorm.maxValue, 6);

    Serial.print("mag_yaw_combined_trust_mean: ");
    Serial.println(g_staticTest.magYawCombinedTrust.mean(), 6);

    Serial.print("mag_yaw_combined_trust_min: ");
    Serial.println(g_staticTest.magYawCombinedTrust.minValue, 6);

    Serial.print("mag_yaw_combined_trust_max: ");
    Serial.println(g_staticTest.magYawCombinedTrust.maxValue, 6);

    Serial.print("mag_yaw_correction_rate_mean_deg_s: ");
    Serial.println(g_staticTest.magYawCorrectionRateDegS.mean(), 8);

    Serial.print("mag_yaw_correction_rate_min_deg_s: ");
    Serial.println(g_staticTest.magYawCorrectionRateDegS.minValue, 8);

    Serial.print("mag_yaw_correction_rate_max_deg_s: ");
    Serial.println(g_staticTest.magYawCorrectionRateDegS.maxValue, 8);

    Serial.print("mag_yaw_correction_step_mean_deg: ");
    Serial.println(g_staticTest.magYawCorrectionStepDeg.mean(), 9);

    Serial.print("mag_yaw_correction_step_min_deg: ");
    Serial.println(g_staticTest.magYawCorrectionStepDeg.minValue, 9);

    Serial.print("mag_yaw_correction_step_max_deg: ");
    Serial.println(g_staticTest.magYawCorrectionStepDeg.maxValue, 9);

    Serial.print("mag_trusted_delta: ");
    Serial.println(ms.trustedSamples - g_staticTest.magTrustedAtStart);

    Serial.print("mag_rejected_delta: ");
    Serial.println(ms.rejectedSamples - g_staticTest.magRejectedAtStart);

    Serial.print("mag_heading_valid_delta: ");
    Serial.println(hs.valid - g_staticTest.magHeadingValidAtStart);

    Serial.print("mag_heading_rejected_delta: ");
    Serial.println(hs.rejected - g_staticTest.magHeadingRejectedAtStart);

    Serial.print("mag_yaw_updates_delta: ");
    Serial.println(ys.updates - g_staticTest.magYawUpdatesAtStart);

    Serial.print("mag_yaw_gate_open_delta: ");
    Serial.println(ys.gateOpenCount - g_staticTest.magYawGateOpenAtStart);

    Serial.print("mag_yaw_gate_closed_delta: ");
    Serial.println(ys.gateClosedCount - g_staticTest.magYawGateClosedAtStart);

    Serial.print("mag_yaw_apply_allowed_delta: ");
    Serial.println(ys.applyAllowedCount - g_staticTest.magYawApplyAllowedAtStart);

    Serial.print("mag_yaw_applied_delta: ");
    Serial.println(ys.appliedCount - g_staticTest.magYawAppliedAtStart);

    Serial.print("mag_yaw_last_error_deg: ");
    Serial.println(g_lastMagYawCorrection.errorDeg, 6);

    Serial.print("mag_yaw_last_correction_rate_deg_s: ");
    Serial.println(g_lastMagYawCorrection.correctionRateDegS, 6);

    Serial.print("mag_yaw_last_correction_step_deg: ");
    Serial.println(g_lastMagYawCorrection.correctionStepDeg, 6);

    Serial.print("mag_yaw_reject_no_reference_delta: ");
    Serial.println(ys.rejectNoReference - g_staticTest.magYawRejectNoReferenceAtStart);

    Serial.print("mag_yaw_reject_heading_invalid_delta: ");
    Serial.println(ys.rejectHeadingInvalid - g_staticTest.magYawRejectHeadingInvalidAtStart);

    Serial.print("mag_yaw_reject_mag_not_trusted_delta: ");
    Serial.println(ys.rejectMagNotTrusted - g_staticTest.magYawRejectMagNotTrustedAtStart);

    Serial.print("mag_yaw_reject_mag_stale_delta: ");
    Serial.println(ys.rejectMagStale - g_staticTest.magYawRejectMagStaleAtStart);

    Serial.print("mag_yaw_reject_horizontal_bad_delta: ");
    Serial.println(ys.rejectHorizontalBad - g_staticTest.magYawRejectHorizontalBadAtStart);

    Serial.print("mag_yaw_reject_innovation_too_large_delta: ");
    Serial.println(ys.rejectInnovationTooLarge - g_staticTest.magYawRejectInnovationTooLargeAtStart);

    Serial.print("mag_yaw_reject_gyro_moving_delta: ");
    Serial.println(ys.rejectGyroMoving - g_staticTest.magYawRejectGyroMovingAtStart);

    Serial.print("mag_yaw_reject_accel_not_trusted_delta: ");
    Serial.println(ys.rejectAccelNotTrusted - g_staticTest.magYawRejectAccelNotTrustedAtStart);
    Serial.println("==============================================================================");
    Serial.println("STATIC TEST DONE");
    Serial.println("==============================================================================");

    const bool completedOk = g_staticTest.samples > 0 &&
                             g_staticTest.gyroAfterRadS.count > 0 &&
                             g_staticTest.tempC.count > 0;
    if (completedOk) {
        g_lastCompletedStaticTest = g_staticTest;
        g_lastCompletedStaticTest.active = false;
        g_lastCompletedStaticTest.stopRequested = false;
        g_lastCompletedStaticTestValid = true;
        g_lastCompletedStaticTestFinishedMs = millis();
    }

    g_staticTest.reset();
}

static void updateStaticTest(const Lsm6dsv::RawSample& raw,
                             const Lsm6dsv::Sample& calibrated,
                             const ImuQualityResult& quality) {
    if (!g_staticTest.active) return;

    const uint32_t nowMs = millis();
    const uint32_t elapsedMs = nowMs - g_staticTest.startMs;

    if (quality.has(imu_quality_flags::TIMESTAMP_HARDWARE)) g_staticTest.hwTs++;
    if (quality.has(imu_quality_flags::TIMESTAMP_FALLBACK)) g_staticTest.fallbackTs++;
    if (quality.has(imu_quality_flags::TIMESTAMP_ZERO) || quality.has(imu_quality_flags::TIMESTAMP_NON_MONOTONIC)) g_staticTest.badTs++;
    if (quality.has(imu_quality_flags::SAMPLE_DROPPED_BEFORE)) g_staticTest.droppedEstimate += quality.estimatedDroppedBefore;
    if (quality.shouldRequestFifoRecovery) g_staticTest.recoveryRequests++;
    if (!quality.shouldUpdateAhrs) g_staticTest.ahrsSkipped++;
    if (!quality.shouldUseAccelCorrection) g_staticTest.accelDisabled++;

    if (quality.dtUs > 0) g_staticTest.dtUs.push(static_cast<float>(quality.dtUs));
    g_staticTest.accelNormG.push(calibrated.accel_g.norm());
    g_staticTest.accelTrust.push(quality.accelConfidence);
    if (!g_staticTest.tempCaptured) {
        g_staticTest.tempCaptured = true;
        g_staticTest.tempStartC = calibrated.temp_c;
    }

    g_staticTest.tempEndC = calibrated.temp_c;

    g_staticTest.tempC.push(calibrated.temp_c);
    g_staticTest.gyroAfterRadS.push(calibrated.gyro_rad_s);
    g_staticTest.samples++;

    if (!g_staticTest.poseCaptured && g_ahrs6dof.initialized()) {
        g_staticTest.poseCaptured = true;
        g_staticTest.eulerStartDeg = g_ahrs6dof.eulerDeg();
        g_staticTest.qStart = g_ahrs6dof.quaternionPositiveW();
    }
    g_staticTest.eulerEndDeg = g_ahrs6dof.eulerDeg();
    g_staticTest.qEnd = g_ahrs6dof.quaternionPositiveW();

    if (nowMs - g_staticTest.lastProgressMs >= HEARTBEAT_PERIOD_MS) {
        g_staticTest.lastProgressMs = nowMs;
        Serial.print("# test progress elapsed_s="); Serial.print(elapsedMs / 1000UL);
        Serial.print(" samples="); Serial.print(g_staticTest.samples);
        Serial.print(" hw_ts="); Serial.print(g_staticTest.hwTs);
        Serial.print(" fb_ts="); Serial.print(g_staticTest.fallbackTs);
        Serial.print(" dropped_est="); Serial.print(g_staticTest.droppedEstimate);
        Serial.print(" accel_norm_mean="); Serial.print(g_staticTest.accelNormG.mean(), 6);
        Serial.print(" gyro_after_mean_dps_norm="); Serial.println((g_staticTest.gyroAfterRadS.mean() * MATH_RAD_TO_DEG).norm(), 6);
    }

    if (g_staticTest.stopRequested || elapsedMs >= g_staticTest.durationMs) {
        finishStaticTest();
    }
}

static bool fitGyroTempFromLastStaticHook(bool persist, Stream& out, void* user) {
    (void)user;

    if (!g_gyroTempComp.valid()) {
        out.println("# ERR gyro temp comp is not valid; run cal gyro first");
        return false;
    }

    if (!g_lastCompletedStaticTestValid ||
        g_lastCompletedStaticTest.samples < 1000 ||
        g_lastCompletedStaticTest.gyroAfterRadS.count < 1000) {
        out.println("# ERR no usable completed static test data; run test static first and let it finish");
        return false;
    }

    const StaticRuntimeTest& test = g_lastCompletedStaticTest;
    const float tempMeanC = test.tempC.mean();
    const float tempStartC = test.tempStartC;
    const float tempEndC = test.tempEndC;
    const float tempDeltaDuringTestC = tempEndC - tempStartC;

    const float refTempC = g_gyroTempComp.referenceTempC();
    const float dT = tempMeanC - refTempC;

    if (!tracker::isFinite(tempMeanC) || !tracker::isFinite(dT) || std::fabs(dT) < 1.5f) {
        out.print("# ERR temp delta from reference too small: dT=");
        out.println(dT, 3);
        return false;
    }

    const Vec3 residualRadS = test.gyroAfterRadS.mean();
    if (!residualRadS.isFinite()) {
        out.println("# ERR residual gyro mean is non-finite");
        return false;
    }

    const Vec3 oldSlopeRadSPerC = g_gyroTempComp.slopeRadSPerC();
    const Vec3 correctionSlopeRadSPerC = residualRadS / dT;
    const Vec3 newSlopeRadSPerC = oldSlopeRadSPerC + correctionSlopeRadSPerC;

    const Vec3 oldSlopeDpsPerC = oldSlopeRadSPerC * MATH_RAD_TO_DEG;
    const Vec3 correctionSlopeDpsPerC = correctionSlopeRadSPerC * MATH_RAD_TO_DEG;
    const Vec3 newSlopeDpsPerC = newSlopeRadSPerC * MATH_RAD_TO_DEG;
    const Vec3 residualDps = residualRadS * MATH_RAD_TO_DEG;

    const GyroTempCompConfig& cfg = g_gyroTempComp.config();
    if (std::fabs(newSlopeDpsPerC.x) > cfg.maxAbsSlopeDpsPerC ||
        std::fabs(newSlopeDpsPerC.y) > cfg.maxAbsSlopeDpsPerC ||
        std::fabs(newSlopeDpsPerC.z) > cfg.maxAbsSlopeDpsPerC) {
        out.println("# ERR fitted slope exceeds maxAbsSlopeDpsPerC");
        out.print("max_abs_slope_dps_per_c=");
        out.println(cfg.maxAbsSlopeDpsPerC, 6);
        tracker_serial_detail::printVec3Line(out, "candidate_slope_dps_per_c", newSlopeDpsPerC, 8);
        return false;
    }

    g_gyroTempComp.setSlopeRadSPerC(newSlopeRadSPerC);
    g_gyroTempComp.setEnabled(true);

    g_config.captureFromGyroTempComp(g_gyroTempComp);
    g_config.sanitize();
    g_config.updateCrc();

    if (persist) {
        if (!g_configStore.save(g_config)) {
            out.print("# ERR gyro temp fit save failed: ");
            out.println(g_configStore.lastErrorName());
            return false;
        }
    }

    out.println("# GYRO TEMP FIT FROM STATIC");
    out.print("reference_temp_c="); out.println(refTempC, 3);
    out.print("static_temp_mean_c="); out.println(tempMeanC, 3);
    out.print("static_temp_start_c="); out.println(tempStartC, 3);
    out.print("static_temp_end_c="); out.println(tempEndC, 3);
    out.print("static_temp_delta_during_test_c="); out.println(tempDeltaDuringTestC, 3);
    out.print("delta_from_reference_c="); out.println(dT, 3);

    if (std::fabs(tempDeltaDuringTestC) > 2.0f) {
        out.println("# WARN static test temperature changed by >2C; fit is usable but less clean than a stabilized temp point");
    }

    tracker_serial_detail::printVec3Line(out, "residual_mean_dps", residualDps, 8);
    tracker_serial_detail::printVec3Line(out, "old_slope_dps_per_c", oldSlopeDpsPerC, 8);
    tracker_serial_detail::printVec3Line(out, "correction_slope_dps_per_c", correctionSlopeDpsPerC, 8);
    tracker_serial_detail::printVec3Line(out, "new_slope_dps_per_c", newSlopeDpsPerC, 8);

    out.print("# OK gyro temperature compensation fitted");
    if (persist) out.print(" and saved");
    out.println();

    return true;
}

static void emitStreamIfNeeded(const Lsm6dsv::RawSample& raw,
                               const Lsm6dsv::Sample& scaled,
                               const Lsm6dsv::Sample& calibrated,
                               const ImuQualityResult& quality) {
    if (!trackerSerialStreamDue(g_streamState, micros())) return;

    switch (g_streamState.mode) {
        case TrackerStreamMode::Off:
            break;
        case TrackerStreamMode::Raw:
            trackerSerialEmitRaw(Serial, raw, quality.flags);
            break;
        case TrackerStreamMode::Scaled:
            trackerSerialEmitScaled(Serial, raw.t_us, calibrated, quality.flags);
            break;
        case TrackerStreamMode::Quat:
            trackerSerialEmitQuat(Serial, raw.t_us, g_ahrs6dof.quaternionPositiveW(), quality.flags, quality.overallConfidence);
            break;
        case TrackerStreamMode::Heartbeat:
            break;
        case TrackerStreamMode::Debug:
            Serial.print("DBG,t="); Serial.print(static_cast<unsigned long>(raw.t_us));
            Serial.print(",euler=");
            {
                const Vec3 e = g_ahrs6dof.eulerDeg();
                Serial.print(e.x, 2); Serial.print(',');
                Serial.print(e.y, 2); Serial.print(',');
                Serial.print(e.z, 2);
            }
            Serial.print(",temp="); Serial.print(calibrated.temp_c, 2);
            Serial.print(",qflags=0x"); Serial.println(quality.flags, HEX);
            break;
    }
}

static void processOneRawSample(const Lsm6dsv::RawSample& raw) {
    updateLatestTemperatureFromFifo();
    g_calIo.latestTempC = g_latestTempC;

    Lsm6dsv::Sample scaled = lsm.scale(raw);
    scaled.temp_c = g_latestTempC;
    Lsm6dsv::Sample calibrated = makeCalibratedSample(scaled);

    const auto& fifoStats = lsmFifo.stats();
    ImuQualityResult quality = g_quality.evaluate(raw, calibrated, fifoStats);

    const bool largeGap = quality.has(imu_quality_flags::TIMESTAMP_LARGE_GAP);

    if (quality.shouldRequestFifoRecovery) {
        g_runtimeSamples++;
        g_lastSampleTimestampUs = raw.t_us;
        g_lastOutputConfidence = quality.overallConfidence;
        emitStreamIfNeeded(raw, scaled, calibrated, quality);
        updateStaticTest(raw, calibrated, quality);
        maybeRecoverFifo(quality, raw);
        return;
    }

    if (largeGap) {
        enterTrackingRecovery(quality.flags, "large_dt_gap", raw.t_us);
    } else if (quality.shouldUpdateAhrs) {
        const Vec3 accelForAhrs = quality.accelForAhrs(calibrated.accel_g);
        g_ahrs6dof.update(calibrated.gyro_rad_s, accelForAhrs, raw.t_us);
    }

    g_runtimeSamples++;
    g_lastSampleTimestampUs = raw.t_us;
    g_lastOutputConfidence = quality.overallConfidence;

    emitStreamIfNeeded(raw, scaled, calibrated, quality);
    updateStaticTest(raw, calibrated, quality);
    updateTrackingRecoveryState(quality);
}

static void processFifoRuntime() {
    if (!consumeFifoInterruptEvent(0)) return;

    const uint8_t maxRounds = g_config.data.fifo.maxDrainRoundsPerEvent > 0
        ? g_config.data.fifo.maxDrainRoundsPerEvent
        : MAX_DRAIN_ROUNDS_PER_EVENT_DEFAULT;

    for (uint8_t round = 0; round < maxRounds; ++round) {
        size_t count = 0;
        const uint64_t drainTimestampUs = micros();
        const uint16_t maxWords = g_config.data.fifo.maxWordsPerDrain > 0
            ? g_config.data.fifo.maxWordsPerDrain
            : FIFO_MAX_WORDS_PER_DRAIN_DEFAULT;

        const bool ok = lsmFifo.drainRawSamples(
            g_fifoRaw,
            FIFO_RAW_BUFFER_CAPACITY,
            count,
            drainTimestampUs,
            maxWords
        );

        const size_t magCount = lsmFifo.popMagSamples(g_magRaw, MAG_RAW_BUFFER_CAPACITY);
        for (size_t i = 0; i < magCount; ++i) {
            processOneMagRawSample(g_magRaw[i]);
        }

        if (!ok) {
            Serial.println("# ERR FIFO drain failed");
            return;
        }

        if (count == 0 && magCount == 0) break;

        for (size_t i = 0; i < count; ++i) {
            processOneRawSample(g_fifoRaw[i]);
        }
    }
}

static void maybePrintBootHeartbeat() {
    if (g_streamState.mode != TrackerStreamMode::Heartbeat) return;
    if (g_staticTest.active) return;

    const uint32_t nowMs = millis();
    if (nowMs - g_lastHeartbeatMs < 60000UL) return;
    g_lastHeartbeatMs = nowMs;

    Serial.print("# alive uptime_s="); Serial.print(nowMs / 1000UL);
    Serial.print(" samples="); Serial.print(g_runtimeSamples);
    Serial.print(" fifo_int="); Serial.print(g_fifoIntCount);
    Serial.print(" temp_c="); Serial.print(g_latestTempC, 2);
    Serial.print(" mag="); Serial.print(g_magState.samples);
    Serial.print(" stream="); Serial.println("heartbeat");
}

// ============================================================
// Arduino entry points
// ============================================================

void setup() {
    Serial.begin(SERIAL_BAUD_DEFAULT);
    sleep(5);
    delay(300);

    Serial.println();
    Serial.println("==============================================================================");
    Serial.println("ESP32-C3 + LSM6DSV COMMAND TRACKER FIRMWARE");
    Serial.println("==============================================================================");

    loadConfigAndApplyRuntime();

    Serial.print("# config_loaded_from_nvs="); Serial.println(g_configLoadedFromNvs ? "yes" : "no");
    Serial.print("# config_valid="); Serial.println(g_config.validate() ? "yes" : "no");

    pinMode(PIN_LSM_INT1, INPUT);

    if (!initLsm()) {
        Serial.println("# fatal: LSM init failed");
        while (true) delay(1000);
    }

    if (!initFifo()) {
        Serial.println("# fatal: FIFO init failed");
        while (true) delay(1000);
    }

    setupCalibrationIo();
    setupCommandInterface();

    resetFifoRuntimeCounters();
    attachInterrupt(digitalPinToInterrupt(PIN_LSM_INT1), onFifoInt1, RISING);

    lsmFifo.resetFifo();
    lsmFifo.resetTimestampReconstruction(0);
    g_quality.reset();
    g_quality.syncFifoStats(lsmFifo.stats());
    g_ahrs6dof.reset();
    resetOrientationDependentState("startup", 0, false);

    if (g_config.data.magCal.driverEnabled) {
        Serial.println("# mag enabled in config; starting QMC6309 FIFO stream");
        if (!setMagRuntimeEnabledHook(true, false, nullptr)) {
            Serial.println("# WARN mag startup failed; continuing 6DoF without mag");
            g_config.data.magCal.driverEnabled = false;
            g_config.updateCrc();
        }
    }

    Serial.println("# OK INT1 attached: FIFO_WTM/FIFO_OVR/FIFO_FULL, RISING");
    Serial.println("# Type: help");
    Serial.println("==============================================================================");
}

void loop() {
    g_cli.poll();
    processFifoRuntime();
    g_cli.poll();
    maybePrintBootHeartbeat();
}