#include <Arduino.h>
#include <SPI.h>

#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
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

    void reset() {
        count = 0;
        sum = Vec3::zero();
        sumSq = Vec3::zero();
    }

    void push(const Vec3& v) {
        if (!v.isFinite()) return;
        count++;
        sum += v;
        sumSq += hadamard(v, v);
    }

    Vec3 mean() const {
        return count == 0 ? Vec3::zero() : sum / static_cast<float>(count);
    }
};

struct StaticRuntimeTest {
    bool active = false;
    bool stopRequested = false;
    uint32_t durationMs = 0;
    uint32_t startMs = 0;
    uint32_t lastProgressMs = 0;

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

    ScalarStats dtUs;
    ScalarStats accelNormG;
    ScalarStats accelTrust;
    ScalarStats tempC;
    Vec3Stats gyroAfterRadS;

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
        dtUs.reset();
        accelNormG.reset();
        accelTrust.reset();
        tempC.reset();
        gyroAfterRadS.reset();
        poseCaptured = false;
        eulerStartDeg = Vec3::zero();
        eulerEndDeg = Vec3::zero();
        qStart = Quat::identity();
        qEnd = Quat::identity();
    }
};

static StaticRuntimeTest g_staticTest;

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

static void seedDefaultCalibrationIfNvsEmpty() {
    // Optional convenience: your measured accel calibration from previous phase.
    // It is only inserted into RAM defaults when NVS has no config.
    if (g_configLoadedFromNvs) return;

    g_config.data.accelCal.valid = true;
    g_config.data.accelCal.biasG = Vec3(0.00214949f, 0.00605807f, 0.00125885f);
    g_config.data.accelCal.scale = Mat3::diagonal(1.00130630f, 1.00026011f, 1.00308013f);
    g_config.updateCrc();
}

static bool loadConfigAndApplyRuntime() {
    g_configStore.loadOrDefaults(g_config, &g_configLoadedFromNvs);
    g_config.sanitize();
    seedDefaultCalibrationIfNvsEmpty();

    g_config.applyToImuCalibration(g_imuCal);
    g_config.applyToGyroTempComp(g_gyroTempComp);

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
    Serial.print("# internal_freq_fine="); Serial.println(fs.internalFreqFine);
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
}

static void hookResetAhrsRuntime(void* user) {
    (void)user;
    g_lastSampleTimestampUs = 0;
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
    out.print("gyro_bias_valid="); out.println(g_imuCal.gyroBiasValid ? "yes" : "no");
    out.print("accel_cal_valid="); out.println(g_imuCal.accelCalValid ? "yes" : "no");
    out.print("quality_recovery_requested="); out.println(g_quality.recoveryRequested() ? "yes" : "no");
    out.print("stream_mode="); out.println(g_streamState.mode == TrackerStreamMode::Off ? "off" :
                                        g_streamState.mode == TrackerStreamMode::Raw ? "raw" :
                                        g_streamState.mode == TrackerStreamMode::Scaled ? "scaled" :
                                        g_streamState.mode == TrackerStreamMode::Quat ? "quat" : "debug");
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
    if (!g_staticTest.active) return;
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

static void setupCommandInterface() {
    g_cmdCtx.io = &Serial;
    g_cmdCtx.config = &g_config;
    g_cmdCtx.configStore = &g_configStore;
    g_cmdCtx.lsm = &lsm;
    g_cmdCtx.fifo = &lsmFifo;
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
    lsmFifo.resetFifo();
    lsmFifo.resetTimestampReconstruction(ts);
    g_quality.clearRecoveryRequest();
    resetFifoRuntimeCounters();
}

static void finishStaticTest();
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

static void finishStaticTest() {
    if (!g_staticTest.active) return;

    const auto& fs = lsmFifo.stats();
    const uint32_t elapsedMs = millis() - g_staticTest.startMs;
    const float durationS = static_cast<float>(elapsedMs) / 1000.0f;
    const float sampleRate = durationS > 0.0f ? static_cast<float>(g_staticTest.samples) / durationS : 0.0f;

    const Vec3 gyroAfterMeanDps = g_staticTest.gyroAfterRadS.mean() * MATH_RAD_TO_DEG;
    const float dRoll = angleDiffDeg(g_staticTest.eulerStartDeg.x, g_staticTest.eulerEndDeg.x);
    const float dPitch = angleDiffDeg(g_staticTest.eulerStartDeg.y, g_staticTest.eulerEndDeg.y);
    const float dYaw = angleDiffDeg(g_staticTest.eulerStartDeg.z, g_staticTest.eulerEndDeg.z);

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
    Serial.print("gyro_after_mean_dps_norm: "); Serial.println(gyroAfterMeanDps.norm(), 6);
    Serial.print("temp_mean_c: "); Serial.println(g_staticTest.tempC.mean(), 3);
    Serial.print("temp_min_c: "); Serial.println(g_staticTest.tempC.minValue, 3);
    Serial.print("temp_max_c: "); Serial.println(g_staticTest.tempC.maxValue, 3);

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
    Serial.println("==============================================================================");
    Serial.println("STATIC TEST DONE");
    Serial.println("==============================================================================");

    g_staticTest.reset();
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

    if (quality.shouldUpdateAhrs) {
        const Vec3 accelForAhrs = quality.accelForAhrs(calibrated.accel_g);
        g_ahrs6dof.update(calibrated.gyro_rad_s, accelForAhrs, raw.t_us);
    }

    g_runtimeSamples++;
    g_lastSampleTimestampUs = raw.t_us;

    emitStreamIfNeeded(raw, scaled, calibrated, quality);
    updateStaticTest(raw, calibrated, quality);
    maybeRecoverFifo(quality, raw);
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

        if (!ok) {
            Serial.println("# ERR FIFO drain failed");
            return;
        }
        if (count == 0) break;

        for (size_t i = 0; i < count; ++i) {
            processOneRawSample(g_fifoRaw[i]);
        }
    }
}

static void maybePrintBootHeartbeat() {
    if (g_streamState.mode != TrackerStreamMode::Off) return;
    if (g_staticTest.active) return;

    const uint32_t nowMs = millis();
    if (nowMs - g_lastHeartbeatMs < 60000UL) return;
    g_lastHeartbeatMs = nowMs;

    Serial.print("# alive uptime_s="); Serial.print(nowMs / 1000UL);
    Serial.print(" samples="); Serial.print(g_runtimeSamples);
    Serial.print(" fifo_int="); Serial.print(g_fifoIntCount);
    Serial.print(" temp_c="); Serial.print(g_latestTempC, 2);
    Serial.print(" stream="); Serial.println("off");
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
