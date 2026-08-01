#pragma once

#include <Arduino.h>
#include <cstdint>

#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/calibration.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/imu_quality.hpp"
#include "runtime/runtime_bias_types.hpp"
#include "runtime/tracker_runtime_types.hpp"

namespace tracker {

bool runtimeBiasHasBaseGyroBiasModel(const ImuCalibration& imuCal,
                                     const GyroTempCompensator& gyroTempComp);
Vec3 runtimeBiasBaseGyroBiasRadS(const ImuCalibration& imuCal,
                                 const GyroTempCompensator& gyroTempComp,
                                 float tempC);
Vec3 runtimeBiasCurrentGyroBiasRadS(const RuntimeGyroBiasEstimator& bias,
                                    const ImuCalibration& imuCal,
                                    const GyroTempCompensator& gyroTempComp,
                                    float tempC);
Vec3 runtimeBiasCurrentGyroBiasRadS(const RuntimeGyroBiasEstimator& bias,
                                    const ImuCalibration& imuCal,
                                    const GyroTempCompRuntimeEval& tempEval);
uint32_t runtimeBiasGyroBiasRuntimeFlags(const RuntimeGyroBiasEstimator& bias,
                                         const GyroTempCompensator& gyroTempComp,
                                         float tempC);
void runtimeBiasApplyGyroTempQualityFlags(const GyroTempCompensator& gyroTempComp,
                                          ImuQualityResult& quality,
                                          float tempC);
void runtimeBiasApplyGyroTempQualityFlags(const GyroTempCompRuntimeEval& tempEval,
                                          ImuQualityResult& quality);

struct RuntimeBiasTempGate {
    bool rangeRelevant = false;
    bool outOfRange = false;
    bool nearOutOfRange = false;
    bool farOutOfRange = false;
    bool reject = false;
    float distanceToRangeC = 0.0f;
    float gainScale = 1.0f;
};

RuntimeBiasTempGate runtimeBiasTempGateFor(const RuntimeGyroBiasEstimator& bias,
                                           const GyroTempCompensator& gyroTempComp,
                                           float tempC);
RuntimeBiasTempGate runtimeBiasTempGateFor(const RuntimeGyroBiasEstimator& bias,
                                           const GyroTempCompRuntimeEval& tempEval);
uint32_t runtimeBiasDecisionFlags(bool accepted,
                                  bool badTiming,
                                  bool saturated,
                                  bool calibrationBad,
                                  bool tempBad,
                                  bool motionBad,
                                  bool accelBad,
                                  bool finiteBad,
                                  bool priming,
                                  bool dryRun,
                                  bool tempCautious = false);

struct RuntimeGyroBiasUpdateDeps {
    RuntimeGyroBiasEstimator& bias;
    const ImuCalibration& imuCal;
    const GyroTempCompensator& gyroTempComp;
    const Ahrs6Dof& ahrs;
    bool trackingRecovering = false;
    bool logEnabled = false;
    uint32_t* logSequence = nullptr;
    MachineLogCounters* logCounters = nullptr;
    Stream* logStream = nullptr;
};

void emitRuntimeBiasUpdateLog(const RuntimeGyroBiasUpdateDeps& deps,
                              uint64_t tUs,
                              float tempC,
                              const Vec3& residualDps,
                              const Vec3& stdDps,
                              const Vec3& deltaDps,
                              const Vec3& trimDps,
                              uint32_t flags);
Vec3 clampRuntimeTrimDps(const RuntimeGyroBiasEstimator& bias, const Vec3& trimDps);
bool runtimeBiasUpdateEstimator(const RuntimeGyroBiasUpdateDeps& deps,
                                const Lsm6dsv::Sample& scaled,
                                const Lsm6dsv::Sample& calibrated,
                                const ImuQualityResult& quality,
                                uint64_t timestampUs);
bool runtimeBiasUpdateEstimator(const RuntimeGyroBiasUpdateDeps& deps,
                                const Lsm6dsv::Sample& scaled,
                                const Lsm6dsv::Sample& calibrated,
                                const ImuQualityResult& quality,
                                uint64_t timestampUs,
                                const GyroTempCompRuntimeEval& tempEval,
                                const Vec3& currentGyroBiasRadS);
bool runtimeBiasFinalizePendingWindow(const RuntimeGyroBiasUpdateDeps& deps);
void runtimeBiasPrintStatus(Stream& out,
                            const RuntimeGyroBiasEstimator& bias,
                            const ImuCalibration& imuCal,
                            const GyroTempCompensator& gyroTempComp,
                            float latestTempC);
void runtimeBiasSetEnabled(RuntimeGyroBiasEstimator& bias, bool enabled);
void runtimeBiasReset(RuntimeGyroBiasEstimator& bias);

} // namespace tracker
