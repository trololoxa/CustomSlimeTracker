#include "runtime/imu_sample_pipeline.hpp"

#include "build_config/profile_contract.hpp"
#include "sensor/frame_transform.hpp"

namespace tracker {

void imuPipelineUpdateLatestTemperature(ImuSamplePipelineDeps& deps) {
    const auto& fs = deps.fifo.stats();
    if (fs.latestTempValid) {
        deps.latestTempC = fs.latestTempC;
    }
    if (deps.calibrationIo != nullptr) {
        deps.calibrationIo->latestTempC = deps.latestTempC;
    }
}

Vec3 imuPipelineCurrentGyroBiasRadS(const ImuSamplePipelineDeps& deps, float tempC) {
    return runtimeBiasCurrentGyroBiasRadS(deps.runtimeBias, deps.imuCal, deps.gyroTempComp, tempC);
}

Lsm6dsv::Sample imuPipelineMakeSensorFrameCalibratedSample(const ImuSamplePipelineDeps& deps,
                                                           const Lsm6dsv::Sample& scaled) {
    const GyroTempCompRuntimeEval tempEval =
        deps.gyroTempComp.evaluateRuntime(deps.latestTempC);
    return imuPipelineMakeSensorFrameCalibratedSample(
        deps,
        scaled,
        tempEval.valid || deps.imuCal.gyroBiasValid,
        runtimeBiasCurrentGyroBiasRadS(deps.runtimeBias, deps.imuCal, tempEval));
}

Lsm6dsv::Sample imuPipelineMakeSensorFrameCalibratedSample(const ImuSamplePipelineDeps& deps,
                                                           const Lsm6dsv::Sample& scaled,
                                                           bool hasBaseGyroBiasModel,
                                                           const Vec3& currentGyroBiasRadS) {
    Lsm6dsv::Sample calibrated = scaled;
    calibrated.temp_c = deps.latestTempC;

    if (hasBaseGyroBiasModel) {
        // Persistent and runtime gyro-bias state is stored in the native
        // sensor frame. Subtract it before rotating into the device frame.
        calibrated.gyro_rad_s = scaled.gyro_rad_s - currentGyroBiasRadS;
    }

    if (deps.imuCal.accelCalValid) {
        // The six-side correction matrix is also defined in sensor frame.
        calibrated.accel_g = deps.imuCal.applyAccel(scaled.accel_g);
    }
    return calibrated;
}

Lsm6dsv::Sample imuPipelineMakeCalibratedSample(const ImuSamplePipelineDeps& deps,
                                                const Lsm6dsv::Sample& scaled) {
    const GyroTempCompRuntimeEval tempEval =
        deps.gyroTempComp.evaluateRuntime(deps.latestTempC);
    return imuPipelineMakeCalibratedSample(
        deps,
        scaled,
        tempEval.valid || deps.imuCal.gyroBiasValid,
        runtimeBiasCurrentGyroBiasRadS(deps.runtimeBias, deps.imuCal, tempEval));
}

Lsm6dsv::Sample imuPipelineMakeCalibratedSample(const ImuSamplePipelineDeps& deps,
                                                const Lsm6dsv::Sample& scaled,
                                                bool hasBaseGyroBiasModel,
                                                const Vec3& currentGyroBiasRadS) {
    Lsm6dsv::Sample calibrated = imuPipelineMakeSensorFrameCalibratedSample(
        deps, scaled, hasBaseGyroBiasModel, currentGyroBiasRadS);
    if (deps.sensorToDeviceFrameCache != nullptr) {
        const SensorToDeviceFrame& frame = deps.sensorToDeviceFrameCache->resolve(
            deps.config.data.crc32,
            deps.config.data.frame.sensorToDeviceValid,
            deps.config.data.frame.sensorToDevice);
        calibrated.gyro_rad_s = frame.apply(calibrated.gyro_rad_s);
        calibrated.accel_g = frame.apply(calibrated.accel_g);
    } else {
        const SensorToDeviceFrame fallbackFrame = makeSensorToDeviceFrame(
            deps.config.data.frame.sensorToDeviceValid,
            deps.config.data.frame.sensorToDevice);
        calibrated.gyro_rad_s = fallbackFrame.apply(calibrated.gyro_rad_s);
        calibrated.accel_g = fallbackFrame.apply(calibrated.accel_g);
    }
    return calibrated;
}

void imuPipelineRecordSampleProcessTime(ImuSamplePipelineDeps& deps, uint32_t dtUs) {
#if TRACKER_HAS_HOTPATH_PERF
    deps.perf.sampleProcessCalls++;
    deps.perf.sampleProcessSumUs += dtUs;
    if (dtUs > deps.perf.sampleProcessMaxUs) {
        deps.perf.sampleProcessMaxUs = dtUs;
    }
#if TRACKER_HAS_STATIC_TEST
    if (deps.staticTestRunner != nullptr) {
        deps.staticTestRunner->recordSampleProcessTime(dtUs);
    }
#endif
#else
    (void)deps;
    (void)dtUs;
#endif
}

bool imuPipelineUpdateRuntimeGyroBiasEstimator(ImuSamplePipelineDeps& deps,
                                               const Lsm6dsv::Sample& scaled,
                                               const Lsm6dsv::Sample& calibrated,
                                               const ImuQualityResult& quality,
                                               uint64_t timestampUs) {
    const GyroTempCompRuntimeEval tempEval =
        deps.gyroTempComp.evaluateRuntime(calibrated.temp_c);
    return imuPipelineUpdateRuntimeGyroBiasEstimator(
        deps, scaled, calibrated, quality, timestampUs,
        tempEval,
        runtimeBiasCurrentGyroBiasRadS(
            deps.runtimeBias, deps.imuCal, tempEval));
}

bool imuPipelineUpdateRuntimeGyroBiasEstimator(ImuSamplePipelineDeps& deps,
                                               const Lsm6dsv::Sample& scaled,
                                               const Lsm6dsv::Sample& calibrated,
                                               const ImuQualityResult& quality,
                                               uint64_t timestampUs,
                                               const GyroTempCompRuntimeEval& tempEval,
                                               const Vec3& currentGyroBiasRadS) {
    if (!deps.runtimeBias.enabled) {
        return false;
    }

    RuntimeGyroBiasUpdateDeps biasDeps{
        deps.runtimeBias,
        deps.imuCal,
        deps.gyroTempComp,
        deps.ahrs,
        deps.trackingState.recoveryActive(),
        deps.logState != nullptr && deps.logState->enabled(),
        deps.logState != nullptr ? &deps.logState->sequence : nullptr,
        deps.logCounters,
        &deps.out
    };
    return runtimeBiasUpdateEstimator(
        biasDeps, scaled, calibrated, quality, timestampUs,
        tempEval, currentGyroBiasRadS);
}

bool imuPipelineEmitPerSampleOutputs(ImuSamplePipelineDeps& deps,
                                     const Lsm6dsv::RawSample& raw,
                                     const Lsm6dsv::Sample& scaled,
                                     const Lsm6dsv::Sample& calibrated,
                                     const ImuQualityResult& quality) {
    const GyroTempCompRuntimeEval tempEval =
        deps.gyroTempComp.evaluateRuntime(calibrated.temp_c);
    return imuPipelineEmitPerSampleOutputs(
        deps, raw, scaled, calibrated, quality,
        tempEval,
        runtimeBiasCurrentGyroBiasRadS(
            deps.runtimeBias, deps.imuCal, tempEval));
}

bool imuPipelineEmitPerSampleOutputs(ImuSamplePipelineDeps& deps,
                                     const Lsm6dsv::RawSample& raw,
                                     const Lsm6dsv::Sample& scaled,
                                     const Lsm6dsv::Sample& calibrated,
                                     const ImuQualityResult& quality,
                                     const GyroTempCompRuntimeEval& tempEval,
                                     const Vec3& currentGyroBiasRadS) {
#if TRACKER_HAS_SERIAL_STREAM
    if (deps.streamState != nullptr &&
        deps.streamState->mode != TrackerStreamMode::Off &&
        deps.streamState->mode != TrackerStreamMode::Heartbeat) {
        // Do not read the MCU clock on every 960 Hz sample merely to discover
        // that the production serial stream is disabled.
        emitSerialStreamIfNeeded(
            *deps.streamState, deps.out, raw, scaled, calibrated,
            deps.ahrs, quality, micros());
    }
#endif
    if (deps.callbacks.emitMachineLogFrame != nullptr) {
        deps.callbacks.emitMachineLogFrame(raw, calibrated, quality, deps.callbacks.user);
    }
#if TRACKER_HAS_RUNTIME_PROFILER
    if (deps.motionDiagnostics != nullptr && deps.motionDiagnostics->enabled()) {
        deps.motionDiagnostics->recordSample(raw, calibrated, quality);
    }
#endif
    const bool hasCoherentAccel = quality.shouldUseAccelCorrection && quality.accelNormValid;
#if TRACKER_HAS_CALIBRATION_AUTONOMY
    if (deps.calibrationAutonomy != nullptr &&
        deps.calibrationAutonomy->imuObservationRequired()) {
        // Feed native sensor-frame physical samples to the RAM-only learner.
        // The controller owns no NVS operation on this hot path.
        deps.calibrationAutonomy->observeImuSample(
            scaled, quality, raw.t_us,
            static_cast<uint32_t>(raw.t_us / 1000ULL));
    }
#endif
#if TRACKER_HAS_STATIC_TEST
    const bool staticTestCaptureActive =
        deps.staticTestRunner != nullptr && deps.staticTestRunner->active();
#else
    constexpr bool staticTestCaptureActive = false;
#endif
#if TRACKER_HAS_CALIBRATION_UI
    const bool gyroTempCaptureActive =
        deps.gyroTempCapture != nullptr && deps.gyroTempCapture->active();
#else
    constexpr bool gyroTempCaptureActive = false;
#endif
#if TRACKER_HAS_STATIC_TEST || TRACKER_HAS_CALIBRATION_UI
    if (hasCoherentAccel && (staticTestCaptureActive || gyroTempCaptureActive)) {
        // Temperature models are stored in native sensor frame. Construct this
        // additional sample only while a real capture is active; normal
        // tracking must not pay the calibration transform/millis cost.
        const Lsm6dsv::Sample sensorFrameCalibrated =
            imuPipelineMakeSensorFrameCalibratedSample(deps, scaled);
#if TRACKER_HAS_STATIC_TEST
        if (staticTestCaptureActive) {
            deps.staticTestRunner->updateSample(sensorFrameCalibrated, quality, deps.out);
        }
#endif
#if TRACKER_HAS_CALIBRATION_UI
        if (gyroTempCaptureActive) {
            deps.gyroTempCapture->updateSample(sensorFrameCalibrated, quality, millis());
        }
#endif
    }
#endif
    if (hasCoherentAccel) {
        return imuPipelineUpdateRuntimeGyroBiasEstimator(
            deps, scaled, calibrated, quality, raw.t_us,
            tempEval, currentGyroBiasRadS);
    }
    return false;
}

FifoRuntimeSampleResult imuSamplePipelineProcessRaw(ImuSamplePipelineDeps& deps,
                                                    const Lsm6dsv::RawSample& raw,
                                                    bool checkFifoStatsDelta) {
#if TRACKER_HAS_RUNTIME_PROFILER
    const bool profileImuStages = deps.runtimeProfiler != nullptr &&
        deps.runtimeProfiler->enabled() &&
        (deps.runtimeSamples % TRACKER_IMU_STAGE_PROFILER_SAMPLE_DIVISOR) == 0u;
    uint32_t imuStageStartUs = profileImuStages ? micros() : 0u;
    const auto finishImuStage = [&](RuntimeProfiler::ImuStage stage) {
        if (!profileImuStages) return;
        const uint32_t nowUs = micros();
        deps.runtimeProfiler->recordImuStage(stage, nowUs - imuStageStartUs);
        imuStageStartUs = nowUs;
    };
#endif
    const uint32_t softwareQueueAgeUs = deps.fifoRuntime != nullptr
        ? deps.fifoRuntime->lastDequeuedQueueAgeUs()
        : 0u;
#if TRACKER_HAS_RUNTIME_PROFILER
    if (deps.runtimeProfiler != nullptr) {
        deps.runtimeProfiler->recordProcessedSoftwareAge(softwareQueueAgeUs);
    }
#endif
#if TRACKER_HAS_HOTPATH_PERF
    const uint32_t sampleProcessStartUs = micros();
#endif

    imuPipelineUpdateLatestTemperature(deps);

    Lsm6dsv::Sample scaled = deps.lsm.scale(raw);
    scaled.temp_c = deps.latestTempC;
    const GyroTempCompRuntimeEval tempEval =
        deps.gyroTempComp.evaluateRuntime(deps.latestTempC);
    const bool hasBaseGyroBiasModel =
        tempEval.valid || deps.imuCal.gyroBiasValid;
    const Vec3 currentGyroBiasRadS = runtimeBiasCurrentGyroBiasRadS(
        deps.runtimeBias, deps.imuCal, tempEval);
    Lsm6dsv::Sample calibrated = imuPipelineMakeCalibratedSample(
        deps, scaled, hasBaseGyroBiasModel, currentGyroBiasRadS);
    if ((raw.components & Lsm6dsv::SAMPLE_COMPONENT_ACCEL) == 0u) {
        // A gyro-only continuity sample must not expose an accel value
        // synthesized by applying bias/matrix calibration to zero raw
        // counts. Quality/recovery already reject the missing component;
        // zeroing it here also keeps diagnostics and future consumers safe.
        scaled.accel_g = Vec3::zero();
        calibrated.accel_g = Vec3::zero();
    }
#if TRACKER_HAS_RUNTIME_PROFILER
    finishImuStage(RuntimeProfiler::ImuStage::ScaleAndCalibration);
#endif

    if (deps.lastScaledSample != nullptr) {
        *deps.lastScaledSample = scaled;
    }
    if (deps.lastCalibratedSample != nullptr) {
        *deps.lastCalibratedSample = calibrated;
    }
    if (deps.lastImuSampleSequence != nullptr) {
        (*deps.lastImuSampleSequence)++;
        if (*deps.lastImuSampleSequence == 0u) {
            *deps.lastImuSampleSequence = 1u;
        }
    }

    const auto& fifoStats = deps.fifo.stats();
    ImuQualityResult quality = deps.qualityMonitor.evaluate(raw, calibrated, fifoStats, checkFifoStatsDelta);
    runtimeBiasApplyGyroTempQualityFlags(tempEval, quality);
#if TRACKER_HAS_RUNTIME_PROFILER
    finishImuStage(RuntimeProfiler::ImuStage::Quality);
#endif

    const bool unreconstructableGap =
        trackingTimestampGapRequiresRecovery(quality, deps.ahrs.config().maxDtS);

    if (quality.shouldRequestFifoRecovery) {
        deps.runtimeSamples++;
        deps.lastSampleTimestampUs = raw.t_us;
        deps.lastOutputConfidence = quality.overallConfidence;
        if (deps.lastQualityFlags != nullptr) {
            *deps.lastQualityFlags = quality.flags;
        }
        deps.preparedOutput.reset();
#if TRACKER_HAS_RUNTIME_PROFILER
        finishImuStage(RuntimeProfiler::ImuStage::AhrsAndRecovery);
        finishImuStage(RuntimeProfiler::ImuStage::PreparedOutput);
#endif

        (void)imuPipelineEmitPerSampleOutputs(
            deps, raw, scaled, calibrated, quality,
            tempEval, currentGyroBiasRadS);
#if TRACKER_HAS_RUNTIME_PROFILER
        finishImuStage(RuntimeProfiler::ImuStage::PerSampleOutputs);
#endif

        if (deps.callbacks.maybeRecoverFifo != nullptr) {
            deps.callbacks.maybeRecoverFifo(quality, raw, deps.callbacks.user);
        }

#if TRACKER_HAS_HOTPATH_PERF
        const uint32_t processUs = micros() - sampleProcessStartUs;
        imuPipelineRecordSampleProcessTime(deps, processUs);
#endif
        return FifoRuntimeSampleResult::FifoRecovered;
    }

    bool ahrsIntegrated = false;
    if (unreconstructableGap) {
        if (deps.callbacks.enterTrackingRecovery != nullptr) {
            deps.callbacks.enterTrackingRecovery(quality.flags, "unreconstructable_dt_gap", raw.t_us, deps.callbacks.user);
        }
    } else if (quality.shouldUpdateAhrs) {
        const bool recoveryNeedsBootstrap = trackingRecoveryNeedsAhrsBootstrap(
            deps.trackingState.recoveryActive(), deps.ahrs.initialized());
        if (deps.trackingState.recoveryActive() && !recoveryNeedsBootstrap) {
            // Keep integrating post-gap gyro so heading changes made while
            // waiting for a short rest are retained. Accel correction remains
            // disabled until tilt is reacquired from a stable mean vector.
            ahrsIntegrated = deps.ahrs.update(
                calibrated.gyro_rad_s, Vec3::zero(), 0.0f, raw.t_us);
        } else {
            // Normal startup, including the defensive case where strict
            // recovery was requested before AHRS had any quaternion, must be
            // allowed to initialize from gravity.
            const Vec3 accelForAhrs = quality.accelForAhrs(calibrated.accel_g);
            const float accelNormForAhrs = quality.shouldUseAccelCorrection ? quality.accelNormG : 0.0f;
            ahrsIntegrated = deps.ahrs.update(
                calibrated.gyro_rad_s, accelForAhrs, accelNormForAhrs, raw.t_us);
        }
    }

    deps.runtimeSamples++;
    deps.lastSampleTimestampUs = raw.t_us;
    deps.lastOutputConfidence = quality.overallConfidence;
    if (deps.lastQualityFlags != nullptr) {
        *deps.lastQualityFlags = quality.flags;
    }

    if (deps.callbacks.updateTrackingRecoveryState != nullptr) {
        deps.callbacks.updateTrackingRecoveryState(quality,
                                                   calibrated.gyro_rad_s,
                                                   calibrated.accel_g,
                                                   raw.t_us,
                                                   ahrsIntegrated,
                                                   deps.callbacks.user);
    }
#if TRACKER_HAS_RUNTIME_PROFILER
    finishImuStage(RuntimeProfiler::ImuStage::AhrsAndRecovery);
#endif

    if (deps.trackingState.recoveryActive()) {
        deps.preparedOutput.reset();
    } else {
        const bool preparedPublished = deps.preparedOutput.update(deps.config,
                                                                  deps.runtimeSamples,
                                                                  raw.t_us,
                                                                  deps.ahrs,
                                                                  quality,
                                                                  calibrated.accel_g,
                                                                  softwareQueueAgeUs);
#if TRACKER_HAS_RUNTIME_PROFILER
        if (preparedPublished && deps.runtimeProfiler != nullptr) {
            deps.runtimeProfiler->recordPreparedSoftwareAge(softwareQueueAgeUs);
        }
#else
        (void)preparedPublished;
#endif
    }
#if TRACKER_HAS_RUNTIME_PROFILER
    finishImuStage(RuntimeProfiler::ImuStage::PreparedOutput);
#endif

    const bool deferredBiasWindowReady = imuPipelineEmitPerSampleOutputs(
        deps, raw, scaled, calibrated, quality,
        tempEval, currentGyroBiasRadS);
#if TRACKER_HAS_RUNTIME_PROFILER
    finishImuStage(RuntimeProfiler::ImuStage::PerSampleOutputs);
#endif

#if TRACKER_HAS_HOTPATH_PERF
    const uint32_t processUs = micros() - sampleProcessStartUs;
    imuPipelineRecordSampleProcessTime(deps, processUs);
#endif
    return deferredBiasWindowReady
        ? FifoRuntimeSampleResult::YieldRequested
        : FifoRuntimeSampleResult::Continue;
}

} // namespace tracker
