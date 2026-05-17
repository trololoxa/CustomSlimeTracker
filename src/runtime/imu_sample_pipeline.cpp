#include "runtime/imu_sample_pipeline.hpp"

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

Lsm6dsv::Sample imuPipelineMakeCalibratedSample(const ImuSamplePipelineDeps& deps,
                                                const Lsm6dsv::Sample& scaled) {
    Lsm6dsv::Sample calibrated = scaled;
    calibrated.temp_c = deps.latestTempC;

    if (runtimeBiasHasBaseGyroBiasModel(deps.imuCal, deps.gyroTempComp)) {
        calibrated.gyro_rad_s = scaled.gyro_rad_s - imuPipelineCurrentGyroBiasRadS(deps, deps.latestTempC);
    }

    if (deps.imuCal.accelCalValid) {
        calibrated.accel_g = deps.imuCal.applyAccel(scaled.accel_g);
    }

    return calibrated;
}

void imuPipelineRecordSampleProcessTime(ImuSamplePipelineDeps& deps, uint32_t dtUs) {
    deps.perf.sampleProcessCalls++;
    deps.perf.sampleProcessSumUs += dtUs;
    if (dtUs > deps.perf.sampleProcessMaxUs) {
        deps.perf.sampleProcessMaxUs = dtUs;
    }
    deps.staticTestRunner.recordSampleProcessTime(dtUs);
}

void imuPipelineUpdateRuntimeGyroBiasEstimator(ImuSamplePipelineDeps& deps,
                                               const Lsm6dsv::Sample& scaled,
                                               const Lsm6dsv::Sample& calibrated,
                                               const ImuQualityResult& quality,
                                               uint64_t timestampUs) {
    if (!deps.runtimeBias.enabled) {
        return;
    }

    RuntimeGyroBiasUpdateDeps biasDeps{
        deps.runtimeBias,
        deps.imuCal,
        deps.gyroTempComp,
        deps.ahrs,
        deps.trackingState.recoveryActive(),
        deps.logState.enabled(),
        &deps.logState.sequence,
        &deps.logCounters,
        &deps.out
    };
    runtimeBiasUpdateEstimator(biasDeps, scaled, calibrated, quality, timestampUs);
}

void imuPipelineEmitPerSampleOutputs(ImuSamplePipelineDeps& deps,
                                     const Lsm6dsv::RawSample& raw,
                                     const Lsm6dsv::Sample& scaled,
                                     const Lsm6dsv::Sample& calibrated,
                                     const ImuQualityResult& quality) {
    emitSerialStreamIfNeeded(deps.streamState, deps.out, raw, scaled, calibrated, deps.ahrs, quality, micros());
    if (deps.callbacks.emitMachineLogFrame != nullptr) {
        deps.callbacks.emitMachineLogFrame(raw, calibrated, quality, deps.callbacks.user);
    }
    deps.staticTestRunner.updateSample(calibrated, quality, deps.out);
    if (deps.gyroTempCapture != nullptr) {
        deps.gyroTempCapture->updateSample(calibrated, quality, millis());
    }
    imuPipelineUpdateRuntimeGyroBiasEstimator(deps, scaled, calibrated, quality, raw.t_us);
}

FifoRuntimeSampleResult imuSamplePipelineProcessRaw(ImuSamplePipelineDeps& deps,
                                                    const Lsm6dsv::RawSample& raw,
                                                    bool checkFifoStatsDelta) {
    const uint32_t sampleProcessStartUs = micros();

    imuPipelineUpdateLatestTemperature(deps);

    Lsm6dsv::Sample scaled = deps.lsm.scale(raw);
    scaled.temp_c = deps.latestTempC;
    Lsm6dsv::Sample calibrated = imuPipelineMakeCalibratedSample(deps, scaled);

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
    runtimeBiasApplyGyroTempQualityFlags(deps.gyroTempComp, quality, calibrated.temp_c);

    const bool largeGap = quality.has(imu_quality_flags::TIMESTAMP_LARGE_GAP);

    if (quality.shouldRequestFifoRecovery) {
        deps.runtimeSamples++;
        deps.lastSampleTimestampUs = raw.t_us;
        deps.lastOutputConfidence = quality.overallConfidence;
        if (deps.lastQualityFlags != nullptr) {
            *deps.lastQualityFlags = quality.flags;
        }
        deps.preparedOutput.update(deps.config, deps.runtimeSamples, raw.t_us, deps.ahrs, quality);

        imuPipelineEmitPerSampleOutputs(deps, raw, scaled, calibrated, quality);

        if (deps.callbacks.maybeRecoverFifo != nullptr) {
            deps.callbacks.maybeRecoverFifo(quality, raw, deps.callbacks.user);
        }

        const uint32_t processUs = micros() - sampleProcessStartUs;
        imuPipelineRecordSampleProcessTime(deps, processUs);
        return FifoRuntimeSampleResult::FifoRecovered;
    }

    if (largeGap) {
        if (deps.callbacks.enterTrackingRecovery != nullptr) {
            deps.callbacks.enterTrackingRecovery(quality.flags, "large_dt_gap", raw.t_us, deps.callbacks.user);
        }
    } else if (quality.shouldUpdateAhrs) {
        const Vec3 accelForAhrs = quality.accelForAhrs(calibrated.accel_g);
        const float accelNormForAhrs = quality.shouldUseAccelCorrection ? quality.accelNormG : 0.0f;
        deps.ahrs.update(calibrated.gyro_rad_s, accelForAhrs, accelNormForAhrs, raw.t_us);
    }

    deps.runtimeSamples++;
    deps.lastSampleTimestampUs = raw.t_us;
    deps.lastOutputConfidence = quality.overallConfidence;
    if (deps.lastQualityFlags != nullptr) {
        *deps.lastQualityFlags = quality.flags;
    }
    deps.preparedOutput.update(deps.config, deps.runtimeSamples, raw.t_us, deps.ahrs, quality);

    imuPipelineEmitPerSampleOutputs(deps, raw, scaled, calibrated, quality);

    if (deps.callbacks.updateTrackingRecoveryState != nullptr) {
        deps.callbacks.updateTrackingRecoveryState(quality, deps.callbacks.user);
    }

    const uint32_t processUs = micros() - sampleProcessStartUs;
    imuPipelineRecordSampleProcessTime(deps, processUs);
    return FifoRuntimeSampleResult::Continue;
}

} // namespace tracker
