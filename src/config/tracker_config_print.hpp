#pragma once

#include <Arduino.h>

#include "core/math.hpp"
#include "config/tracker_config_store.hpp"
#include "config/tracker_network_config.hpp"

namespace tracker {

inline void printTrackerConfigSummary(Stream& out, const TrackerConfig& cfg) {
    out.println("==============================================================================");
    out.println("TRACKER CONFIG SUMMARY");
    out.println("==============================================================================");
    out.print("magic=0x"); out.print(cfg.data.magic, HEX);
    out.print(" version="); out.print(cfg.data.version);
    out.print(" size="); out.print(cfg.data.size);
    out.print(" crc=0x"); out.println(cfg.data.crc32, HEX);
    out.print("schema hardware/imu/fifo/ahrs/gyro/accel/mag/magyaw/quality/output/frame=");
    out.print(cfg.data.schemas.hardware); out.print('/');
    out.print(cfg.data.schemas.imu); out.print('/');
    out.print(cfg.data.schemas.fifo); out.print('/');
    out.print(cfg.data.schemas.ahrs); out.print('/');
    out.print(cfg.data.schemas.gyroCal); out.print('/');
    out.print(cfg.data.schemas.accelCal); out.print('/');
    out.print(cfg.data.schemas.magCal); out.print('/');
    out.print(cfg.data.schemas.magYaw); out.print('/');
    out.print(cfg.data.schemas.quality); out.print('/');
    out.print(cfg.data.schemas.output); out.print('/');
    out.println(cfg.data.schemas.frame);

    out.println("-- hardware --");
    out.print("pins sck/miso/mosi/cs/int1=");
    out.print(cfg.data.hardware.pinLsmSck); out.print('/');
    out.print(cfg.data.hardware.pinLsmMiso); out.print('/');
    out.print(cfg.data.hardware.pinLsmMosi); out.print('/');
    out.print(cfg.data.hardware.pinLsmCs); out.print('/');
    out.println(cfg.data.hardware.pinLsmInt1);
    out.print("serialBaud="); out.print(cfg.data.hardware.serialBaud);
    out.print(" spiHz="); out.println(cfg.data.hardware.spiHz);

    out.println("-- imu/fifo --");
    out.print("imuOdrHz="); out.println(Lsm6dsv::odrHz(cfg.data.imu.imuOdr), 3);
    out.print("fifoAccelBdrHz="); out.println(Lsm6dsv::odrHz(cfg.data.fifo.accelBdr), 3);
    out.print("fifoGyroBdrHz="); out.println(Lsm6dsv::odrHz(cfg.data.fifo.gyroBdr), 3);
    out.print("samplePeriodUsOverride="); out.println(cfg.data.fifo.samplePeriodUsOverride, 3);
    out.print("fifoWatermarkWords="); out.println(cfg.data.fifo.watermarkWords);
    out.print("maxWordsPerDrain="); out.println(cfg.data.fifo.maxWordsPerDrain);
    out.print("maxDrainRoundsPerEvent="); out.println(cfg.data.fifo.maxDrainRoundsPerEvent);
    out.print("useHardwareTimestamps="); out.println(cfg.data.fifo.useHardwareTimestamps ? "yes" : "no");
    out.print("timestampFallback="); out.println(cfg.data.fifo.allowTimestampFallback ? "yes" : "no");

    out.println("-- ahrs effective config --");
    out.print("ahrsAccelCorrectionEnabled="); out.println((cfg.data.ahrs.useAccelCorrection && cfg.data.ahrsRuntime.accelCorrectionEnabled) ? "yes" : "no");
    out.print("ahrsAdaptiveAccelCorrection="); out.println(cfg.data.ahrsRuntime.adaptiveAccelCorrection ? "yes" : "no");
    out.print("ahrsAccelKp="); out.println(cfg.data.ahrsRuntime.accelKp, 6);
    out.print("ahrsDtMinMaxS="); out.print(cfg.data.ahrsRuntime.minDtS, 7); out.print(','); out.println(cfg.data.ahrsRuntime.maxDtS, 7);
    out.print("ahrsClampLargeDt="); out.println(cfg.data.ahrsRuntime.clampLargeDt ? "yes" : "no");
    out.print("ahrsMaxAccelCorrectionDeg="); out.println(cfg.data.ahrsRuntime.maxAccelCorrectionDegPerUpdate, 6);
    out.print("ahrsAccelNormGoodBadErrG="); out.print(cfg.data.ahrsRuntime.accelNormGoodErrorG, 6); out.print(','); out.println(cfg.data.ahrsRuntime.accelNormBadErrorG, 6);
    out.print("ahrsAccelInnovationGoodBadDeg="); out.print(cfg.data.ahrsRuntime.accelInnovationGoodDeg, 3); out.print(','); out.println(cfg.data.ahrsRuntime.accelInnovationBadDeg, 3);
    out.print("ahrsAccelNormStdGoodBadG="); out.print(cfg.data.ahrsRuntime.accelNormStdGoodG, 6); out.print(','); out.println(cfg.data.ahrsRuntime.accelNormStdBadG, 6);
    out.print("ahrsGyroMotionGoodBadDps="); out.print(cfg.data.ahrsRuntime.gyroMotionGoodDps, 3); out.print(','); out.println(cfg.data.ahrsRuntime.gyroMotionBadDps, 3);
    out.print("ahrsNormalizeEvery="); out.println(cfg.data.ahrsRuntime.normalizeEvery);

    out.println("-- gyro calibration --");
    out.print("gyroBiasValid="); out.println(cfg.data.gyroCal.biasValid ? "yes" : "no");
    out.print("gyroBiasRadS=");
    out.print(cfg.data.gyroCal.biasRadS.x, 8); out.print(',');
    out.print(cfg.data.gyroCal.biasRadS.y, 8); out.print(',');
    out.println(cfg.data.gyroCal.biasRadS.z, 8);
    out.print("gyroBiasDps=");
    const Vec3 gyroBiasDps = cfg.data.gyroCal.biasRadS * MATH_RAD_TO_DEG;
    out.print(gyroBiasDps.x, 5); out.print(',');
    out.print(gyroBiasDps.y, 5); out.print(',');
    out.println(gyroBiasDps.z, 5);
    out.print("tempCompValid="); out.println(cfg.data.gyroCal.tempCompValid ? "yes" : "no");
    out.print("tempCompEnabled="); out.println(cfg.data.gyroCal.tempCompEnabled ? "yes" : "no");
    out.print("referenceTempC="); out.println(cfg.data.gyroCal.referenceTempC, 3);
    const Vec3 tempSlopeDps = cfg.data.gyroCal.tempSlopeRadSPerC * MATH_RAD_TO_DEG;
    out.print("tempSlopeDpsPerC=");
    out.print(tempSlopeDps.x, 8); out.print(',');
    out.print(tempSlopeDps.y, 8); out.print(',');
    out.println(tempSlopeDps.z, 8);
    out.print("tempRangeMinMaxC="); out.print(cfg.data.gyroTempQuality.tempRangeMinC, 3); out.print(','); out.println(cfg.data.gyroTempQuality.tempRangeMaxC, 3);
    out.print("tempFitQuality="); out.println(cfg.data.gyroTempQuality.fitQuality, 6);
    out.print("tempFitResidualBeforeAfterDps="); out.print(cfg.data.gyroTempQuality.residualBeforeDps, 6); out.print(','); out.println(cfg.data.gyroTempQuality.residualAfterDps, 6);
    out.print("gyroBiasCalibrationUptimeMs="); out.println(cfg.data.gyroCalMeta.biasCalibrationUptimeMs);
    out.print("gyroTempModelUpdatedUptimeMs="); out.println(cfg.data.gyroCalMeta.tempModelUpdatedUptimeMs);
    out.print("gyroTempModelSampleCount="); out.println(cfg.data.gyroCalMeta.tempModelSampleCount);

    out.println("-- accel calibration --");
    out.print("accelCalValid="); out.println(cfg.data.accelCal.valid ? "yes" : "no");
    out.print("accelBiasG=");
    out.print(cfg.data.accelCal.biasG.x, 8); out.print(',');
    out.print(cfg.data.accelCal.biasG.y, 8); out.print(',');
    out.println(cfg.data.accelCal.biasG.z, 8);
    out.print("accelScaleDiag=");
    out.print(cfg.data.accelCal.scale.m[0][0], 8); out.print(',');
    out.print(cfg.data.accelCal.scale.m[1][1], 8); out.print(',');
    out.println(cfg.data.accelCal.scale.m[2][2], 8);
    out.print("accelScaleMatrix=");
    out.print(cfg.data.accelCal.scale.m[0][0], 8); out.print(',');
    out.print(cfg.data.accelCal.scale.m[0][1], 8); out.print(',');
    out.print(cfg.data.accelCal.scale.m[0][2], 8); out.print(';');
    out.print(cfg.data.accelCal.scale.m[1][0], 8); out.print(',');
    out.print(cfg.data.accelCal.scale.m[1][1], 8); out.print(',');
    out.print(cfg.data.accelCal.scale.m[1][2], 8); out.print(';');
    out.print(cfg.data.accelCal.scale.m[2][0], 8); out.print(',');
    out.print(cfg.data.accelCal.scale.m[2][1], 8); out.print(',');
    out.println(cfg.data.accelCal.scale.m[2][2], 8);
    out.print("accelCalQualityScore="); out.println(cfg.data.accelCalQuality.qualityScore, 6);
    out.print("accelCalQualityFlags=0x"); out.println(cfg.data.accelCalQuality.qualityFlags, HEX);
    out.print("accelCalUptimeMs="); out.println(cfg.data.accelCalQuality.calibrationUptimeMs);
    out.print("accelCalMaxFaceNormErrorG="); out.println(cfg.data.accelCalQuality.maxFaceNormErrorG, 8);
    out.print("accelCalMaxAxisResidualG="); out.println(cfg.data.accelCalQuality.maxAxisResidualG, 8);
    out.print("accelCalFaceSamples=");
    for (uint8_t i = 0; i < 6; ++i) { if (i) out.print(','); out.print(cfg.data.accelCalQuality.faceSamples[i]); }
    out.println();

    out.println("-- magnetometer --");
    out.print("magDriverEnabled="); out.println(cfg.data.magCal.driverEnabled ? "yes" : "no");
    out.print("magCalibrationValid="); out.println(cfg.data.magCal.calibrationValid ? "yes" : "no");
    out.print("magAxisAlignmentValid="); out.println(cfg.data.magCal.axisAlignmentValid ? "yes" : "no");
    out.print("magHardIron=");
    out.print(cfg.data.magCal.hardIron.x, 6); out.print(',');
    out.print(cfg.data.magCal.hardIron.y, 6); out.print(',');
    out.println(cfg.data.magCal.hardIron.z, 6);
    out.print("magExpectedFieldNorm="); out.println(cfg.data.magCal.expectedFieldNorm, 6);
    out.print("magTrustNormMinMax=");
    out.print(cfg.data.magCal.minTrustNorm, 6); out.print(',');
    out.println(cfg.data.magCal.maxTrustNorm, 6);
    out.print("magCalUptimeMs="); out.println(cfg.data.magCalQuality.calibrationUptimeMs);
    out.print("magCalSamplesRejectedSaturated="); out.print(cfg.data.magCalQuality.sampleCount); out.print(','); out.print(cfg.data.magCalQuality.rejectedSamples); out.print(','); out.println(cfg.data.magCalQuality.saturatedSamples);
    out.print("magCalRadiusXYZ="); out.print(cfg.data.magCalQuality.radiusX, 3); out.print(','); out.print(cfg.data.magCalQuality.radiusY, 3); out.print(','); out.println(cfg.data.magCalQuality.radiusZ, 3);
    out.print("magCalNormMinMeanMax="); out.print(cfg.data.magCalQuality.normMin, 3); out.print(','); out.print(cfg.data.magCalQuality.normMean, 3); out.print(','); out.println(cfg.data.magCalQuality.normMax, 3);
    out.print("magCalCoverageScore="); out.println(cfg.data.magCalQuality.coverageScore, 6);

    out.println("-- frame/device --");
    out.print("sensorToDeviceValid="); out.println(cfg.data.frame.sensorToDeviceValid ? "yes" : "no");
    out.print("applyMountingOffsetInFirmware="); out.println(cfg.data.frame.applyMountingOffsetInFirmware ? "yes" : "no");
    out.print("outputConvention="); out.println(cfg.data.frame.outputConvention);
    out.print("deviceId="); out.println(cfg.data.device.deviceId);
    out.print("sensorId="); out.println(cfg.data.device.sensorId);
    out.print("deviceName="); out.println(cfg.data.device.deviceName);

    out.println("-- mag yaw correction --");
    out.print("magYawControllerEnabled="); out.println(cfg.data.magYaw.controllerEnabled ? "yes" : "no");
    out.print("magYawApplyEnabled="); out.println(cfg.data.magYaw.applyEnabled ? "yes" : "no");
    out.print("magYawRequireAccelTrusted="); out.println(cfg.data.magYaw.requireAccelTrusted ? "yes" : "no");

    out.print("magYawMaxInnovationDeg="); out.println(cfg.data.magYaw.maxInnovationDeg, 3);

    out.print("magYawHorizontalBadGood=");
    out.print(cfg.data.magYaw.horizontalNormBad, 3); out.print(',');
    out.println(cfg.data.magYaw.horizontalNormGood, 3);

    out.print("magYawGyroGoodBadDps=");
    out.print(cfg.data.magYaw.gyroNormGoodDps, 3); out.print(',');
    out.println(cfg.data.magYaw.gyroNormBadDps, 3);

    out.print("magYawAccelTrustBadGood=");
    out.print(cfg.data.magYaw.accelTrustBad, 3); out.print(',');
    out.println(cfg.data.magYaw.accelTrustGood, 3);

    out.print("magYawMaxAgeMs="); out.println(cfg.data.magYaw.maxMagAgeMs);
    out.print("magYawTimeConstantS="); out.println(cfg.data.magYaw.timeConstantS, 3);
    out.print("magYawMaxRateDegS="); out.println(cfg.data.magYaw.maxCorrectionRateDegS, 3);
    out.print("magYawMaxStepDeg="); out.println(cfg.data.magYaw.maxCorrectionStepDeg, 3);
    out.print("magYawCooldownGyroAccelMagMs="); out.print(cfg.data.magYaw.gyroMovingCooldownMs); out.print(','); out.print(cfg.data.magYaw.accelBadCooldownMs); out.print(','); out.println(cfg.data.magYaw.magDisturbanceCooldownMs);
    out.print("magYawFallbackDtS="); out.println(cfg.data.magYaw.fallbackDtS, 6);

    out.println("-- quality --");
    out.print("largeGapFactor="); out.println(cfg.data.quality.largeGapFactor, 3);
    out.print("gyroNearSatRaw="); out.println(cfg.data.quality.gyroNearSaturationAbsRaw);
    out.print("accelNearSatRaw="); out.println(cfg.data.quality.accelNearSaturationAbsRaw);

    out.println("-- output --");
    out.print("serialDebugEnabled="); out.println(cfg.data.output.serialDebugEnabled ? "yes" : "no");
    out.print("quaternionOutputEnabled="); out.println(cfg.data.output.quaternionOutputEnabled ? "yes" : "no");
    out.print("outputRateHz="); out.println(cfg.data.output.outputRateHz);
    out.print("packetFormat="); out.println(cfg.data.output.packetFormat);

    out.println("-- future network storage --");
    out.print("networkNvsNamespace="); out.println(tracker_network_detail::NVS_NAMESPACE);
    out.print("networkNvsKey="); out.println(tracker_network_detail::NVS_KEY_CONFIG);
    out.println("networkRuntimeWired=no");
    out.println("==============================================================================");
}

inline void printTrackerNetworkConfigSummary(Stream& out, const TrackerNetworkConfig& cfg, bool revealSecrets = false) {
    out.println("==============================================================================");
    out.println("TRACKER NETWORK CONFIG SUMMARY");
    out.println("==============================================================================");
    out.print("magic=0x"); out.print(cfg.data.magic, HEX);
    out.print(" version="); out.print(cfg.data.version);
    out.print(" size="); out.print(cfg.data.size);
    out.print(" crc=0x"); out.println(cfg.data.crc32, HEX);
    out.print("wifiEnabled="); out.println(cfg.data.wifiEnabled ? "yes" : "no");
    out.print("credentialsValid="); out.println(cfg.data.credentialsValid ? "yes" : "no");
    out.print("ssid="); out.println(cfg.data.ssid);
    out.print("password="); out.println(revealSecrets ? cfg.data.password : "<hidden>");
    out.print("discoveryEnabled="); out.println(cfg.data.discoveryEnabled ? "yes" : "no");
    out.print("manualServerEnabled="); out.println(cfg.data.manualServerEnabled ? "yes" : "no");
    out.print("serverHost="); out.println(cfg.data.serverHost);
    out.print("serverPort="); out.println(cfg.data.serverPort);
    out.print("deviceId="); out.println(cfg.data.deviceId);
    out.print("sensorId="); out.println(cfg.data.sensorId);
    out.print("deviceName="); out.println(cfg.data.deviceName);
    out.println("==============================================================================");
}

} // namespace tracker
