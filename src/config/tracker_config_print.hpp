#pragma once

#include <Arduino.h>

#include "build_config/feature_flags.hpp"
#include "config/tracker_config_runtime.hpp"
#include "config/tracker_network_config.hpp"

namespace tracker {

#if TRACKER_ENABLE_FULL_CONFIG_PRINT

void printTrackerConfigSummary(Stream& out, const TrackerConfig& cfg);
void printTrackerNetworkConfigSummary(Stream& out, const TrackerNetworkConfig& cfg, bool revealSecrets = false);

#else

inline void printTrackerConfigSummary(Stream& out, const TrackerConfig& cfg) {
    out.println("# CONFIG SUMMARY COMPACT");
    out.print("valid="); out.println(cfg.validate() ? "yes" : "no");
    out.print("version="); out.println(cfg.data.version);
    out.print("size="); out.println(cfg.data.size);
    out.print("crc=0x"); out.println(cfg.data.crc32, HEX);
    out.print("spi_hz="); out.println(cfg.data.hardware.spiHz);
    out.print("output_rate_hz="); out.println(cfg.data.output.outputRateHz);
    out.print("slimevr_motion_mode=");
    out.println(slimevrMotionPacketPolicyName(cfg.slimevrMotionPacketPolicy()));
    out.print("gyro_bias_valid="); out.println(cfg.data.gyroCal.biasValid ? "yes" : "no");
    out.print("accel_cal_valid="); out.println(cfg.data.accelCal.valid ? "yes" : "no");
    out.print("mag_driver_enabled="); out.println(cfg.data.magCal.driverEnabled ? "yes" : "no");
    out.print("mag_cal_valid="); out.println(cfg.data.magCal.calibrationValid ? "yes" : "no");
    out.print("mag_axis_valid="); out.println(cfg.data.magCal.axisAlignmentValid ? "yes" : "no");
    out.println("# full config print is not compiled in this profile");
}

inline void printTrackerNetworkConfigSummary(Stream& out, const TrackerNetworkConfig& cfg, bool revealSecrets = false) {
    (void)revealSecrets;
    out.println("# NETWORK CONFIG SUMMARY COMPACT");
    out.print("valid="); out.println(cfg.validate() ? "yes" : "no");
    out.print("device_name="); out.println(cfg.data.deviceName);
    out.print("ssid_set="); out.println(cfg.data.ssid[0] != '\0' ? "yes" : "no");
    out.print("manual_server_enabled="); out.println(cfg.data.manualServerEnabled ? "yes" : "no");
    out.print("server_port="); out.println(cfg.data.serverPort);
    out.print("discovery_enabled="); out.println(cfg.data.discoveryEnabled ? "yes" : "no");
    out.println("# full network config print is not compiled in this profile");
}

#endif

} // namespace tracker
