#include "serial/tracker_slimevr_commands.hpp"

#include <Arduino.h>

#include "config/tracker_network_config.hpp"
#include "network/udp_transport.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

namespace {

Stream& outFor(TrackerSerialCommandContext& ctx) {
    return ctx.io ? *ctx.io : Serial;
}

const char* yn(bool v) { return v ? "yes" : "no"; }

SlimeVROutputRuntimeConfig makeConfigFromNetwork(const TrackerNetworkConfig& net) {
    SlimeVROutputRuntimeConfig cfg;
    cfg.enabled = true;
    cfg.discoveryEnabled = net.data.discoveryEnabled;
    cfg.manualServerEnabled = net.data.manualServerEnabled;
    cfg.deviceName = net.data.deviceName;
    cfg.sensorId = net.data.sensorId;
    cfg.serverPort = net.data.serverPort;
    cfg.localPort = SLIMEVR_DISCOVERY_LOCAL_PORT;
    cfg.discoveryIntervalMs = 1000;
    cfg.incomingPacketsPerUpdate = 4;
    return cfg;
}

void printSlimeStatus(Stream& out, const SlimeVROutputRuntimeStatus& s) {
    char ipBuf[24];
    out.println("# SLIMEVR STATUS");
    out.print("enabled="); out.println(yn(s.enabled));
    out.print("state="); out.println(slimevrOutputStateName(s.state));
    out.print("wifi_connected="); out.println(yn(s.wifiConnected));
    out.print("udp_ready="); out.println(yn(s.udpReady));
    out.print("local_port="); out.println(s.localPort);
    out.print("discovery_enabled="); out.println(yn(s.discoveryEnabled));
    out.print("manual_server_enabled="); out.println(yn(s.manualServerEnabled));
    out.print("server_found="); out.println(yn(s.serverFound));
    out.print("server_ip="); out.println(s.serverIpv4 ? udpIpv4ToCString(s.serverIpv4, ipBuf, sizeof(ipBuf)) : "0.0.0.0");
    out.print("server_port="); out.println(s.serverPort);
    out.print("sensor_id="); out.println(s.sensorId);
    out.print("protocol_version="); out.println(s.protocolVersion);
    out.print("board_type="); out.println(s.boardType);
    out.print("imu_type="); out.println(s.imuType);
    out.print("mcu_type="); out.println(s.mcuType);
    out.print("handshakes_sent="); out.println(s.handshakesSent);
    out.print("sensor_info_sent="); out.println(s.sensorInfoSent);
    out.print("heartbeat_sent="); out.println(s.heartbeatSent);
    out.print("packets_received="); out.println(s.packetsReceived);
    out.print("discovery_responses="); out.println(s.discoveryResponses);
    out.print("send_failures="); out.println(s.sendFailures);
    out.print("udp_begin_failures="); out.println(s.udpBeginFailures);
    out.print("last_handshake_ms="); out.println(s.lastHandshakeMs);
    out.print("last_incoming_packet_ms="); out.println(s.lastIncomingPacketMs);
    out.print("last_state_change_ms="); out.println(s.lastStateChangeMs);
}

void printHelp(Stream& out) {
    out.println("slime status");
    out.println("slime start");
    out.println("slime stop");
    out.println("slime reconnect");
    out.println("slime counters reset");
}

} // namespace

bool trackerSerialDispatchSlimeVRCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    if (argc <= 0 || !argv || !argv[0]) return false;
    if (!tracker_serial_detail::eqIgnoreCase(argv[0], "slime") &&
        !tracker_serial_detail::eqIgnoreCase(argv[0], "slimevr")) {
        return false;
    }

    Stream& out = outFor(ctx);
    if (!ctx.slimevrRuntime) {
        tracker_serial_detail::printErr(out, "SlimeVR runtime not available");
        return true;
    }

    if (argc < 2 || tracker_serial_detail::eqIgnoreCase(argv[1], "status")) {
        printSlimeStatus(out, ctx.slimevrRuntime->status());
        return true;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "help") || tracker_serial_detail::eqIgnoreCase(argv[1], "?")) {
        printHelp(out);
        return true;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "start")) {
        if (!ctx.networkConfig) {
            tracker_serial_detail::printErr(out, "network config not available");
            return true;
        }
        ctx.networkConfig->sanitize();
        ctx.slimevrRuntime->configure(makeConfigFromNetwork(*ctx.networkConfig));
        tracker_serial_detail::printOk(out, "SlimeVR discovery started");
        out.println("# use: slime status");
        return true;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "stop")) {
        ctx.slimevrRuntime->stop();
        tracker_serial_detail::printOk(out, "SlimeVR discovery stopped");
        return true;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "reconnect") || tracker_serial_detail::eqIgnoreCase(argv[1], "restart")) {
        if (!ctx.networkConfig) {
            tracker_serial_detail::printErr(out, "network config not available");
            return true;
        }
        ctx.networkConfig->sanitize();
        ctx.slimevrRuntime->configure(makeConfigFromNetwork(*ctx.networkConfig));
        ctx.slimevrRuntime->restart();
        tracker_serial_detail::printOk(out, "SlimeVR discovery restarted");
        return true;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "counters")) {
        if (argc >= 3 && tracker_serial_detail::eqIgnoreCase(argv[2], "reset")) {
            ctx.slimevrRuntime->resetCounters();
            tracker_serial_detail::printOk(out, "SlimeVR counters reset");
            return true;
        }
        tracker_serial_detail::printErr(out, "usage: slime counters reset");
        return true;
    }

    tracker_serial_detail::printErr(out, "unknown slime command; use slime help");
    return true;
}

} // namespace tracker
