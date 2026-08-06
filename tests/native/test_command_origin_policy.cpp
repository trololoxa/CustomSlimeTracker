#include "test_common.hpp"

#include <cstdio>
#include <initializer_list>

#include "serial/tracker_command_origin.hpp"

using namespace tracker;

namespace {

bool allowed(std::initializer_list<const char*> args) {
    char storage[10][32] = {};
    char* argv[10] = {};
    int argc = 0;
    for (const char* arg : args) {
        std::snprintf(storage[argc], sizeof(storage[argc]), "%s", arg);
        argv[argc] = storage[argc];
        ++argc;
    }
    return trackerRemoteDiagnosticCommandAllowed(argc, argv);
}

} // namespace

int main() {
    TestContext ctx;

    CHECK(ctx, allowed({"version"}));
    CHECK(ctx, allowed({"log", "start", "full"}));
    CHECK(ctx, allowed({"log", "rate", "20"}));
    CHECK(ctx, allowed({"test", "static", "600"}));
    CHECK(ctx, allowed({"test", "summary", "static"}));
    CHECK(ctx, allowed({"test", "summary", "runtime"}));
    CHECK(ctx, allowed({"perf", "tracking", "reset"}));
    CHECK(ctx, allowed({"console", "reset"}));
    CHECK(ctx, allowed({"MAG", "STATUS"}));

    CHECK(ctx, !allowed({"reboot"}));
    CHECK(ctx, !allowed({"factory_reset"}));
    CHECK(ctx, !allowed({"FRST"}));
    CHECK(ctx, !allowed({"SET", "WIFI", "ssid", "secret"}));
    CHECK(ctx, !allowed({"config", "save"}));
    CHECK(ctx, !allowed({"cal", "erase_all", "confirm"}));
    CHECK(ctx, !allowed({"net", "set", "pass", "secret", "save"}));
    CHECK(ctx, !allowed({"remote", "off"}));
    CHECK(ctx, !allowed({"mag", "cal", "start"}));
    CHECK(ctx, !allowed({"log", "start", "debug"}));
    CHECK(ctx, !allowed({"test", "static", "600", "extra"}));
    CHECK(ctx, !allowed({"test", "report", "static"}));
    CHECK(ctx, !allowed({"test", "summary", "static", "extra"}));
    CHECK(ctx, !allowed({"perf", "on"}));
    CHECK(ctx, !allowed({"motion", "on"}));
    CHECK(ctx, !allowed({"imu", "read"}));

    return ctx.finish("command_origin_policy");
}
