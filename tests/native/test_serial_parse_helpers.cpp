#include "test_common.hpp"

#include <cstring>

#include "serial/tracker_serial_parse.hpp"
#include "serial/tracker_serial_commands.hpp"
#include "serial/tracker_command_line_parser.hpp"

int main() {
    TestContext ctx;
    using namespace tracker::tracker_serial_detail;

    CHECK(ctx, eqIgnoreCase("status", "STATUS"));
    CHECK(ctx, eqIgnoreCase("Mag", "mAg"));
    CHECK(ctx, !eqIgnoreCase("mag", "magnet"));
    CHECK(ctx, startsWithIgnoreCase("quality stats", "QUALITY"));
    CHECK(ctx, !startsWithIgnoreCase("quality", "quality stats"));

    bool b = false;
    CHECK(ctx, parseBool("on", b) && b);
    CHECK(ctx, parseBool("FALSE", b) && !b);
    CHECK(ctx, !parseBool("maybe", b));

    uint32_t u = 0;
    CHECK(ctx, parseU32("240", u) && u == 240u);
    CHECK(ctx, parseU32("0x10", u) && u == 16u);
    CHECK(ctx, !parseU32("12x", u));
    CHECK(ctx, !parseU32("", u));

    float f = 0.0f;
    CHECK(ctx, parseFloat("1.25", f));
    CHECK_NEAR(ctx, f, 1.25f, 0.000001f);
    CHECK(ctx, parseFloat("-3.5", f));
    CHECK_NEAR(ctx, f, -3.5f, 0.000001f);
    CHECK(ctx, !parseFloat("nan", f));
    CHECK(ctx, !parseFloat("1.0x", f));

    tracker::TrackerTelnetInputFilter telnet;
    CHECK(ctx, telnet.consume(0xffu));
    CHECK(ctx, telnet.consume(0xfdu));
    CHECK(ctx, telnet.consume(0x01u));
    CHECK(ctx, !telnet.consume(static_cast<uint8_t>('l')));
    CHECK(ctx, telnet.consume(0xffu));
    CHECK(ctx, telnet.consume(0xfau));
    CHECK(ctx, telnet.consume(0x18u));
    CHECK(ctx, telnet.consume(0x00u));
    CHECK(ctx, telnet.consume(0xffu));
    CHECK(ctx, telnet.consume(0xf0u));
    CHECK(ctx, !telnet.consume(static_cast<uint8_t>('o')));
    CHECK(ctx, telnet.consume(0u));

    {
        char line[] = "net set ssid \"living room tracker\" save";
        char* argv[10]{};
        const auto parsed = tracker::trackerParseCommandLine(line, argv, 10u);
        CHECK(ctx, parsed.status == tracker::TrackerCommandLineParseStatus::Ok);
        CHECK(ctx, parsed.argc == 5u);
        CHECK(ctx, std::strcmp(argv[3], "living room tracker") == 0);
    }
    {
        char line[] = "net set ssid \"unclosed";
        char* argv[10]{};
        const auto parsed = tracker::trackerParseCommandLine(line, argv, 10u);
        CHECK(ctx, parsed.status == tracker::TrackerCommandLineParseStatus::UnclosedQuote);
    }
    {
        char line[] = "one two three";
        char* argv[2]{};
        const auto parsed = tracker::trackerParseCommandLine(line, argv, 2u);
        CHECK(ctx, parsed.status == tracker::TrackerCommandLineParseStatus::TooManyArguments);
    }
    {
        char line[] = "net set ssid \"ok\"garbage";
        char* argv[10]{};
        const auto parsed = tracker::trackerParseCommandLine(line, argv, 10u);
        CHECK(ctx, parsed.status ==
            tracker::TrackerCommandLineParseStatus::TrailingCharactersAfterQuote);
    }

    return ctx.finish("test_serial_parse_helpers");
}
