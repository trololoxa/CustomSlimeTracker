#include "runtime/runtime_profiler.hpp"
#include "test_common.hpp"

using namespace tracker;

int main() {
    TestContext t;

    RuntimeLatencyHistogram histogram;
    histogram.record(0u);
    histogram.record(9u);
    histogram.record(26u);
    histogram.record(9000u);
    histogram.record(60000u);
    histogram.record(550347u);
    CHECK(t, histogram.count == 6u);
    CHECK(t, histogram.maxUs == 550347u);
    CHECK(t, histogram.percentileUpperBoundUs(1u) == 0u);
    CHECK(t, histogram.percentileUpperBoundUs(50u) == 50u);
    CHECK(t, histogram.percentileUpperBoundUs(100u) == 750000u);
    RuntimeLatencyHistogram overflowHistogram;
    overflowHistogram.record(3000000u);
    CHECK(t, overflowHistogram.percentileUpperBoundUs(50u) == 3000000u);

    RuntimeProfiler profiler;
    profiler.begin(0u);
    profiler.setEnabled(true, 0u);

    profiler.record(RuntimeProfiler::Section::Fifo, 3200u, true, 1u);
    profiler.record(RuntimeProfiler::Section::Fifo, 7100u, false, 2u);
    const auto& fifo = profiler.stats(RuntimeProfiler::Section::Fifo);
    CHECK(t, fifo.calls == 2u);
    CHECK(t, fifo.worked == 1u);
    CHECK(t, fifo.maxUs == 7100u);
    CHECK(t, fifo.latency.percentileUpperBoundUs(50u) == 3500u);
    CHECK(t, fifo.latency.percentileUpperBoundUs(100u) == 7500u);

    // One 6 ms interval in the first frame, then a 4 ms interval crossing the
    // 10 ms boundary. The first frame is completely busy; the second retains
    // 6 ms headroom.
    profiler.recordLoopInterval(0u, 6000u);
    profiler.recordLoopInterval(6000u, 4000u);
    profiler.recordLoopInterval(10000u, 4000u);
    CHECK(t, profiler.frameStats().completed == 1u);
    CHECK(t, profiler.frameStats().overBudget == 1u);
    CHECK(t, profiler.frameStats().busyMaxUs == 10000u);
    CHECK(t, profiler.frameStats().headroom.percentileUpperBoundUs(1u) == 0u);

    profiler.recordProcessedSoftwareAge(1200u);
    profiler.recordPreparedSoftwareAge(1800u);
    profiler.recordRotationSoftwareAge(4200u);
    profiler.recordProfilerOverhead(37u);
    CHECK(t, profiler.processedSoftwareAge().count == 1u);
    CHECK(t, profiler.preparedSoftwareAge().maxUs == 1800u);
    CHECK(t, profiler.rotationSoftwareAge().maxUs == 4200u);
    CHECK(t, profiler.profilerOverhead().maxUs == 37u);
    profiler.recordImuStage(RuntimeProfiler::ImuStage::Quality, 44u);
    profiler.recordImuStage(RuntimeProfiler::ImuStage::Quality, 66u);
    CHECK(t, profiler.imuStageStats(RuntimeProfiler::ImuStage::Quality).samples == 2u);
    CHECK(t, profiler.imuStageStats(RuntimeProfiler::ImuStage::Quality).maxUs == 66u);
    profiler.recordOptionalServiceAdmissionSkip(RuntimeProfiler::OptionalService::Battery);

    // 32-bit micros rollover must preserve frame progression.
    profiler.reset(0u);
    profiler.recordLoopInterval(0xfffffff0u, 16u);
    profiler.recordLoopInterval(0u, 10000u);
    CHECK(t, profiler.frameStats().completed >= 1u);

    return t.finish("runtime_profiler");
}
