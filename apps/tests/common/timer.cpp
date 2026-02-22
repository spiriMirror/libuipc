#include <app/app.h>
#include <uipc/common/timer.h>
#include <thread>
#include <chrono>

using namespace uipc;

TEST_CASE("timer_statistics", "[timer]")
{
    Timer::enable_all();

    GlobalTimer gt{"test_timer"};
    gt.set_as_current();

    // Run the same named block multiple times to accumulate samples
    constexpr int N = 5;
    for(int i = 0; i < N; ++i)
    {
        Timer t{"work"};
        // small sleep so each sample has a measurable positive duration
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    Json j = gt.report_merged_as_json();

    // The JSON root is the "test_timer" node
    REQUIRE(!j.is_null());

    // Find the "work" child
    auto& children = j["children"];
    REQUIRE(!children.empty());

    auto& work = children[0];
    REQUIRE(work["name"] == "work");
    REQUIRE(work["count"] == N);

    // Statistical fields must be present
    REQUIRE(work.contains("avg"));
    REQUIRE(work.contains("min"));
    REQUIRE(work.contains("max"));
    REQUIRE(work.contains("stddev"));

    double total  = work["duration"].get<double>();
    double avg    = work["avg"].get<double>();
    double min_d  = work["min"].get<double>();
    double max_d  = work["max"].get<double>();
    double stddev = work["stddev"].get<double>();

    // avg = total / N
    REQUIRE(std::abs(avg - total / N) < 1e-12);

    // min <= avg <= max
    REQUIRE(min_d <= avg + 1e-12);
    REQUIRE(avg <= max_d + 1e-12);

    // stddev >= 0
    REQUIRE(stddev >= 0.0);

    Timer::disable_all();
}
