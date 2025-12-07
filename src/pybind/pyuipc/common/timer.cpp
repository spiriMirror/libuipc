#include <pyuipc/common/timer.h>
#include <uipc/common/timer.h>
#include <pyuipc/common/json.h>

namespace pyuipc
{
using namespace uipc;
PyTimer::PyTimer(py::module& m)
{
    auto class_Timer = py::class_<Timer>(m, "Timer",
                                         R"(Timer class for performance profiling and timing measurements.)");

    class_Timer.def_static("enable_all", &Timer::enable_all,
                          R"(Enable all timers for performance measurement.)");
    class_Timer.def_static("disable_all", &Timer::disable_all,
                          R"(Disable all timers.)");
    class_Timer.def_static("report", []() { Timer::report(); },
                          R"(Print timing report to the console.)");
    class_Timer.def_static("report_as_json", Timer::report_as_json,
                          R"(Get timing report as a JSON object.
Returns:
    dict: Timing report as a dictionary.)");
}
}  // namespace pyuipc
