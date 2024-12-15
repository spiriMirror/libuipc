#include <pyuipc/common/timer.h>
#include <uipc/common/timer.h>
#include <pyuipc/common/json.h>

namespace pyuipc
{
using namespace uipc;
PyTimer::PyTimer(py::module& m)
{
    auto class_Timer = py::class_<Timer>(m, "Timer");

    class_Timer.def_static("enable_all", &Timer::enable_all);
    class_Timer.def_static("disable_all", &Timer::disable_all);
    class_Timer.def_static("report", []() { Timer::report(); });
    class_Timer.def_static("report_as_json", Timer::report_as_json);
}
}  // namespace pyuipc
