#include <pyuipc/common/resident_thread.h>
#include <uipc/common/resident_thread.h>

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>

namespace pyuipc
{
using namespace uipc;
PyResidentThread::PyResidentThread(py::module& m)
{
    auto class_ResidentThread = py::class_<ResidentThread>(m, "ResidentThread");
    class_ResidentThread.def(py::init<>());
    class_ResidentThread.def(
        "post",
        [](ResidentThread& self, py::function func)
        {
            // func must be a callable object with no arguments
            PYUIPC_ASSERT(py::isinstance<py::function>(func), "func must be a callable object");
            return self.post(
                [func]()
                {
                    py::gil_scoped_acquire acquire;
                    try
                    {
                        func();
                    }
                    catch(const std::exception& e)
                    {
                        log::error("Exception in ResidentThread:", e.what());
                    }
                });
        },
        py::arg("func"),
        "Post a callable to be executed in the resident thread");
    class_ResidentThread.def("is_ready", &ResidentThread::is_ready);
}
}  // namespace pyuipc
