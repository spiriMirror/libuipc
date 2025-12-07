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
    auto class_ResidentThread = py::class_<ResidentThread>(m, "ResidentThread",
                                                             R"(Resident thread for executing Python callables in a separate thread.)");
    class_ResidentThread.def(py::init<>(),
                             R"(Create a new resident thread.)");
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
                        logger::error("Exception in ResidentThread:", e.what());
                    }
                });
        },
        py::arg("func"),
        R"(Post a callable to be executed in the resident thread.
Args:
    func: Python callable object (function) with no arguments to execute.
Returns:
    bool: True if the function was successfully posted.)");
    class_ResidentThread.def("is_ready", &ResidentThread::is_ready,
                            R"(Check if the resident thread is ready to accept new tasks.
Returns:
    bool: True if the thread is ready, False otherwise.)");
}
}  // namespace pyuipc
