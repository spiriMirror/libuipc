#include <pyuipc/common/future.h>
#include <uipc/common/future.h>

namespace pyuipc
{
class Future
{
  public:
    Future(uipc::Future&& f)
        : m_handle(std::move(f))
    {
    }

    static Future launch(py::function func)
    {
        return Future(std::async(std::launch::async,
                                 [func]()
                                 {
                                     py::gil_scoped_acquire acquire;
                                     func();
                                 }));
    }

    void wait() { m_handle.wait(); }

    bool is_ready() const
    {
        return m_handle.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    }

  private:
    uipc::Future m_handle;
};

PyFuture::PyFuture(py::module& m)
{
    auto class_Future = py::class_<Future>(m, "Future");

    class_Future.def(
        "wait",
        [](Future& future) { future.wait(); },
        "Wait for the future to be ready.",
        py::call_guard<py::gil_scoped_release>());

    class_Future.def(
        "is_ready",
        [](const Future& future) { return future.is_ready(); },
        py::call_guard<py::gil_scoped_release>());

    class_Future.def_static(
        "launch",
        [](py::function func) -> Future
        {
            if(!py::isinstance<py::function>(func))
            {
                throw py::type_error("The argument must be a callable");
            }
            return Future::launch(func);
        },
        py::call_guard<py::gil_scoped_release>());
}
}  // namespace pyuipc
