#include <pyuipc/backend/buffer.h>
#include <uipc/backend/buffer.h>
#include <uipc/common/log.h>

namespace pyuipc::backend
{
using namespace uipc::backend;
PyBuffer::PyBuffer(py::module& m)
{
    auto class_Buffer = py::class_<Buffer>(m, "Buffer");

    class_Buffer.def(py::init(
        [](std::function<void(SizeT)> resize_func, std::function<BufferView()> get_buffer_view_func)
        {
            return Buffer{[resize_func](SizeT size)
                          {
                              try
                              {
                                  resize_func(size);
                              }
                              catch(const std::exception& e)
                              {
                                  spdlog::error(PYUIPC_MSG("Error in resize_func: {}",
                                                           e.what()));
                              }
                          },
                          [get_buffer_view_func]() -> BufferView
                          {
                              BufferView bv;
                              try
                              {
                                  bv = get_buffer_view_func();
                              }
                              catch(const std::exception& e)
                              {
                                  spdlog::error(PYUIPC_MSG("Error in get_buffer_view_func: {}",
                                                           e.what()));
                              }
                              return bv;
                          }};
        }));

    class_Buffer.def("resize", &Buffer::resize);
    class_Buffer.def("view", &Buffer::view);
}
}  // namespace pyuipc::backend