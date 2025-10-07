#include <pyuipc/backend/buffer.h>
#include <uipc/backend/buffer.h>
#include <uipc/common/log.h>

namespace pyuipc::backend
{
using namespace uipc::backend;
PyBuffer::PyBuffer(py::module& m)
{
    // allow add attributes to this class
    auto class_Buffer = py::class_<Buffer>(m, "Buffer", py::dynamic_attr());

    class_Buffer.def(
        py::init(
            [](py::function resize_func, py::function get_buffer_view_func)
            {
                return Buffer{[resize_func](SizeT size)
                              {
                                  try
                                  {
                                      resize_func(size);
                                  }
                                  catch(const std::exception& e)
                                  {
                                      log::error(PYUIPC_MSG("Error in resize_func: {}",
                                                               e.what()));
                                  }
                              },
                              [get_buffer_view_func]() -> BufferView
                              {
                                  BufferView bv;
                                  try
                                  {
                                      bv = py::cast<BufferView>(get_buffer_view_func());
                                  }
                                  catch(const std::exception& e)
                                  {
                                      log::error(PYUIPC_MSG("Error in get_buffer_view_func: {}",
                                                               e.what()));
                                  }
                                  return bv;
                              }};
            }),
        py::arg("resize_func"),
        py::arg("get_buffer_view_func"),
        R"(Constructs a Buffer object with provided resize and get_buffer_view functions.
Args:
    resize_func f:(int)->None: Function to resize the buffer.
    get_buffer_view_func f:()->BufferView: Function to retrieve the buffer view.)");

    class_Buffer.def("resize", &Buffer::resize);
    class_Buffer.def("view", &Buffer::view);
}
}  // namespace pyuipc::backend