#include <fmt/core.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pyuipc/usd/module.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/base/tf/weakPtr.h>
#include <pxr/base/tf/pyObjWrapper.h>
#include <pxr/external/boost/python.hpp>

namespace pyuipc::usd
{
PyModule::PyModule(py::module& m)
{
    // [TODO]: Now I just add a test API for checking the casting of TfWeakPtr<UsdStage> from Python
    // need further implement useful APIs

    namespace pbpy = PXR_BOOST_PYTHON_NAMESPACE;

    m.def("print_stage",
          [](py::object stage_obj)
          {
              PyObject* pyObj = stage_obj.ptr();
              auto boost_pyobj = pbpy::object(pbpy::handle<>(pbpy::borrowed(pyObj)));

              auto weak_ptr = pbpy::extract<pxr::UsdStagePtr>(boost_pyobj);
              if(!weak_ptr.check())
              {
                  throw std::invalid_argument("Argument is not a TfWeakPtr<UsdStage>");
              }
              pxr::UsdStagePtr stage_weak = weak_ptr();
              // traverse the stage and print all prims
              if(!stage_weak)
              {
                  fmt::print("Stage pointer is expired or null.\n");
                  return;
              }
              for(const auto& prim : stage_weak->Traverse())
              {
                  fmt::print("Prim: {}\n", prim.GetPath().GetString());
              }
          });
}
}  // namespace pyuipc::usd