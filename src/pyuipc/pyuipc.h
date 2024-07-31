#pragma once
#include <pybind11/pybind11.h>
#include <functional>
#include <uipc/common/type_define.h>

namespace pyuipc
{
using namespace uipc;
namespace py = pybind11;
class Module
{
  public:
    Module(std::function<void(py::module&)>&& reg)
    {
        creators().push_back(reg);
    }
    friend class ModuleLoader;

  private:
    using Creator = std::function<void(py::module&)>;
    static std::list<Creator>& creators()
    {
        static std::list<Creator> creators;
        return creators;
    }
};

std::string remove_project_prefix(std::string_view path);
}  // namespace pyuipc

#ifdef UIPC_RUNTIME_CHECK
#define PYUIPC_MSG(...)                                                        \
    (fmt::format(__VA_ARGS__) + fmt::format(R"(. Source '{}({})')", __FILE__, __LINE__))
#else
#define PYUIPC_MSG(...)                                                        \
    (fmt::format(__VA_ARGS__)                                                  \
     + fmt::format(R"(. Source '{}({})')", remove_project_prefix(__FILE__), __LINE__))
#endif