#include <pyuipc/pyuipc.h>

namespace pyuipc
{
class ModuleLoader
{
  public:
    static void load(py::module& m)
    {
        auto& creators = Module::creators();
        for(auto& creator : creators)
        {
            creator(m);
        }
    }
};

std::string remove_project_prefix(std::string_view path)
{
    // find last "pyuipc" in path
    auto pos = path.rfind("pyuipc");
    if(pos == std::string::npos)
    {
        return std::string(path);
    }
    return std::string(path.substr(pos));
}
}  // namespace pyuipc

PYBIND11_MODULE(pyuipc, m)
{
    m.doc() = "pyuipc python binding";
    pyuipc::ModuleLoader::load(m);
}
