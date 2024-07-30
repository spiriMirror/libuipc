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
}  // namespace pyuipc

PYBIND11_MODULE(pyuipc, m)
{
    m.doc() = "pyuipc python binding";
    pyuipc::ModuleLoader::load(m);
}
