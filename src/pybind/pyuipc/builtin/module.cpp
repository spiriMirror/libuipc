#include <pyuipc/builtin/module.h>
#include <pyuipc/builtin/uid_info.h>
#include <pyuipc/builtin/uid_register.h>
#include <pyuipc/builtin/constitution_uid_collection.h>
#include <pyuipc/builtin/implicit_geometry_uid_collection.h>
#include <pyuipc/builtin/attribute_name.h>
#include <pyuipc/builtin/constants.h>
#include <pyuipc/builtin/geometry_type.h>
#include <pyuipc/builtin/constitution_type.h>

namespace pyuipc::builtin
{
PyModule::PyModule(py::module& m)
{
    PyAttributeName{m};
    PyConstants{m};
    PyGeometryType{m};
    PyConstitutionType{m};

    PyUIDInfo{m};
    PyUIDRegister{m};
    PyConstitutionUIDCollection{m};
    PyImplicitGeometryUIDCollection{m};
}
}  // namespace pyuipc::builtin
