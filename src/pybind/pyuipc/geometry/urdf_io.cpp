#include <pyuipc/geometry/urdf_io.h>
#include <uipc/io/urdf_io.h>
#include <pyuipc/as_numpy.h>
#include <pyuipc/common/json.h>

namespace pyuipc::geometry
{
using namespace uipc::io;
PyUrdfIO::PyUrdfIO(py::module& m)
{
    auto class_UrdfController = py::class_<UrdfController>(m, "UrdfController");
    class_UrdfController.def(
        "rotate_to", &UrdfController::rotate_to, py::arg("joint_name"), py::arg("angle"));
    class_UrdfController.def("apply_to", &UrdfController::apply_to, py::arg("attr"));
    class_UrdfController.def("revolute_joints", &UrdfController::revolute_joints);
    class_UrdfController.def("links",
                             [](UrdfController& self)
                             {
                                 auto     links = self.links();
                                 py::list list;
                                 for(auto& link : links)
                                 {
                                     list.append(link);
                                 }
                                 return list;
                             });
    class_UrdfController.def("sync_visual_mesh", &UrdfController::sync_visual_mesh);
    class_UrdfController.def(
        "move_root",
        [](const UrdfController& self, py::array_t<Float> xyz, py::array_t<Float> rpy)
        {
            auto v_xyz = to_matrix<Vector3>(xyz);
            auto v_rpy = to_matrix<Vector3>(rpy);
            self.move_root(v_xyz, v_rpy);
        });

    auto class_UrdfIO = py::class_<UrdfIO>(m, "UrdfIO");
    class_UrdfIO.def(py::init<const Json&>(), py::arg("config") = UrdfIO::default_config());
    class_UrdfIO.def("read", &UrdfIO::read, py::arg("object"), py::arg("urdf_path"));
    class_UrdfIO.def_static("default_config", &UrdfIO::default_config);
}
}  // namespace pyuipc::geometry
