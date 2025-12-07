#include <pyuipc/geometry/urdf_io.h>
#include <uipc/io/urdf_io.h>
#include <pyuipc/as_numpy.h>
#include <pyuipc/common/json.h>

namespace pyuipc::geometry
{
using namespace uipc::io;
PyUrdfIO::PyUrdfIO(py::module& m)
{
    auto class_UrdfController = py::class_<UrdfController>(m, "UrdfController",
                                                            R"(UrdfController class for controlling URDF robot models.)");
    class_UrdfController.def(
        "rotate_to", &UrdfController::rotate_to, py::arg("joint_name"), py::arg("angle"),
        R"(Set the rotation angle of a revolute joint.
Args:
    joint_name: Name of the joint.
    angle: Rotation angle in radians.)");
    class_UrdfController.def("apply_to", &UrdfController::apply_to, py::arg("attr"),
                            R"(Apply controller state to attributes.
Args:
    attr: Attribute collection to apply to.)");
    class_UrdfController.def("revolute_joints", &UrdfController::revolute_joints,
                           R"(Get the list of revolute joint names.
Returns:
    list: List of revolute joint names.)");
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
                             },
                             R"(Get the list of link names.
Returns:
    list: List of link names.)");
    class_UrdfController.def("sync_visual_mesh", &UrdfController::sync_visual_mesh,
                            R"(Synchronize visual mesh with current joint positions.)");
    class_UrdfController.def(
        "move_root",
        [](const UrdfController& self, py::array_t<Float> xyz, py::array_t<Float> rpy)
        {
            auto v_xyz = to_matrix<Vector3>(xyz);
            auto v_rpy = to_matrix<Vector3>(rpy);
            self.move_root(v_xyz, v_rpy);
        },
        py::arg("xyz"),
        py::arg("rpy"),
        R"(Move the root link of the robot.
Args:
    xyz: Translation vector (3D).
    rpy: Rotation vector (roll, pitch, yaw in radians).)");

    auto class_UrdfIO = py::class_<UrdfIO>(m, "UrdfIO",
                                           R"(UrdfIO class for loading URDF robot models.)");
    class_UrdfIO.def(py::init<const Json&>(), py::arg("config") = UrdfIO::default_config(),
                    R"(Create a UrdfIO instance.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");
    class_UrdfIO.def("read", &UrdfIO::read, py::arg("object"), py::arg("urdf_path"),
                    R"(Read a URDF file and populate an object.
Args:
    object: Object to populate with URDF data.
    urdf_path: Path to the URDF file.
Returns:
    UrdfController: Controller for the loaded URDF model.)");
    class_UrdfIO.def_static("default_config", &UrdfIO::default_config,
                           R"(Get the default UrdfIO configuration.
Returns:
    dict: Default configuration dictionary.)");
}
}  // namespace pyuipc::geometry
