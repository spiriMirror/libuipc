#include <pyuipc/core/animator.h>
#include <uipc/core/animator.h>
#include <pyuipc/as_numpy.h>

namespace pyuipc::core
{
using namespace uipc::core;
PyAnimator::PyAnimator(py::module& m)
{
    auto class_Animation = py::class_<Animation>(m, "Animation",
                                                  R"(Animation class for managing object animations.)");
    auto class_UpdateHint = py::class_<Animation::UpdateHint>(class_Animation, "UpdateHint",
                                                               R"(Update hint enumeration for animation updates.)");

    auto class_UpdateInfo = py::class_<Animation::UpdateInfo>(class_Animation, "UpdateInfo",
                                                               R"(Update information passed to animation callbacks.)");

    class_UpdateInfo
        .def("object", &Animation::UpdateInfo::object, py::return_value_policy::reference_internal,
             R"(Get the object being animated.
Returns:
    Object: Reference to the object.)")
        .def("geo_slots",
             [](Animation::UpdateInfo& self) -> py::list
             {
                 py::list list;
                 for(auto slot : self.geo_slots())
                 {
                     list.append(py::cast(slot));
                 }
                 return list;
             },
             R"(Get the geometry slots.
Returns:
    list: List of geometry slots.)")
        .def("rest_geo_slots",
             [](Animation::UpdateInfo& self) -> py::list
             {
                 py::list list;
                 for(auto slot : self.rest_geo_slots())
                 {
                     list.append(py::cast(slot));
                 }
                 return list;
             },
             R"(Get the rest geometry slots.
Returns:
    list: List of rest geometry slots.)")
        .def("frame", &Animation::UpdateInfo::frame,
             R"(Get the current frame number.
Returns:
    int: Current frame number.)")
        .def("hint", &Animation::UpdateInfo::hint,
             R"(Get the update hint.
Returns:
    UpdateHint: Update hint value.)")
        .def("dt", &Animation::UpdateInfo::dt,
             R"(Get the time step.
Returns:
    float: Time step (delta time).)");

    auto class_Animator = py::class_<Animator>(m, "Animator",
                                               R"(Animator class for managing object animations in a scene.)");
    class_Animator.def("insert",
                       [](Animator& self, Object& obj, py::function callable)
                       {
                           if(!py::isinstance<py::function>(callable))
                           {
                               throw py::type_error("The second argument must be a callable");
                           }
                           self.insert(obj,
                                       [callable](Animation::UpdateInfo& info)
                                       {
                                           py::gil_scoped_acquire acquire;
                                           try
                                           {
                                               // acquire gil
                                               callable(py::cast(info));
                                           }
                                           catch(const std::exception& e)
                                           {
                                               logger::error("Python Animation Script Error in Object [{}]({}):\n{}",
                                                             info.object().name(),
                                                             info.object().id(),
                                                             e.what());
                                           }
                                       });
                       },
                       py::arg("object"),
                       py::arg("callable"),
                       R"(Insert an animation callback for an object.
Args:
    object: Object to animate.
    callable: Python function that takes an UpdateInfo argument and updates the object.)");
    class_Animator.def("erase", &Animator::erase,
                      py::arg("object"),
                      R"(Remove animation callback for an object.
Args:
    object: Object to remove animation from.)");

    class_Animator.def(
        "substep", [](Animator& self, SizeT substep) { self.substep(substep); },
        py::arg("substep"),
        R"(Set the substep count.
Args:
    substep: Number of substeps per frame.)");

    class_Animator.def("substep", [](Animator& self) { return self.substep(); },
                       R"(Get the substep count.
Returns:
    int: Number of substeps per frame.)");
}
}  // namespace pyuipc::core