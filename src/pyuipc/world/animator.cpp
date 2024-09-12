#include <pyuipc/world/animator.h>
#include <uipc/world/animator.h>
#include <pyuipc/as_numpy.h>

namespace pyuipc::world
{
using namespace uipc::world;
PyAnimator::PyAnimator(py::module& m)
{
    auto class_Animation = py::class_<Animation>(m, "Animation");
    auto class_UpdateHint = py::class_<Animation::UpdateHint>(class_Animation, "UpdateHint");

    auto class_UpdateInfo = py::class_<Animation::UpdateInfo>(class_Animation, "UpdateInfo");
    class_UpdateInfo.def("object", &Animation::UpdateInfo::object)
        .def("geo_slots",
             [](Animation::UpdateInfo& self) -> py::list
             {
                 py::list list;
                 for(auto slot : self.geo_slots())
                 {
                     list.append(py::cast(slot));
                 }
                 return list;
             })
        .def("rest_geo_slots",
             [](Animation::UpdateInfo& self) -> py::list
             {
                 py::list list;
                 for(auto slot : self.rest_geo_slots())
                 {
                     list.append(py::cast(slot));
                 }
                 return list;
             })
        .def("frame", &Animation::UpdateInfo::frame)
        .def("hint", &Animation::UpdateInfo::hint);

    auto class_Animator = py::class_<Animator>(m, "Animator");
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
                                           try
                                           {
                                               callable(py::cast(info));
                                           }
                                           catch(const std::exception& e)
                                           {
                                               spdlog::error("Python Animation Script Error in Object [{}]({}):\n{}",
                                                             info.object().name(),
                                                             info.object().id(),
                                                             e.what());
                                           }
                                       });
                       });
    class_Animator.def("erase", &Animator::erase);
}
}  // namespace pyuipc::world