#include <pyuipc/common/transform.h>
#include <Eigen/Geometry>
#include <uipc/common/type_define.h>
#include <pyuipc/as_numpy.h>

namespace pyuipc
{
using namespace uipc;
PyTransform::PyTransform(py::module& m)
{
    using AngleAxis = Eigen::AngleAxis<Float>;

    auto class_AngleAxis = py::class_<AngleAxis>(m, "AngleAxis");

    class_AngleAxis.def_static("Identity", []() { return AngleAxis::Identity(); });

    class_AngleAxis.def(py::init<>(
        [](Float angle, py::array_t<Float> axis) -> AngleAxis
        {
            Vector3 A = to_matrix<Vector3>(axis);
            return AngleAxis(angle, A.normalized());
        }));

    class_AngleAxis.def("angle", [](AngleAxis& self) { return self.angle(); });

    class_AngleAxis.def("axis",
                        [](AngleAxis& self) { return as_numpy(self.axis()); });


    auto class_Transform = py::class_<Transform>(m, "Transform");

    class_Transform.def_static("Identity", []() { return Transform::Identity(); });

    class_Transform.def_static("Translation",
                               [](py::array_t<Float> v) -> Transform
                               {
                                   Vector3   v3 = to_matrix<Vector3>(v);
                                   Transform t  = Transform::Identity();
                                   t.translate(v3);
                                   return t;
                               });

    class_Transform.def_static("Rotation",
                               [](const AngleAxis& aa) -> Transform
                               {
                                   Transform t = Transform::Identity();
                                   t.rotate(aa);
                                   return t;
                               });

    class_Transform.def_static("Scaling",
                               [](py::array_t<Float> v) -> Transform
                               {
                                   if(v.ndim() == 0)
                                   {
                                       Float     s = v.cast<Float>();
                                       Transform t = Transform::Identity();
                                       t.scale(s);
                                       return t;
                                   }
                                   else
                                   {
                                       Vector3   v3 = to_matrix<Vector3>(v);
                                       Transform t  = Transform::Identity();
                                       t.scale(v3);
                                       return t;
                                   }
                               });

    class_Transform.def("matrix",
                        [](Transform& self) { return as_numpy(self.matrix()); });

    // transform

    class_Transform.def("translate",
                        [](Transform& self, py::array_t<Float> v) -> Transform&
                        {
                            Vector3 v3 = to_matrix<Vector3>(v);
                            return self.translate(v3);
                        });

    class_Transform.def("rotate",
                        [](Transform& self, const AngleAxis& aa) -> Transform&
                        { return self.rotate(aa); });

    class_Transform.def("scale",
                        [](Transform& self, py::array_t<Float> v) -> Transform&
                        {
                            if(v.ndim() == 0)
                            {
                                Float s = v.cast<Float>();
                                return self.scale(s);
                            }
                            else
                            {
                                Vector3 v3 = to_matrix<Vector3>(v);
                                return self.scale(v3);
                            }
                        });

    // pretransform

    class_Transform.def("pretranslate",
                        [](Transform& self, py::array_t<Float> v) -> Transform&
                        {
                            Vector3 v3 = to_matrix<Vector3>(v);
                            return self.pretranslate(v3);
                        });

    class_Transform.def("prerotate",
                        [](Transform& self, const AngleAxis& aa) -> Transform&
                        { return self.prerotate(aa); });

    class_Transform.def("prescale",
                        [](Transform& self, py::array_t<Float> v) -> Transform&
                        {
                            if(v.ndim() == 0)
                            {
                                Float s = v.cast<Float>();
                                return self.prescale(s);
                            }
                            else
                            {
                                Vector3 v3 = to_matrix<Vector3>(v);
                                return self.prescale(v3);
                            }
                        });

    // __mul__
    class_Transform.def("__mul__",
                        [](Transform& self, const Transform& other) -> Transform
                        { return self * other; });
    class_Transform.def("__mul__",
                        [](Transform& self, py::array_t<Float> v) -> py::array_t<Float>
                        {
                            Vector3 v3 = to_matrix<Vector3>(v);
                            return as_numpy(self * v3);
                        });

    class_Transform.def("apply_to",
                        [](Transform& self, py::array_t<Float> v) -> py::array_t<Float>
                        {
                            if(v.ndim() >= 2)
                            {
                                auto span_v = as_span_of<Vector3>(v);
                                for(auto& v3 : span_v)
                                    v3 = self * v3;
                                return v;
                            }

                            Vector3 v3 = to_matrix<Vector3>(v);
                            return as_numpy(self * v3);
                        });

    class_Transform.def("inverse", &Transform::inverse);
}
}  // namespace pyuipc
