#include <pyuipc/common/transform.h>
#include <Eigen/Geometry>
#include <uipc/common/type_define.h>
#include <pyuipc/as_numpy.h>

namespace pyuipc
{
using namespace uipc;
PyTransform::PyTransform(py::module& m)
{
    using Quaternion = Eigen::Quaternion<Float>;
    using AngleAxis  = Eigen::AngleAxis<Float>;

    auto class_Quaternion = py::class_<Quaternion>(m, "Quaternion");
    auto class_AngleAxis  = py::class_<AngleAxis>(m, "AngleAxis");

    class_Quaternion.def_static("Identity",
                                []() { return Quaternion::Identity(); });

    class_Quaternion.def(py::init<>(
                             [](py::array_t<Float> wxyz) -> Quaternion
                             {
                                 Vector4 v4 = to_matrix<Vector4>(wxyz);
                                 return Quaternion(v4[0], v4[1], v4[2], v4[3]);
                             }),
                         py::arg("wxyz"));

    class_Quaternion.def("__mul__",
                         [](Quaternion& self, const Quaternion& other) -> Quaternion
                         { return self * other; });

    class_Quaternion.def("inverse", &Quaternion::inverse);

    class_Quaternion.def("conjugate", &Quaternion::conjugate);

    class_Quaternion.def("norm", &Quaternion::norm);

    class_Quaternion.def("normalized", &Quaternion::normalized);

    class_Quaternion.def(py::init<>([](const AngleAxis& ax) -> Quaternion
                                    { return Quaternion(ax); }));


    class_AngleAxis.def_static("Identity", []() { return AngleAxis::Identity(); });

    class_AngleAxis.def(py::init<>(
                            [](Float angle, py::array_t<Float> axis) -> AngleAxis
                            {
                                Vector3 A = to_matrix<Vector3>(axis);
                                return AngleAxis(angle, A.normalized());
                            }),
                        py::arg("angle"),
                        py::arg("axis"));

    class_AngleAxis.def(py::init<>([](const Quaternion& q) -> AngleAxis
                                   { return AngleAxis(q); }));

    class_AngleAxis.def("angle", [](AngleAxis& self) { return self.angle(); });


    class_AngleAxis.def("axis",
                        [](AngleAxis& self) { return as_numpy(self.axis()); });

    class_AngleAxis.def("__mul__",
                        [](AngleAxis& self, const AngleAxis& other) -> Quaternion
                        { return self * other; });

    class_Quaternion.def("__mul__",
                         [](Quaternion& self, const AngleAxis& other) -> Quaternion
                         { return self * other; });

    class_AngleAxis.def("__mul__",
                        [](AngleAxis& self, const Quaternion& other) -> Quaternion
                        { return self * other; });


    auto class_Transform = py::class_<Transform>(m, "Transform");

    class_Transform.def_static("Identity", []() { return Transform::Identity(); });

    class_Transform.def("matrix",
                        [](Transform& self) { return as_numpy(self.matrix()); });

    // transform
    class_Transform.def(py::init<>(
        [](py::array_t<Float> m) -> Transform
        {
            Transform t = Transform::Identity();
            t.matrix()  = to_matrix<Matrix4x4>(m);
            return t;
        }));

    class_Transform.def("translate",
                        [](Transform& self, py::array_t<Float> v) -> Transform&
                        {
                            Vector3 v3 = to_matrix<Vector3>(v);
                            return self.translate(v3);
                        });

    class_Transform.def("rotate",
                        [](Transform& self, const AngleAxis& aa) -> Transform&
                        { return self.rotate(aa); });

    class_Transform.def("rotate",
                        [](Transform& self, const Quaternion& Q) -> Transform&
                        { return self.rotate(Q); });

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

    class_Transform.def("translation",
                        [](Transform& self)
                        { return as_numpy(self.translation().eval()); });

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

    class_Transform.def("prerotate",
                        [](Transform& self, const Quaternion& Q) -> Transform&
                        { return self.prerotate(Q); });

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
