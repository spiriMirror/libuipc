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

    auto class_Quaternion = py::class_<Quaternion>(m, "Quaternion",
                                                     R"(Quaternion class for representing 3D rotations.)");
    auto class_AngleAxis  = py::class_<AngleAxis>(m, "AngleAxis",
                                                   R"(AngleAxis class for representing rotations as an angle-axis pair.)");

    class_Quaternion.def_static("Identity",
                                []() { return Quaternion::Identity(); },
                                R"(Create an identity quaternion (no rotation).
Returns:
    Quaternion: Identity quaternion.)");

    class_Quaternion.def(py::init<>(
                             [](py::array_t<Float> wxyz) -> Quaternion
                             {
                                 Vector4 v4 = to_matrix<Vector4>(wxyz);
                                 return Quaternion(v4[0], v4[1], v4[2], v4[3]);
                             }),
                         py::arg("wxyz"),
                         R"(Create a quaternion from w, x, y, z components.
Args:
    wxyz: Array of 4 floats [w, x, y, z] representing the quaternion.)");

    class_Quaternion.def("__mul__",
                         [](Quaternion& self, const Quaternion& other) -> Quaternion
                         { return self * other; },
                         py::arg("other"),
                         R"(Multiply two quaternions (composition of rotations).
Args:
    other: Another quaternion.
Returns:
    Quaternion: Result of quaternion multiplication.)");

    class_Quaternion.def("inverse", &Quaternion::inverse,
                         R"(Get the inverse quaternion.
Returns:
    Quaternion: Inverse quaternion.)");

    class_Quaternion.def("conjugate", &Quaternion::conjugate,
                         R"(Get the conjugate quaternion.
Returns:
    Quaternion: Conjugate quaternion.)");

    class_Quaternion.def("norm", &Quaternion::norm,
                         R"(Get the norm (magnitude) of the quaternion.
Returns:
    float: Norm of the quaternion.)");

    class_Quaternion.def("normalized", &Quaternion::normalized,
                         R"(Get a normalized copy of the quaternion.
Returns:
    Quaternion: Normalized quaternion.)");

    class_Quaternion.def(py::init<>([](const AngleAxis& ax) -> Quaternion
                                    { return Quaternion(ax); }),
                         py::arg("angle_axis"),
                         R"(Create a quaternion from an AngleAxis.
Args:
    angle_axis: AngleAxis object to convert.)");


    class_AngleAxis.def_static("Identity", []() { return AngleAxis::Identity(); },
                               R"(Create an identity angle-axis (no rotation).
Returns:
    AngleAxis: Identity angle-axis.)");

    class_AngleAxis.def(py::init<>(
                            [](Float angle, py::array_t<Float> axis) -> AngleAxis
                            {
                                Vector3 A = to_matrix<Vector3>(axis);
                                return AngleAxis(angle, A.normalized());
                            }),
                        py::arg("angle"),
                        py::arg("axis"),
                        R"(Create an angle-axis from an angle and axis vector.
Args:
    angle: Rotation angle in radians.
    axis: 3D axis vector (will be normalized).)");

    class_AngleAxis.def(py::init<>([](const Quaternion& q) -> AngleAxis
                                   { return AngleAxis(q); }),
                        py::arg("quaternion"),
                        R"(Create an angle-axis from a quaternion.
Args:
    quaternion: Quaternion to convert.)");

    class_AngleAxis.def("angle", [](AngleAxis& self) { return self.angle(); },
                        R"(Get the rotation angle.
Returns:
    float: Rotation angle in radians.)");


    class_AngleAxis.def("axis",
                        [](AngleAxis& self) { return as_numpy(self.axis()); },
                        R"(Get the rotation axis.
Returns:
    numpy.ndarray: 3D axis vector.)");

    class_AngleAxis.def("__mul__",
                        [](AngleAxis& self, const AngleAxis& other) -> Quaternion
                        { return self * other; },
                        py::arg("other"),
                        R"(Multiply two angle-axis rotations (returns quaternion).
Args:
    other: Another angle-axis.
Returns:
    Quaternion: Result of rotation composition.)");

    class_Quaternion.def("__mul__",
                         [](Quaternion& self, const AngleAxis& other) -> Quaternion
                         { return self * other; },
                         py::arg("angle_axis"),
                         R"(Multiply quaternion by angle-axis.
Args:
    angle_axis: AngleAxis to multiply with.
Returns:
    Quaternion: Result of multiplication.)");

    class_AngleAxis.def("__mul__",
                        [](AngleAxis& self, const Quaternion& other) -> Quaternion
                        { return self * other; },
                        py::arg("quaternion"),
                        R"(Multiply angle-axis by quaternion.
Args:
    quaternion: Quaternion to multiply with.
Returns:
    Quaternion: Result of multiplication.)");


    auto class_Transform = py::class_<Transform>(m, "Transform",
                                                  R"(Transform class representing a 4x4 homogeneous transformation matrix.)");

    class_Transform.def_static("Identity", []() { return Transform::Identity(); },
                               R"(Create an identity transform (no translation, no rotation, unit scale).
Returns:
    Transform: Identity transformation matrix.)");

    class_Transform.def("matrix",
                        [](Transform& self) { return as_numpy(self.matrix()); },
                        R"(Get the 4x4 transformation matrix.
Returns:
    numpy.ndarray: 4x4 transformation matrix.)");

    // transform
    class_Transform.def(py::init<>(
        [](py::array_t<Float> m) -> Transform
        {
            Transform t = Transform::Identity();
            t.matrix()  = to_matrix<Matrix4x4>(m);
            return t;
        }),
        py::arg("matrix"),
        R"(Create a transform from a 4x4 matrix.
Args:
    matrix: 4x4 transformation matrix.)");

    class_Transform.def("translate",
                        [](Transform& self, py::array_t<Float> v) -> Transform&
                        {
                            Vector3 v3 = to_matrix<Vector3>(v);
                            return self.translate(v3);
                        },
                        py::arg("translation"),
                        R"(Apply translation to the transform (post-multiply).
Args:
    translation: 3D translation vector.
Returns:
    Transform: Reference to self for chaining.)");

    class_Transform.def("rotate",
                        [](Transform& self, const AngleAxis& aa) -> Transform&
                        { return self.rotate(aa); },
                        py::arg("angle_axis"),
                        R"(Apply rotation to the transform (post-multiply).
Args:
    angle_axis: AngleAxis rotation.
Returns:
    Transform: Reference to self for chaining.)");

    class_Transform.def("rotate",
                        [](Transform& self, const Quaternion& Q) -> Transform&
                        { return self.rotate(Q); },
                        py::arg("quaternion"),
                        R"(Apply rotation to the transform (post-multiply).
Args:
    quaternion: Quaternion rotation.
Returns:
    Transform: Reference to self for chaining.)");

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
                        },
                        py::arg("scale"),
                        R"(Apply scaling to the transform (post-multiply).
Args:
    scale: Either a scalar (uniform scale) or 3D vector (non-uniform scale).
Returns:
    Transform: Reference to self for chaining.)");

    class_Transform.def("translation",
                        [](Transform& self)
                        { return as_numpy(self.translation().eval()); },
                        R"(Get the translation component of the transform.
Returns:
    numpy.ndarray: 3D translation vector.)");

    // pretransform

    class_Transform.def("pretranslate",
                        [](Transform& self, py::array_t<Float> v) -> Transform&
                        {
                            Vector3 v3 = to_matrix<Vector3>(v);
                            return self.pretranslate(v3);
                        },
                        py::arg("translation"),
                        R"(Apply translation to the transform (pre-multiply).
Args:
    translation: 3D translation vector.
Returns:
    Transform: Reference to self for chaining.)");

    class_Transform.def("prerotate",
                        [](Transform& self, const AngleAxis& aa) -> Transform&
                        { return self.prerotate(aa); },
                        py::arg("angle_axis"),
                        R"(Apply rotation to the transform (pre-multiply).
Args:
    angle_axis: AngleAxis rotation.
Returns:
    Transform: Reference to self for chaining.)");

    class_Transform.def("prerotate",
                        [](Transform& self, const Quaternion& Q) -> Transform&
                        { return self.prerotate(Q); },
                        py::arg("quaternion"),
                        R"(Apply rotation to the transform (pre-multiply).
Args:
    quaternion: Quaternion rotation.
Returns:
    Transform: Reference to self for chaining.)");

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
                        },
                        py::arg("scale"),
                        R"(Apply scaling to the transform (pre-multiply).
Args:
    scale: Either a scalar (uniform scale) or 3D vector (non-uniform scale).
Returns:
    Transform: Reference to self for chaining.)");

    // __mul__
    class_Transform.def("__mul__",
                        [](Transform& self, const Transform& other) -> Transform
                        { return self * other; },
                        py::arg("other"),
                        R"(Compose two transforms (matrix multiplication).
Args:
    other: Another transform.
Returns:
    Transform: Result of transform composition.)");
    class_Transform.def("__mul__",
                        [](Transform& self, py::array_t<Float> v) -> py::array_t<Float>
                        {
                            Vector3 v3 = to_matrix<Vector3>(v);
                            return as_numpy(self * v3);
                        },
                        py::arg("vector"),
                        R"(Apply transform to a 3D vector.
Args:
    vector: 3D vector to transform.
Returns:
    numpy.ndarray: Transformed 3D vector.)");

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
                        },
                        py::arg("vectors"),
                        R"(Apply transform to a vector or array of vectors (in-place for arrays).
Args:
    vectors: Single 3D vector or array of 3D vectors.
Returns:
    numpy.ndarray: Transformed vector(s).)");

    class_Transform.def("inverse", &Transform::inverse,
                        R"(Get the inverse transform.
Returns:
    Transform: Inverse transformation matrix.)");
}
}  // namespace pyuipc
