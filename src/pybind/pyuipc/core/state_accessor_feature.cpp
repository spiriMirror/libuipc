#include <pyuipc/core/state_accessor_feature.h>
#include <uipc/core/finite_element_state_accessor_feature.h>
#include <uipc/core/affine_body_state_accessor_feature.h>

namespace pyuipc::core
{
using namespace uipc::core;

PyStateAccessorFeature::PyStateAccessorFeature(py::module& m)
{
    auto class_FiniteElementStateAccessorFeature =
        py::class_<FiniteElementStateAccessorFeature, IFeature, S<FiniteElementStateAccessorFeature>>(
            m,
            "FiniteElementStateAccessorFeature",
            R"(Feature for accessing finite element simulation state (vertex positions, velocities, etc.).)");

    class_FiniteElementStateAccessorFeature.def("vertex_count",
                                                &FiniteElementStateAccessorFeature::vertex_count,
                                                R"(Get the number of vertices.
Returns:
    int: Number of vertices.)");

    class_FiniteElementStateAccessorFeature.def("create_geometry",
                                                &FiniteElementStateAccessorFeature::create_geometry,
                                                py::arg("vertex_offset") = 0,
                                                py::arg("vertex_count") = ~0ull,
                                                R"(Create geometry from state data.
Args:
    vertex_offset: Starting vertex index (default: 0).
    vertex_count: Number of vertices to include (default: all).
Returns:
    Geometry: Geometry created from state data.)");

    class_FiniteElementStateAccessorFeature.def("copy_from",
                                                &FiniteElementStateAccessorFeature::copy_from,
                                                py::arg("state_geo"),
                                                R"(Copy state from geometry.
Args:
    state_geo: Geometry containing state data to copy from.)");

    class_FiniteElementStateAccessorFeature.def("copy_to",
                                                &FiniteElementStateAccessorFeature::copy_to,
                                                py::arg("state_geo"),
                                                R"(Copy state to geometry.
Args:
    state_geo: Geometry to copy state data to.)");

    class_FiniteElementStateAccessorFeature.def("copy_position_to",
                                                &FiniteElementStateAccessorFeature::copy_position_to,
                                                py::arg("buffer_view"),
                                                py::arg("vertex_offset") = 0,
                                                py::arg("vertex_count")  = ~0ull,
                                                R"(Copy position data (Vector3) for the specified vertex range into an externally-owned buffer.
Args:
    buffer_view: Destination buffer view to copy position data into.
    vertex_offset: Starting vertex index (default: 0).
    vertex_count: Number of vertices to include (default: all).)");

    class_FiniteElementStateAccessorFeature.def("copy_velocity_to",
                                                &FiniteElementStateAccessorFeature::copy_velocity_to,
                                                py::arg("buffer_view"),
                                                py::arg("vertex_offset") = 0,
                                                py::arg("vertex_count")  = ~0ull,
                                                R"(Copy velocity data (Vector3) for the specified vertex range into an externally-owned buffer.
Args:
    buffer_view: Destination buffer view to copy velocity data into.
    vertex_offset: Starting vertex index (default: 0).
    vertex_count: Number of vertices to include (default: all).)");


    class_FiniteElementStateAccessorFeature.attr("FeatureName") =
        FiniteElementStateAccessorFeature::FeatureName;


    auto class_AffineBodyStateAccessorFeature =
        py::class_<AffineBodyStateAccessorFeature, IFeature, S<AffineBodyStateAccessorFeature>>(
            m,
            "AffineBodyStateAccessorFeature",
            R"(Feature for accessing affine body simulation state (body transforms, velocities, etc.).)");

    class_AffineBodyStateAccessorFeature.def("body_count",
                                             &AffineBodyStateAccessorFeature::body_count,
                                             R"(Get the number of affine bodies.
Returns:
    int: Number of affine bodies.)");

    class_AffineBodyStateAccessorFeature.def("create_geometry",
                                             &AffineBodyStateAccessorFeature::create_geometry,
                                             py::arg("body_offset") = 0,
                                             py::arg("body_count")  = ~0ull,
                                             R"(Create geometry from state data.
Args:
    body_offset: Starting body index (default: 0).
    body_count: Number of bodies to include (default: all).
Returns:
    Geometry: Geometry created from state data.)");

    class_AffineBodyStateAccessorFeature.def("copy_from",
                                             &AffineBodyStateAccessorFeature::copy_from,
                                             py::arg("state_geo"),
                                             R"(Copy state from geometry.
Args:
    state_geo: Geometry containing state data to copy from.)");

    class_AffineBodyStateAccessorFeature.def("copy_to",
                                             &AffineBodyStateAccessorFeature::copy_to,
                                             py::arg("state_geo"),
                                             R"(Copy state to geometry.
Args:
    state_geo: Geometry to copy state data to.)");

    class_AffineBodyStateAccessorFeature.def("copy_transform_to",
                                             &AffineBodyStateAccessorFeature::copy_transform_to,
                                             py::arg("buffer_view"),
                                             py::arg("body_offset") = 0,
                                             py::arg("body_count")  = ~0ull,
                                             R"(Copy transform data (Matrix4x4) for the specified body range into an externally-owned buffer.
Args:
    buffer_view: Destination buffer view to copy transform data into.
    body_offset: Starting body index (default: 0).
    body_count: Number of bodies to include (default: all).)");

    class_AffineBodyStateAccessorFeature.def("copy_velocity_to",
                                             &AffineBodyStateAccessorFeature::copy_velocity_to,
                                             py::arg("buffer_view"),
                                             py::arg("body_offset") = 0,
                                             py::arg("body_count")  = ~0ull,
                                             R"(Copy velocity data (Matrix4x4) for the specified body range into an externally-owned buffer.
Args:
    buffer_view: Destination buffer view to copy velocity data into.
    body_offset: Starting body index (default: 0).
    body_count: Number of bodies to include (default: all).)");


    class_AffineBodyStateAccessorFeature.attr("FeatureName") =
        AffineBodyStateAccessorFeature::FeatureName;
}
}  // namespace pyuipc::core
