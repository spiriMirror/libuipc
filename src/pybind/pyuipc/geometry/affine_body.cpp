#include <pyuipc/geometry/affine_body.h>
#include <uipc/geometry/utils/affine_body/transform.h>
#include <uipc/geometry/utils/affine_body/compute_body_force.h>
#include <uipc/geometry/utils/affine_body/compute_dyadic_mass.h>
#include <uipc/geometry/utils/affine_body/affine_body_from_rigid_body.h>
#include <cstring>

namespace pyuipc::geometry
{
using namespace uipc::geometry;
using namespace uipc::geometry::affine_body;

namespace
{
    template <auto Convert>
    py::array_t<Float> batch_to_transforms(const py::array_t<Float>& input)
    {
        const auto         count = input.shape(0);
        py::array_t<Float> result({count, py::ssize_t{4}, py::ssize_t{4}});
        auto               dst  = result.mutable_unchecked<3>();
        const auto*        data = reinterpret_cast<const char*>(input.data());
        const auto         batch_stride     = input.strides(0);
        const auto         component_stride = input.strides(1);
        for(py::ssize_t i = 0; i < count; ++i)
        {
            const auto* src = data + i * batch_stride;
            Vector12    q;
            for(int j = 0; j < 12; ++j)
                std::memcpy(&q[j], src + j * component_stride, sizeof(Float));
            const Matrix4x4 transform = Convert(q);
            for(int row = 0; row < 4; ++row)
                for(int col = 0; col < 4; ++col)
                    dst(i, row, col) = transform(row, col);
        }
        return result;
    }

    template <auto Convert>
    py::array_t<Float> batch_to_vectors(const py::array_t<Float>& input)
    {
        const auto         count = input.shape(0);
        py::array_t<Float> result({count, py::ssize_t{12}, py::ssize_t{1}});
        auto               dst  = result.mutable_unchecked<3>();
        const auto*        data = reinterpret_cast<const char*>(input.data());
        const auto         batch_stride = input.strides(0);
        const auto         row_stride   = input.strides(1);
        const auto         col_stride   = input.strides(2);
        for(py::ssize_t i = 0; i < count; ++i)
        {
            const auto* src = data + i * batch_stride;
            Matrix4x4   transform;
            for(int row = 0; row < 4; ++row)
                for(int col = 0; col < 4; ++col)
                    std::memcpy(&transform(row, col),
                                src + row * row_stride + col * col_stride,
                                sizeof(Float));
            const Vector12 q = Convert(transform);
            for(int j = 0; j < 12; ++j)
                dst(i, j, 0) = q[j];
        }
        return result;
    }
}  // namespace

PyAffineBody::PyAffineBody(py::module& m)
{
    // Transform functions (single and batched)
    m.def(
        "q_to_transform",
        [](py::array_t<Float> q) -> py::array_t<Float>
        {
            // Check if batched (2D array with shape (N, 12))
            if(q.ndim() == 2)
            {
                if(q.shape(1) != 12)
                {
                    throw PyException(PYUIPC_MSG("q_to_transform: batched input must have shape (N, 12), got ({}, {})",
                                                 q.shape(0),
                                                 q.shape(1)));
                }
                return batch_to_transforms<q_to_transform>(q);
            }
            // Single item (1D array with shape (12,))
            if(q.ndim() == 1)
            {
                if(q.shape(0) != 12)
                {
                    throw PyException(PYUIPC_MSG("q_to_transform: single input must have shape (12,), got ({})",
                                                 q.shape(0)));
                }
                auto q_ = to_matrix<Vector12>(q);
                return as_numpy(q_to_transform(q_));
            }
            throw PyException(PYUIPC_MSG("q_to_transform: input must be 1D (12,) or 2D (N, 12), got {}D",
                                         q.ndim()));
        },
        py::arg("q"),
        R"(Convert a 12D configuration vector to a 4x4 transformation matrix.
Supports both single and batched operations.
Args:
    q: 12D vector [translation(3), rotation_row0(3), rotation_row1(3), rotation_row2(3)] 
       or array of shape (N, 12) for batched processing.
Returns:
    numpy.ndarray: 4x4 transformation matrix or array of shape (N, 4, 4) for batched input.)");

    m.def(
        "transform_to_q",
        [](py::array_t<Float> trans) -> py::array_t<Float>
        {
            // Check if batched (3D array with shape (N, 4, 4))
            if(trans.ndim() == 3)
            {
                if(trans.shape(1) != 4 || trans.shape(2) != 4)
                {
                    throw PyException(PYUIPC_MSG("transform_to_q: batched input must have shape (N, 4, 4), got ({}, {}, {})",
                                                 trans.shape(0),
                                                 trans.shape(1),
                                                 trans.shape(2)));
                }
                return batch_to_vectors<transform_to_q>(trans);
            }
            // Single item (2D array with shape (4, 4))
            if(trans.ndim() == 2)
            {
                if(trans.shape(0) != 4 || trans.shape(1) != 4)
                {
                    throw PyException(PYUIPC_MSG("transform_to_q: single input must have shape (4, 4), got ({}, {})",
                                                 trans.shape(0),
                                                 trans.shape(1)));
                }
                auto trans_ = to_matrix<Matrix4x4>(trans);
                return as_numpy(transform_to_q(trans_));
            }
            throw PyException(PYUIPC_MSG("transform_to_q: input must be 2D (4, 4) or 3D (N, 4, 4), got {}D",
                                         trans.ndim()));
        },
        py::arg("transform"),
        R"(Convert a 4x4 transformation matrix to a 12D configuration vector.
Supports both single and batched operations.
Args:
    transform: 4x4 transformation matrix or array of shape (N, 4, 4) for batched processing.
Returns:
    numpy.ndarray: 12D vector [translation(3), rotation_row0(3), rotation_row1(3), rotation_row2(3)]
                  or array of shape (N, 12, 1) for batched input.)");

    m.def(
        "q_v_to_transform_v",
        [](py::array_t<Float> q) -> py::array_t<Float>
        {
            // Check if batched (2D array with shape (N, 12))
            if(q.ndim() == 2)
            {
                if(q.shape(1) != 12)
                {
                    throw PyException(PYUIPC_MSG("q_v_to_transform_v: batched input must have shape (N, 12), got ({}, {})",
                                                 q.shape(0),
                                                 q.shape(1)));
                }
                return batch_to_transforms<q_v_to_transform_v>(q);
            }
            // Single item (1D array with shape (12,))
            if(q.ndim() == 1)
            {
                if(q.shape(0) != 12)
                {
                    throw PyException(PYUIPC_MSG("q_v_to_transform_v: single input must have shape (12,), got ({})",
                                                 q.shape(0)));
                }
                auto q_ = to_matrix<Vector12>(q);
                return as_numpy(q_v_to_transform_v(q_));
            }
            throw PyException(PYUIPC_MSG("q_v_to_transform_v: input must be 1D (12,) or 2D (N, 12), got {}D",
                                         q.ndim()));
        },
        py::arg("q"),
        R"(Convert a 12D velocity vector to a 4x4 velocity transformation matrix.
Supports both single and batched operations.
Args:
    q: 12D velocity vector [translation(3), rotation_row0(3), rotation_row1(3), rotation_row2(3)]
       or array of shape (N, 12) for batched processing.
Returns:
    numpy.ndarray: 4x4 velocity transformation matrix (last row is zero) 
                  or array of shape (N, 4, 4) for batched input.)");

    m.def(
        "transform_v_to_q_v",
        [](py::array_t<Float> transform_v) -> py::array_t<Float>
        {
            // Check if batched (3D array with shape (N, 4, 4))
            if(transform_v.ndim() == 3)
            {
                if(transform_v.shape(1) != 4 || transform_v.shape(2) != 4)
                {
                    throw PyException(PYUIPC_MSG("transform_v_to_q_v: batched input must have shape (N, 4, 4), got ({}, {}, {})",
                                                 transform_v.shape(0),
                                                 transform_v.shape(1),
                                                 transform_v.shape(2)));
                }
                return batch_to_vectors<transform_v_to_q_v>(transform_v);
            }
            // Single item (2D array with shape (4, 4))
            if(transform_v.ndim() == 2)
            {
                if(transform_v.shape(0) != 4 || transform_v.shape(1) != 4)
                {
                    throw PyException(PYUIPC_MSG("transform_v_to_q_v: single input must have shape (4, 4), got ({}, {})",
                                                 transform_v.shape(0),
                                                 transform_v.shape(1)));
                }
                auto transform_v_ = to_matrix<Matrix4x4>(transform_v);
                return as_numpy(transform_v_to_q_v(transform_v_));
            }
            throw PyException(PYUIPC_MSG("transform_v_to_q_v: input must be 2D (4, 4) or 3D (N, 4, 4), got {}D",
                                         transform_v.ndim()));
        },
        py::arg("transform_v"),
        R"(Convert a 4x4 velocity transformation matrix to a 12D velocity vector.
Supports both single and batched operations.
Args:
    transform_v: 4x4 velocity transformation matrix or array of shape (N, 4, 4) for batched processing.
Returns:
    numpy.ndarray: 12D velocity vector [translation(3), rotation_row0(3), rotation_row1(3), rotation_row2(3)]
                  or array of shape (N, 12, 1) for batched input.)");

    // Compute body force
    m.def(
        "compute_body_force",
        [](const SimplicialComplex& sc,
           py::array_t<Float>       body_force_density) -> py::array_t<Float>
        {
            auto body_force_density_ = to_matrix<Vector3>(body_force_density);
            return as_numpy(compute_body_force(sc, body_force_density_));
        },
        py::arg("sc"),
        py::arg("body_force_density"),
        R"(Compute the body force of an affine body.
Args:
    sc: The simplicial complex.
    body_force_density: The body force density in N/m^3 (3D vector).
Returns:
    numpy.ndarray: 12D body force vector.)");

    // Compute dyadic mass
    m.def(
        "compute_dyadic_mass",
        [](const SimplicialComplex& sc, Float rho) -> py::tuple
        {
            Float     m;
            Vector3   m_x_bar;
            Matrix3x3 m_x_bar_x_bar;
            compute_dyadic_mass(sc, rho, m, m_x_bar, m_x_bar_x_bar);
            return py::make_tuple(m, as_numpy(m_x_bar), as_numpy(m_x_bar_x_bar));
        },
        py::arg("sc"),
        py::arg("rho"),
        R"(Compute the dyadic mass of a simplicial complex.
Integrate the mass density over the simplicial complex to compute the dyadic mass.
Args:
    sc: The simplicial complex.
    rho: Mass density.
Returns:
    tuple: (m, m_x_bar, m_x_bar_x_bar) where:
        - m: The total mass (float).
        - m_x_bar: The total mass times the center of mass (3D vector).
        - m_x_bar_x_bar: The total mass times the center of mass times the center of mass transpose (3x3 matrix).)");

    m.def(
        "from_rigid_body",
        [](Float              mass,
           py::array_t<Float> center_of_mass,
           py::array_t<Float> inertia_cm) -> py::array_t<Float>
        {
            auto c = to_matrix<Vector3>(center_of_mass);
            auto I = to_matrix<Matrix3x3>(inertia_cm);
            return as_numpy(from_rigid_body(mass, c, I));
        },
        py::arg("mass"),
        py::arg("center_of_mass"),
        py::arg("inertia_cm"),
        R"(Build the 12x12 ABD mass matrix from rigid body quantities.
Args:
    mass: Total mass of the rigid body.
    center_of_mass: Center of mass in the reference configuration (3D vector).
    inertia_cm: 3x3 inertia tensor about the center of mass.
Returns:
    numpy.ndarray: 12x12 ABD mass matrix.)");

    m.def(
        "build_abd_mass_matrix",
        [](Float m, py::array_t<Float> m_x_bar, py::array_t<Float> m_x_bar_x_bar) -> py::array_t<Float>
        {
            auto mx  = to_matrix<Vector3>(m_x_bar);
            auto mxx = to_matrix<Matrix3x3>(m_x_bar_x_bar);
            return as_numpy(build_abd_mass_matrix(m, mx, mxx));
        },
        py::arg("m"),
        py::arg("m_x_bar"),
        py::arg("m_x_bar_x_bar"),
        R"(Build the 12x12 ABD mass matrix from dyadic mass components.
Args:
    m: Total mass.
    m_x_bar: First moment of mass (m * center_of_mass), 3D vector.
    m_x_bar_x_bar: Second moment of mass tensor S, 3x3 matrix.
Returns:
    numpy.ndarray: 12x12 ABD mass matrix.)");

    m.def(
        "to_rigid_body",
        [](Float m, py::array_t<Float> m_x_bar, py::array_t<Float> m_x_bar_x_bar) -> py::tuple
        {
            auto      mx  = to_matrix<Vector3>(m_x_bar);
            auto      mxx = to_matrix<Matrix3x3>(m_x_bar_x_bar);
            Float     total_mass;
            Vector3   center_of_mass;
            Matrix3x3 inertia_cm;
            to_rigid_body(m, mx, mxx, total_mass, center_of_mass, inertia_cm);
            return py::make_tuple(total_mass, as_numpy(center_of_mass), as_numpy(inertia_cm));
        },
        py::arg("m"),
        py::arg("m_x_bar"),
        py::arg("m_x_bar_x_bar"),
        R"(Extract rigid body properties from dyadic mass components.
Args:
    m: Total mass.
    m_x_bar: First moment of mass (m * center_of_mass), 3D vector.
    m_x_bar_x_bar: Second moment of mass tensor S, 3x3 matrix.
Returns:
    tuple: (total_mass, center_of_mass, inertia_cm) where:
        - total_mass: Total mass (float).
        - center_of_mass: Center of mass in the reference frame (3D vector).
        - inertia_cm: 3x3 inertia tensor about the center of mass.)");

    m.def(
        "to_rigid_body",
        [](py::array_t<Float> mass_matrix) -> py::tuple
        {
            auto      M = to_matrix<Matrix12x12>(mass_matrix);
            Float     total_mass;
            Vector3   center_of_mass;
            Matrix3x3 inertia_cm;
            to_rigid_body(M, total_mass, center_of_mass, inertia_cm);
            return py::make_tuple(total_mass, as_numpy(center_of_mass), as_numpy(inertia_cm));
        },
        py::arg("mass_matrix"),
        R"(Extract rigid body properties from a 12x12 ABD mass matrix.
Args:
    mass_matrix: 12x12 ABD mass matrix.
Returns:
    tuple: (total_mass, center_of_mass, inertia_cm) where:
        - total_mass: Total mass (float).
        - center_of_mass: Center of mass in the reference frame (3D vector).
        - inertia_cm: 3x3 inertia tensor about the center of mass.)");
}
}  // namespace pyuipc::geometry
