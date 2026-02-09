#include <pyuipc/geometry/affine_body.h>
#include <uipc/geometry/utils/affine_body/transform.h>
#include <uipc/geometry/utils/affine_body/compute_body_force.h>
#include <uipc/geometry/utils/affine_body/compute_dyadic_mass.h>

namespace pyuipc::geometry
{
using namespace uipc::geometry;
using namespace uipc::geometry::affine_body;

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
                auto                     q_span = as_span_of<const Vector12>(q);
                std::vector<py::ssize_t> shape  = {q.shape(0), 4, 4};
                py::array_t<Float>       result(shape);
                auto result_span = as_span_of<Matrix4x4>(result);
                std::ranges::transform(q_span, result_span.begin(), q_to_transform);
                return result;
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
                auto trans_span = as_span_of<const Matrix4x4>(trans);
                std::vector<py::ssize_t> shape = {trans.shape(0), 12, 1};
                py::array_t<Float>       result(shape);
                auto result_3d = result.mutable_unchecked<3>();
                auto indices = std::ranges::views::iota(size_t{0}, trans_span.size());
                std::ranges::for_each(
                    indices,
                    [&](size_t i)
                    {
                        Vector12 q = transform_to_q(trans_span[i]);
                        auto     j_indices =
                            std::ranges::views::iota(py::ssize_t{0}, py::ssize_t{12});
                        std::ranges::for_each(j_indices,
                                              [&](py::ssize_t j)
                                              { result_3d(i, j, 0) = q(j); });
                    });
                return result;
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
                auto                     q_span = as_span_of<const Vector12>(q);
                std::vector<py::ssize_t> shape  = {q.shape(0), 4, 4};
                py::array_t<Float>       result(shape);
                auto result_span = as_span_of<Matrix4x4>(result);
                std::ranges::transform(q_span, result_span.begin(), q_v_to_transform_v);
                return result;
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
                auto transform_v_span = as_span_of<const Matrix4x4>(transform_v);
                std::vector<py::ssize_t> shape = {transform_v.shape(0), 12, 1};
                py::array_t<Float>       result(shape);
                auto result_3d = result.mutable_unchecked<3>();
                auto indices =
                    std::ranges::views::iota(size_t{0}, transform_v_span.size());
                std::ranges::for_each(
                    indices,
                    [&](size_t i)
                    {
                        Vector12 q = transform_v_to_q_v(transform_v_span[i]);
                        auto     j_indices =
                            std::ranges::views::iota(py::ssize_t{0}, py::ssize_t{12});
                        std::ranges::for_each(j_indices,
                                              [&](py::ssize_t j)
                                              { result_3d(i, j, 0) = q(j); });
                    });
                return result;
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
}
}  // namespace pyuipc::geometry
