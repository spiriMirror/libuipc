#include <pyuipc/diff_sim/sparse_coo_view.h>
#include <uipc/diff_sim/sparse_coo_view.h>
#include <pybind11/eigen.h>
#include <pyuipc/as_numpy.h>

namespace pyuipc::diff_sim
{
using namespace uipc::diff_sim;
PySparseCOOView::PySparseCOOView(py::module& m)
{
    auto class_SparseCOOView = py::class_<SparseCOOView>(m, "SparseCOOView",
                                                          R"(SparseCOOView class for viewing sparse matrices in COO (Coordinate) format.)");

    class_SparseCOOView.def("shape", &SparseCOOView::shape,
                           R"(Get the matrix shape.
Returns:
    tuple: (rows, cols) shape of the matrix.)")
        .def("row_indices",
             [](const SparseCOOView& self)
             { return as_numpy(self.row_indices(), py::cast(self)); },
             R"(Get the row indices.
Returns:
    numpy.ndarray: Array of row indices.)");

    class_SparseCOOView.def("col_indices",
                            [](const SparseCOOView& self) {
                                return as_numpy(self.col_indices(), py::cast(self));
                            },
                            R"(Get the column indices.
Returns:
    numpy.ndarray: Array of column indices.)");

    class_SparseCOOView.def("values",
                            [](const SparseCOOView& self)
                            { return as_numpy(self.values(), py::cast(self)); },
                            R"(Get the values.
Returns:
    numpy.ndarray: Array of matrix values.)");

    class_SparseCOOView.def("to_dense", &SparseCOOView::to_dense,
                           R"(Convert to dense matrix representation.
Returns:
    numpy.ndarray: Dense matrix.)");

    class_SparseCOOView.def("to_sparse", &SparseCOOView::to_sparse,
                           R"(Convert to sparse matrix representation (scipy.sparse).
Returns:
    scipy.sparse matrix: Sparse matrix object.)");
}
}  // namespace pyuipc::diff_sim
