#include <pyuipc/diff_sim/sparse_coo_view.h>
#include <uipc/diff_sim/sparse_coo_view.h>
#include <pybind11/eigen.h>
#include <pyuipc/as_numpy.h>

namespace pyuipc::diff_sim
{
using namespace uipc::diff_sim;
PySparseCOOView::PySparseCOOView(py::module& m)
{
    auto class_SparseCOOView = py::class_<SparseCOOView>(m, "SparseCOOView");

    class_SparseCOOView.def("shape", &SparseCOOView::shape)
        .def("row_indices",
             [](const SparseCOOView& self)
             { return as_numpy(self.row_indices(), py::cast(self)); });

    class_SparseCOOView.def("col_indices",
                            [](const SparseCOOView& self) {
                                return as_numpy(self.col_indices(), py::cast(self));
                            });

    class_SparseCOOView.def("values",
                            [](const SparseCOOView& self)
                            { return as_numpy(self.values(), py::cast(self)); });

    class_SparseCOOView.def("to_dense", &SparseCOOView::to_dense);

    class_SparseCOOView.def("to_sparse", &SparseCOOView::to_sparse);
}
}  // namespace pyuipc::diff_sim
