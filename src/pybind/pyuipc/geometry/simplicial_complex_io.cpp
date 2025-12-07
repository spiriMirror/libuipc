#include <pyuipc/geometry/simplicial_complex_io.h>
#include <uipc/io/simplicial_complex_io.h>
#include <uipc/io/spread_sheet_io.h>
#include <pyuipc/as_numpy.h>
#include <Eigen/Geometry>

namespace pyuipc::geometry
{
using namespace uipc::geometry;
PySimplicialComplexIO::PySimplicialComplexIO(py::module& m)
{
    auto class_SimplicialComplexIO =
        py::class_<SimplicialComplexIO>(m, "SimplicialComplexIO",
                                        R"(SimplicialComplexIO class for reading and writing simplicial complexes to/from files.)");

    class_SimplicialComplexIO.def(py::init<>(),
                                 R"(Create a SimplicialComplexIO instance without pre-transform.)");

    class_SimplicialComplexIO.def(
        py::init<>([](const Transform& pre_transform)
                   { return SimplicialComplexIO(pre_transform); }),
        py::arg("pre_transform"),
        R"(Create a SimplicialComplexIO instance with a pre-transform.
Args:
    pre_transform: Transform to apply before reading/writing.)");

    class_SimplicialComplexIO.def(py::init<>(
        [](py::array_t<Float> pre_transform)
        {
            auto mat = to_matrix<Matrix4x4>(pre_transform);
            return SimplicialComplexIO(mat);
        }),
        py::arg("pre_transform"),
        R"(Create a SimplicialComplexIO instance with a pre-transform matrix.
Args:
    pre_transform: 4x4 transformation matrix.)");

    class_SimplicialComplexIO.def("read", &SimplicialComplexIO::read,
                                  py::arg("filename"),
                                  R"(Read a simplicial complex from a file.
Args:
    filename: Input file path.
Returns:
    SimplicialComplex: Loaded simplicial complex.)");

    class_SimplicialComplexIO.def("write", &SimplicialComplexIO::write,
                                  py::arg("sc"),
                                  py::arg("filename"),
                                  R"(Write a simplicial complex to a file.
Args:
    sc: SimplicialComplex to write.
    filename: Output file path.)");
}
}  // namespace pyuipc::geometry
