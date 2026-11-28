#include <pyuipc/core/fem_system_feature.h>
#include <uipc/core/fem_system_feature.h>
#include <pybind11/stl.h>

namespace pyuipc::core
{
using namespace uipc::core;
PyFEMSystemFeature::PyFEMSystemFeature(py::module& m)
{
    auto class_FEMSystemFeature =
        py::class_<FEMSystemFeature, IFeature, S<FEMSystemFeature>>(m,
                                                                    "FEMSystemFeature",
                                                                    R"(Feature for exporting FEM energy, gradients, and Hessians.
All quantities carry the solver's dt^2 time-integration scaling: divide by dt^2 for the physical values.)");

    class_FEMSystemFeature.def(
        "fem_constitution_uids",
        [](const FEMSystemFeature& self) { return self.fem_constitution_uids(); },
        R"(UIDs of the constitutions available for export.)");

    class_FEMSystemFeature.def(
        "fem_energy",
        [](FEMSystemFeature& self, U64 uid, geometry::Geometry& energy)
        { self.fem_energy(uid, energy); },
        py::arg("uid"),
        py::arg("energy"),
        R"(Compute per-element FEM energy for a constitution UID.
Each instance holds "energy" <float> and the "topo" element vertex ids when defined.
Note: some constitutions (e.g. StableNeoHookean) have a nonzero rest energy by design.
Args:
    uid: Constitution UID (see fem_constitution_uids()).
    energy: Geometry to store energy values (modified in-place).)");

    class_FEMSystemFeature.def(
        "fem_gradient",
        [](FEMSystemFeature& self, U64 uid, geometry::Geometry& vert_grad)
        { self.fem_gradient(uid, vert_grad); },
        py::arg("uid"),
        py::arg("vert_grad"),
        R"(Compute per-element FEM gradient for a constitution UID.
Each instance holds "i" <int> vertex index and "grad" <Vector3>.
Args:
    uid: Constitution UID (see fem_constitution_uids()).
    vert_grad: Geometry to store gradient doublets (modified in-place).)");

    class_FEMSystemFeature.def(
        "fem_hessian",
        [](FEMSystemFeature& self, U64 uid, geometry::Geometry& vert_hess)
        { self.fem_hessian(uid, vert_hess); },
        py::arg("uid"),
        py::arg("vert_hess"),
        R"(Compute per-element FEM Hessian for a constitution UID.
Each instance holds "i", "j" <int> vertex indices and "hess" <Matrix3x3>.
Args:
    uid: Constitution UID (see fem_constitution_uids()).
    vert_hess: Geometry to store Hessian triplets (modified in-place).)");

    class_FEMSystemFeature.def(
        "fem_vertex_gradient",
        [](FEMSystemFeature& self, U64 uid, geometry::SimplicialComplex& vert_grad)
        { self.fem_vertex_gradient(uid, vert_grad); },
        py::arg("uid"),
        py::arg("vert_grad"),
        R"(Compute per-vertex aggregated FEM gradient for a constitution UID.
Adds a "gradient" <Vector3> vertex attribute; the force is -gradient / dt^2.
The geometry must be created by FiniteElementStateAccessorFeature.create_geometry().
Args:
    uid: Constitution UID (see fem_constitution_uids()).
    vert_grad: Geometry receiving the vertex gradient (modified in-place).)");

    class_FEMSystemFeature.def(
        "fem_energy",
        [](FEMSystemFeature& self, const constitution::IConstitution& c, geometry::Geometry& energy)
        { self.fem_energy(c, energy); },
        py::arg("constitution"),
        py::arg("energy"),
        R"(Compute per-element FEM energy for the given constitution.
Args:
    constitution: The constitution to export (e.g. StableNeoHookean()).
    energy: Geometry to store energy values (modified in-place).)");

    class_FEMSystemFeature.def(
        "fem_gradient",
        [](FEMSystemFeature& self, const constitution::IConstitution& c, geometry::Geometry& vert_grad)
        { self.fem_gradient(c, vert_grad); },
        py::arg("constitution"),
        py::arg("vert_grad"),
        R"(Compute per-element FEM gradient for the given constitution.
Args:
    constitution: The constitution to export.
    vert_grad: Geometry to store gradient doublets (modified in-place).)");

    class_FEMSystemFeature.def(
        "fem_hessian",
        [](FEMSystemFeature& self, const constitution::IConstitution& c, geometry::Geometry& vert_hess)
        { self.fem_hessian(c, vert_hess); },
        py::arg("constitution"),
        py::arg("vert_hess"),
        R"(Compute per-element FEM Hessian for the given constitution.
Args:
    constitution: The constitution to export.
    vert_hess: Geometry to store Hessian triplets (modified in-place).)");

    class_FEMSystemFeature.def(
        "fem_vertex_gradient",
        [](FEMSystemFeature& self, const constitution::IConstitution& c, geometry::SimplicialComplex& vert_grad)
        { self.fem_vertex_gradient(c, vert_grad); },
        py::arg("constitution"),
        py::arg("vert_grad"),
        R"(Compute per-vertex aggregated FEM gradient for the given constitution.
Args:
    constitution: The constitution to export.
    vert_grad: Geometry receiving the vertex gradient (modified in-place).)");

    class_FEMSystemFeature.attr("FeatureName") = FEMSystemFeature::FeatureName;
}
}  // namespace pyuipc::core
