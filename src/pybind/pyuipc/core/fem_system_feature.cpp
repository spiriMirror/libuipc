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
                                                                    R"(Feature to export finite element (FEM) energy, gradient and Hessian data.

Data is recomputed on demand at the backend's current positions, so it always
matches the state retrieved by `World.retrieve()`.

All quantities carry the same `dt^2` time-integration scaling as the solver's
incremental potential (consistent with the ContactSystemFeature exports):
divide by `dt^2` to obtain the physical values (energy in J, gradient in J/m,
Hessian in J/m^2).

Obtain the feature via `world.features().find(FEMSystemFeature)`. The geometry
arguments are *output* parameters: create an empty one (e.g. `Geometry()`) and
pass it in; it is filled in place.)");

    class_FEMSystemFeature.def(
        "fem_constitution_uids",
        [](const FEMSystemFeature& self) { return self.fem_constitution_uids(); },
        R"(UIDs of the constitutions available for export.

Returns:
    list[int]: Constitution UIDs, usable as the `uid` argument of the export methods.)");

    class_FEMSystemFeature.def(
        "fem_energy",
        [](FEMSystemFeature& self, U64 uid, geometry::Geometry& energy)
        { self.fem_energy(uid, energy); },
        py::arg("uid"),
        py::arg("energy"),
        R"(Per-element elastic energy of a constitution (modified in place).

Each instance holds:
- "topo" <Vector4i/Vector3i/Vector2i> element vertex indices (global FEM vertex ids;
  absent for particles and extra constitutions),
- "energy" <float> dt^2-scaled elastic energy of the element (divide by dt^2 for J).

Note: some constitutions have a nonzero energy at the rest state by design, e.g.
StableNeoHookean evaluates to Psi(I) = mu^2 / (2*lambda_hat) > 0 at the identity
deformation, so the exported energies are offset by this constant at rest; subtract
it client-side if unwanted (it is constant, so it does not affect the exported
gradient/Hessian).

Args:
    uid: Constitution UID (see `fem_constitution_uids()`).
    energy: Empty Geometry, filled in place.)");

    class_FEMSystemFeature.def(
        "fem_gradient",
        [](FEMSystemFeature& self, U64 uid, geometry::Geometry& vert_grad)
        { self.fem_gradient(uid, vert_grad); },
        py::arg("uid"),
        py::arg("vert_grad"),
        R"(Per-element elastic gradient of a constitution (modified in place).

Each instance holds:
- "i" <int> global FEM vertex index,
- "grad" <Vector3> dt^2-scaled partial derivative of the elastic energy w.r.t.
  the vertex position (divide by dt^2 for J/m). The elastic force on a vertex is
  minus the sum of all its "grad" entries (divided by dt^2).

Args:
    uid: Constitution UID.
    vert_grad: Empty Geometry, filled in place.)");

    class_FEMSystemFeature.def(
        "fem_hessian",
        [](FEMSystemFeature& self, U64 uid, geometry::Geometry& vert_hess)
        { self.fem_hessian(uid, vert_hess); },
        py::arg("uid"),
        py::arg("vert_hess"),
        R"(Per-element elastic Hessian of a constitution (modified in place).

Each instance holds:
- "i" <int>, "j" <int> global FEM vertex indices,
- "hess" <Matrix3x3> dt^2-scaled 3x3 block of the (symmetrized) elastic Hessian
  (divide by dt^2 for J/m^2).

Args:
    uid: Constitution UID.
    vert_hess: Empty Geometry, filled in place.)");

    class_FEMSystemFeature.def(
        "fem_vertex_gradient",
        [](FEMSystemFeature& self, U64 uid, geometry::SimplicialComplex& vert_grad)
        { self.fem_vertex_gradient(uid, vert_grad); },
        py::arg("uid"),
        py::arg("vert_grad"),
        R"(Per-vertex aggregated elastic gradient of a constitution (modified in place).

Adds a "gradient" <Vector3> vertex attribute to `vert_grad`, holding the
dt^2-scaled sum of the per-element gradients acting on each vertex (divide by
dt^2 for J/m). The elastic force on the vertex is `-gradient / dt^2` [N].

`vert_grad` must be created by `FiniteElementStateAccessorFeature.create_geometry()`,
which defines the exported vertex range.

Args:
    uid: Constitution UID.
    vert_grad: State-accessor geometry, modified in place.)");

    class_FEMSystemFeature.def(
        "fem_energy",
        [](FEMSystemFeature& self, const constitution::IConstitution& c, geometry::Geometry& energy)
        { self.fem_energy(c, energy); },
        py::arg("constitution"),
        py::arg("energy"),
        R"(Per-element elastic energy of the given constitution (modified in place).

Args:
    constitution: The constitution to export (e.g. `StableNeoHookean()`).
    energy: Empty Geometry, filled in place.)");

    class_FEMSystemFeature.def(
        "fem_gradient",
        [](FEMSystemFeature& self, const constitution::IConstitution& c, geometry::Geometry& vert_grad)
        { self.fem_gradient(c, vert_grad); },
        py::arg("constitution"),
        py::arg("vert_grad"),
        R"(Per-element elastic gradient of the given constitution (modified in place).

Args:
    constitution: The constitution to export.
    vert_grad: Empty Geometry, filled in place.)");

    class_FEMSystemFeature.def(
        "fem_hessian",
        [](FEMSystemFeature& self, const constitution::IConstitution& c, geometry::Geometry& vert_hess)
        { self.fem_hessian(c, vert_hess); },
        py::arg("constitution"),
        py::arg("vert_hess"),
        R"(Per-element elastic Hessian of the given constitution (modified in place).

Args:
    constitution: The constitution to export.
    vert_hess: Empty Geometry, filled in place.)");

    class_FEMSystemFeature.def(
        "fem_vertex_gradient",
        [](FEMSystemFeature& self, const constitution::IConstitution& c, geometry::SimplicialComplex& vert_grad)
        { self.fem_vertex_gradient(c, vert_grad); },
        py::arg("constitution"),
        py::arg("vert_grad"),
        R"(Per-vertex aggregated elastic gradient of the given constitution (modified in place).

Args:
    constitution: The constitution to export.
    vert_grad: State-accessor geometry, modified in place.)");

    class_FEMSystemFeature.attr("FeatureName") = FEMSystemFeature::FeatureName;
}
}  // namespace pyuipc::core
