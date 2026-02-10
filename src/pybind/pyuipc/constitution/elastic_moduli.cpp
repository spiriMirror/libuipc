#include <pyuipc/constitution/elastic_moduli.h>
#include <uipc/constitution/elastic_moduli.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyElasticModuli::PyElasticModuli(py::module& m)
{
    py::class_<ElasticModuli>(m, "ElasticModuli", R"(ElasticModuli class for elastic material parameters (Lame parameters).)")
        .def_static("lame",
                    &ElasticModuli::lame,
                    py::arg("lambda_"),
                    py::arg("mu"),
                    R"(Create elastic moduli from Lame parameters.
Args:
    lambda_: First Lame parameter.
    mu: Second Lame parameter (shear modulus).
Returns:
    ElasticModuli: Elastic moduli object.)")
        .def_static("youngs_shear",
                    &ElasticModuli::youngs_shear,
                    py::arg("E"),
                    py::arg("G"),
                    R"(Create elastic moduli from Young's modulus and shear modulus.
Args:
    E: Young's modulus.
    G: Shear modulus.
Returns:
    ElasticModuli: Elastic moduli object.)")
        .def_static("youngs_poisson",
                    &ElasticModuli::youngs_poisson,
                    py::arg("E"),
                    py::arg("nu"),
                    R"(Create elastic moduli from Young's modulus and Poisson's ratio.
Args:
    E: Young's modulus.
    nu: Poisson's ratio.
Returns:
    ElasticModuli: Elastic moduli object.)")
        .def("lambda_",
             &ElasticModuli::lambda,
             R"(Get the first Lame parameter (lambda).
Returns:
    float: First Lame parameter.)")
        .def("mu",
             &ElasticModuli::mu,
             R"(Get the second Lame parameter (mu, shear modulus).
Returns:
    float: Second Lame parameter.)")
        .def("__repr__",
             [](const ElasticModuli& self)
             {
                 return fmt::format("ElasticModuli(lambda_={}, mu={})", self.lambda(), self.mu());
             },
             R"(String representation of elastic moduli.)");


    py::class_<ElasticModuli2D>(m, "ElasticModuli2D", R"(ElasticModuli2D class for codim 2D elastic material parameters (Lame parameters).)")
        .def_static("lame",
                    &ElasticModuli2D::lame,
                    py::arg("lambda_"),
                    py::arg("mu"),
                    R"(Create 2D elastic moduli from Lame parameters.
Args:
    lambda_: First Lame parameter.
    mu: Second Lame parameter (shear modulus).
Returns:
    ElasticModuli2D: 2D Elastic moduli object.)")
        .def_static("youngs_shear",
                    &ElasticModuli2D::youngs_shear,
                    py::arg("E"),
                    py::arg("G"),
                    R"(Create 2D elastic moduli from Young's modulus and shear modulus.
Args:
    E: Young's modulus.
    G: Shear modulus.
Returns:
    ElasticModuli2D: 2D Elastic moduli object.)")
        .def_static("youngs_poisson",
                    &ElasticModuli2D::youngs_poisson,
                    py::arg("E"),
                    py::arg("nu"),
                    R"(Create 2D elastic moduli from Young's modulus and Poisson's ratio.
Args:
    E: Young's modulus.
    nu: Poisson's ratio.
Returns:
    ElasticModuli2D: 2D Elastic moduli object.)")
        .def("lambda_",
             &ElasticModuli2D::lambda,
             R"(Get the first Lame parameter (lambda).
Returns:
    float: First Lame parameter.)")
        .def("mu",
             &ElasticModuli2D::mu,
             R"(Get the second Lame parameter (mu, shear modulus).
Returns:
    float: Second Lame parameter.)")
        .def("__repr__",
             [](const ElasticModuli2D& self)
             {
                 return fmt::format("ElasticModuli2D(lambda_={}, mu={})", self.lambda(), self.mu());
             },
             R"(String representation of 2D elastic moduli.)");
}
}  // namespace pyuipc::constitution
