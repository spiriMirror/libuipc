#include <pyuipc/constitution/elastic_moduli.h>
#include <uipc/constitution/elastic_moduli.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyElasticModuli::PyElasticModuli(py::module& m)
{
    py::class_<ElasticModuli>(m, "ElasticModuli")
        .def_static("lame", &ElasticModuli::lame)
        .def_static("youngs_shear", &ElasticModuli::youngs_shear)
        .def_static("youngs_poisson", &ElasticModuli::youngs_poisson)
        .def("lambda", &ElasticModuli::lambda)
        .def("mu", &ElasticModuli::mu);
}
}  // namespace pyuipc::constitution
