#include <uipc/constitution/elastic_moduli.h>
#include <uipc/constitution/conversion.h>
#include <uipc/common/exception.h>

namespace uipc::constitution
{
ElasticModuli ElasticModuli::lame(Float lambda, Float mu) noexcept
{
    return ElasticModuli{lambda, mu};
}

ElasticModuli ElasticModuli::youngs_shear(Float E, Float G) noexcept
{
    Float lambda, mu;
    Float poisson;
    EG_to_lame(E, G, lambda, mu, poisson);
    return ElasticModuli{lambda, mu};
}

ElasticModuli ElasticModuli::youngs_poisson(Float E, Float nu)
{
    Float lambda, mu;
    EP_to_lame(E, nu, lambda, mu);
    if(nu == 0.5)
        throw Exception("Poission Rate can't be 0.5");
    return ElasticModuli{lambda, mu};
}

ElasticModuli::ElasticModuli(Float lambda, Float mu) noexcept
    : m_lambda(lambda)
    , m_mu(mu)
{
}
}  // namespace uipc::constitution
