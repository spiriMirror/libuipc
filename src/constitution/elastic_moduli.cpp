#include <uipc/constitution/elastic_moduli.h>
#include <uipc/constitution/conversion.h>
#include <uipc/common/exception.h>
#include <uipc/common/log.h>

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
    UIPC_ASSERT(abs(nu) != 0.5 && abs(nu) != 1.0,
                "Poission Rate ({}) can't be 0.5 or -1.0 in ElasticModuli",
                nu);
    return ElasticModuli{lambda, mu};
}

ElasticModuli::ElasticModuli(Float lambda, Float mu) noexcept
    : m_lambda(lambda)
    , m_mu(mu)
{
}

ElasticModuli2D ElasticModuli2D::lame(Float lambda, Float mu) noexcept
{
    return ElasticModuli2D{lambda, mu};
}

ElasticModuli2D ElasticModuli2D::youngs_shear(Float E, Float G) noexcept
{
    Float lambda, mu;
    Float poisson;
    EG_to_lame_2D(E, G, lambda, mu, poisson);
    return ElasticModuli2D{lambda, mu};
}

ElasticModuli2D ElasticModuli2D::youngs_poisson(Float E, Float nu)
{
    Float lambda, mu;
    EP_to_lame_2D(E, nu, lambda, mu);
    UIPC_ASSERT(abs(nu) != 1.0, "Poission Rate ({}) can't be 1.0 or -1.0 in ElasticModuli 2D", nu);
    return ElasticModuli2D{lambda, mu};
}

ElasticModuli2D::ElasticModuli2D(Float lambda, Float mu) noexcept
    : m_lambda(lambda)
    , m_mu(mu)
{
}
}  // namespace uipc::constitution
