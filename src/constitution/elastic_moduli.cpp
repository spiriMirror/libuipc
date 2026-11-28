#include <uipc/constitution/elastic_moduli.h>
#include <uipc/constitution/conversion.h>
#include <uipc/common/exception.h>
#include <uipc/common/log.h>
#include <cmath>

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
    UIPC_ASSERT_THROW(std::isfinite(E) && E >= 0,
                      "Young's modulus must be finite and nonnegative, got {}.",
                      E);
    UIPC_ASSERT_THROW(nu > -1 && nu < 0.5, "3D Poisson's ratio must lie in (-1, 0.5), got {}.", nu);
    Float lambda, mu;
    EP_to_lame(E, nu, lambda, mu);
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
    UIPC_ASSERT_THROW(std::isfinite(E) && E >= 0,
                      "Young's modulus must be finite and nonnegative, got {}.",
                      E);
    UIPC_ASSERT_THROW(nu > -1 && nu < 1, "2D Poisson's ratio must lie in (-1, 1), got {}.", nu);
    Float lambda, mu;
    EP_to_lame_2D(E, nu, lambda, mu);
    return ElasticModuli2D{lambda, mu};
}

ElasticModuli2D::ElasticModuli2D(Float lambda, Float mu) noexcept
    : m_lambda(lambda)
    , m_mu(mu)
{
}
}  // namespace uipc::constitution
