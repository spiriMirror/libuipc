#pragma once
#include <uipc/common/dllexport.h>
#include <uipc/common/type_define.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API ElasticModuli
{
  public:
    static ElasticModuli lame(Float lambda, Float mu) noexcept;
    static ElasticModuli youngs_shear(Float E, Float G) noexcept;
    static ElasticModuli youngs_poisson(Float E, Float nu);

    auto lambda() const noexcept { return m_lambda; }
    auto mu() const noexcept { return m_mu; }

  private:
    ElasticModuli() = default;
    ElasticModuli(Float lambda, Float mu) noexcept;
    Float m_lambda;
    Float m_mu;
};
}  // namespace uipc::constitution
