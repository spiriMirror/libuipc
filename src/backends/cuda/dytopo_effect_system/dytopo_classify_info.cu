#include <dytopo_effect_system/dytopo_classify_info.h>
#include <uipc/common/log.h>

namespace uipc::backend::cuda
{
void DyTopoClassifyInfo::range(const Vector2i& LRange, const Vector2i& RRange)
{
    m_type             = Type::Range;
    m_hessian_i_range  = LRange;
    m_hessian_j_range  = RRange;
    m_gradient_i_range = Vector2i::Zero();
}

void DyTopoClassifyInfo::range(const Vector2i& Range)
{
    m_type             = Type::Range;
    m_gradient_i_range = Range;
    m_hessian_i_range  = Range;
    m_hessian_j_range  = Range;
}

bool DyTopoClassifyInfo::is_empty() const
{
    return m_hessian_i_range[0] == m_hessian_i_range[1]
           || m_hessian_j_range[0] == m_hessian_j_range[1];
}

bool DyTopoClassifyInfo::is_diag() const
{
    return m_gradient_i_range[0] != m_gradient_i_range[1];
}

void DyTopoClassifyInfo::sanity_check()
{
    if(is_diag())
    {
        UIPC_ASSERT(m_gradient_i_range.x() <= m_gradient_i_range.y(),
                    "Diagonal Contact Gradient Range is invalid");

        UIPC_ASSERT(m_hessian_i_range == m_hessian_j_range,
                    "Diagonal Contact Hessian must have the same i_range and j_range");
    }
    else
    {
        UIPC_ASSERT(m_gradient_i_range.x() == m_gradient_i_range.y(),
                    "Off-Diagonal Contact must not have Gradient Part");
    }

    UIPC_ASSERT(m_hessian_i_range.x() <= m_hessian_i_range.y(),
                "Contact Hessian Range-i is invalid");
    UIPC_ASSERT(m_hessian_j_range.x() <= m_hessian_j_range.y(),
                "Contact Hessian Range-j is invalid");
}
}  // namespace uipc::backend::cuda