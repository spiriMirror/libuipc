#pragma once
#include <type_define.h>

namespace uipc::backend::cuda
{
class DyTopoClassifyInfo
{
    enum class Type
    {
        Range,
        // MultiRange
    };

  public:
    /**
     * @brief The range of contact hessian i,j, and gradient range is empty. (Off-diagnonal)
     * 
     * $$ i \in [LBegin, LEnd) $$, $$ j \in [RBegin, REnd) $$. 
     * Contact hessian $H_{ij}$ will be passed to the reporter later.
     * 
     * @param LRange LRange=[LBegin, LEnd)
     * @param RRange RRange=[RBegin, REnd)
     */
    void range(const Vector2i& LRange, const Vector2i& RRange);

    /**
     * @brief The range of contact hessian and gradient. (Diagnonal)
     * 
     * $$ i \in [Begin, End) $$.
     * Contact gradient $G_{i}$ will be passed to the reporter later.
     * 
     * @param Range Range=[Begin, End)
     */
    void range(const Vector2i& Range);

  public:
    bool is_empty() const;

    bool is_diag() const;

    void sanity_check();

    auto hessian_i_range() const { return m_hessian_i_range; }
    auto hessian_j_range() const { return m_hessian_j_range; }
    auto gradient_i_range() const { return m_gradient_i_range; }

  private:
    Vector2i m_hessian_i_range  = {0, 0};
    Vector2i m_hessian_j_range  = {0, 0};
    Vector2i m_gradient_i_range = {0, 0};
    Type     m_type;
};
}  // namespace uipc::backend::cuda