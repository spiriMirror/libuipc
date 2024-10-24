#pragma once
#include <uipc/common/macro.h>
#include <uipc/common/type_define.h>
#include <uipc/common/span.h>
namespace uipc::diff_sim
{
class UIPC_CORE_API SparseCOOView
{
  public:
    SparseCOOView(span<const IndexT> row_indices,
                  span<const IndexT> col_indices,
                  span<const Float>  values,
                  Vector2i           shape);

    span<const IndexT> row_indices() const;
    span<const IndexT> col_indices() const;
    span<const Float>  values() const;
    Vector2i           shape() const;

  private:
    span<const IndexT> m_row_indices;
    span<const IndexT> m_col_indices;
    span<const Float>  m_values;
    Vector2i           m_shape;
};
}  // namespace uipc::diff_sim