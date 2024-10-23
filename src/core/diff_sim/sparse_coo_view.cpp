#include <uipc/diff_sim/sparse_coo_view.h>

namespace uipc::diff_sim
{
SparseCOOView::SparseCOOView(span<const IndexT> row_indices,
                             span<const IndexT> col_indices,
                             span<const Float>  values,
                             Vector2i           shape)
    : m_row_indices(row_indices)
    , m_col_indices(col_indices)
    , m_values(values)
    , m_shape(shape)
{
}

span<const IndexT> SparseCOOView::row_indices() const
{
    return m_row_indices;
}

span<const IndexT> SparseCOOView::col_indices() const
{
    return m_col_indices;
}

span<const Float> SparseCOOView::values() const
{
    return m_values;
}

Vector2i SparseCOOView::shape() const
{
    return m_shape;
}
}  // namespace uipc::diff_sim