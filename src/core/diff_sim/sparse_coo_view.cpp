#include <uipc/diff_sim/sparse_coo_view.h>
#include <uipc/common/zip.h>
#include <iostream>
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

Matrix<Float, Eigen::Dynamic, Eigen::Dynamic> SparseCOOView::to_dense() const
{
    using DenseMatrix = Matrix<Float, Eigen::Dynamic, Eigen::Dynamic>;
    DenseMatrix dense = DenseMatrix::Zero(m_shape(0), m_shape(1));
    for(auto&& [i, j, v] : zip(m_row_indices, m_col_indices, m_values))
    {
        dense(i, j) = v;
    }
    return dense;
}

Eigen::SparseMatrix<Float> SparseCOOView::to_sparse() const
{
    std::vector<Eigen::Triplet<Float>> triplets;
    triplets.reserve(m_row_indices.size());
    for(auto&& [i, j, v] : zip(m_row_indices, m_col_indices, m_values))
    {
        triplets.emplace_back(i, j, v);
    }
    Eigen::SparseMatrix<Float> sparse(m_shape(0), m_shape(1));
    sparse.setFromTriplets(triplets.begin(), triplets.end());
    return sparse;
}
}  // namespace uipc::diff_sim