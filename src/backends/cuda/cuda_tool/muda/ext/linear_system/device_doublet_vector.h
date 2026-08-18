#pragma once
#include <muda/buffer/device_buffer.h>
#include <muda/ext/eigen/eigen_core_cxx20.h>
#include <muda/ext/linear_system/doublet_vector_view.h>

namespace muda::details
{
template <typename T, int M, int N>
class MatrixFormatConverter;
}

namespace muda
{
template <typename T, int N>
class DeviceDoubletVector
{
    template <typename U, int M_, int N_>
    friend class details::MatrixFormatConverter;

  public:
    using ValueT = std::conditional_t<N == 1, T, Eigen::Vector<T, N>>;
    static constexpr bool IsSegmentVector = (N > 1);

  protected:
    muda::DeviceBuffer<ValueT> m_values;
    muda::DeviceBuffer<int>    m_indices;
    int                        m_count = 0;

  public:
    DeviceDoubletVector()  = default;
    ~DeviceDoubletVector() = default;

    void reshape(int num_segment) { m_count = num_segment; }

    void resize_doublets(size_t nonzero_count)
    {
        m_values.resize(nonzero_count);
        m_indices.resize(nonzero_count);
    }

    void reserve_doublets(size_t nonzero_count)
    {
        m_values.reserve(nonzero_count);
        m_indices.reserve(nonzero_count);
    }

    void resize(int num_segment, size_t nonzero_count)
    {
        reshape(num_segment);
        resize_doublets(nonzero_count);
    }

    void clear()
    {
        m_values.clear();
        m_indices.clear();
    }

    auto count() const { return m_count; }
    auto values() { return m_values.view(); }
    auto values() const { return m_values.view(); }
    auto indices() { return m_indices.view(); }
    auto indices() const { return m_indices.view(); }

    auto doublet_count() const { return m_values.size(); }
    auto doublet_capacity() const { return m_values.capacity(); }

    auto view()
    {
        return DoubletVectorView<T, N>{
            m_count, (int)m_values.size(), m_indices.data(), m_values.data()};
    }

    auto view() const { return remove_const(*this).view().as_const(); }

    auto cview() const { return view(); }
    auto viewer() { return view().viewer(); }
    auto viewer() const { return view().cviewer(); };
    auto cviewer() const { return view().cviewer(); };
};
}  // namespace muda

#include "details/device_doublet_vector.inl"
