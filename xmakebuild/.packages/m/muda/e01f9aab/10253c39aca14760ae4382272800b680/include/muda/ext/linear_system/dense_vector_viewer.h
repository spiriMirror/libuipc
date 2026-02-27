#pragma once
#include <muda/ext/eigen/eigen_core_cxx20.h>
#include <muda/buffer/buffer_2d_view.h>
#include <muda/viewer/viewer_base.h>
#include <cublas_v2.h>
#include <muda/atomic.h>
namespace muda
{
template <bool IsConst, typename T>
class DenseVectorViewerT : public ViewerBase<IsConst>
{
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                  "now only support real number");

    using Base = ViewerBase<IsConst>;
    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    template <bool OtherIsConst, typename U>
    friend class DenseVectorViewerT;

  public:
    using CBufferView    = CBufferView<T>;
    using BufferView     = BufferView<T>;
    using ThisBufferView = std::conditional_t<IsConst, CBufferView, BufferView>;

    using ConstViewer    = DenseVectorViewerT<true, T>;
    using NonConstViewer = DenseVectorViewerT<false, T>;
    using ThisViewer = std::conditional_t<IsConst, ConstViewer, NonConstViewer>;

    using VectorType = Eigen::Vector<T, Eigen::Dynamic>;
    template <typename U>
    using MapVectorT =
        Eigen::Map<U, Eigen::AlignmentType::Unaligned, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>;
    using MapVector     = MapVectorT<VectorType>;
    using CMapVector    = MapVectorT<const VectorType>;
    using ThisMapVector = std::conditional_t<IsConst, CMapVector, MapVector>;

    MUDA_VIEWER_COMMON_NAME(DenseVectorViewerT);

  protected:
    auto_const_t<T>* m_data;
    int              m_offset      = 0;
    int              m_size        = 0;
    int              m_origin_size = 0;

  public:
    MUDA_GENERIC DenseVectorViewerT(auto_const_t<T>* data, int offset, int size, int origin_size)
        : m_data(data)
        , m_offset(offset)
        , m_size(size)
        , m_origin_size(origin_size)
    {
    }

    template <bool OtherIsConst>
    MUDA_GENERIC DenseVectorViewerT(const DenseVectorViewerT<OtherIsConst, T>& other)
        MUDA_REQUIRES(IsConst)
        : m_data(other.m_data)
        , m_offset(other.m_offset)
        , m_size(other.m_size)
        , m_origin_size(other.m_origin_size)
    {
        static_assert(IsConst);
    }

    MUDA_GENERIC auto as_const() const
    {
        return ConstViewer{m_data, m_offset, m_size, m_origin_size};
    }

    MUDA_GENERIC auto segment(int offset, int size) const
    {
        check_segment(offset, size);
        auto ret = ThisViewer{m_data, m_offset + offset, size, m_origin_size};
        ret.copy_label(*this);
        return ret;
    }

    template <int N>
    MUDA_GENERIC auto segment(int offset) const
    {
        return segment(offset, N);
    }

    MUDA_GENERIC auto_const_t<T>& operator()(int i) const
    {
        return m_data[index(i)];
    }

    MUDA_GENERIC Eigen::VectorBlock<ThisMapVector> as_eigen() const
    {
        check_data();
        return ThisMapVector{m_data,
                             (int)origin_size(),
                             Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>{1, 1}}
            .segment(m_offset, m_size);
    }

    MUDA_GENERIC operator Eigen::VectorBlock<ThisMapVector>()
    {
        return as_eigen();
    }

    MUDA_GENERIC auto size() const { return m_size; }

    MUDA_GENERIC auto offset() const { return m_offset; }

    MUDA_GENERIC auto origin_data() const { return m_data; }

    MUDA_GENERIC auto origin_size() const { return m_origin_size; }

    MUDA_DEVICE T atomic_add(int i, T val) const MUDA_REQUIRES(!IsConst)
    {
        auto ptr = &this->operator()(i);
        return muda::atomic_add(ptr, val);
    }

    template <int N>
    MUDA_DEVICE Eigen::Vector<T, N> atomic_add(const Eigen::Vector<T, N>& val) const
        MUDA_REQUIRES(!IsConst)
    {
        this->check_size_matching(N);
        Eigen::Vector<T, N> ret;
#pragma unroll
        for(int i = 0; i < N; ++i)
        {
            ret(i) = atomic_add(i, val(i));
        }
        return ret;
    }

    MUDA_DEVICE T atomic_add(const T& val)
    {
        this->check_size_matching(1);
        T ret = atomic_add(0, val);
        return ret;
    }

    template <int N>
    MUDA_GENERIC DenseVectorViewerT& operator=(const Eigen::Vector<T, N>& other)
    {
        this->check_size_matching(N);
#pragma unroll
        for(int i = 0; i < N; ++i)
        {
            this->operator()(i) = other(i);
        }
        return *this;
    }


  protected:
    MUDA_INLINE MUDA_GENERIC void check_size_matching(int N) const
    {
        MUDA_KERNEL_ASSERT(m_size == N,
                           "DenseVectorViewerBase [%s:%s]: size not match, yours size=%d, expected size=%d. %s(%d)",
                           this->name(),
                           this->kernel_name(),
                           m_size,
                           N,
                           this->kernel_file(),
                           this->kernel_line());
    }

    MUDA_INLINE MUDA_GENERIC int index(int i) const
    {
        MUDA_KERNEL_ASSERT(origin_data(),
                           "DenseVectorViewerBase [%s:%s]: data is null. %s(%d)",
                           this->name(),
                           this->kernel_name(),
                           this->kernel_file(),
                           this->kernel_line());
        MUDA_KERNEL_ASSERT(i < m_size,
                           "DenseVectorViewerBase [%s:%s]: index out of range, size=%d, yours index=%d. %s(%d)",
                           this->name(),
                           this->kernel_name(),
                           m_size,
                           i,
                           this->kernel_file(),
                           this->kernel_line());
        return m_offset + i;
    }

    MUDA_INLINE MUDA_GENERIC void check_data() const
    {
        MUDA_KERNEL_ASSERT(origin_data(),
                           "DenseVectorViewerBase [%s:%s]: data is null. %s(%d)",
                           this->name(),
                           this->kernel_name(),
                           this->kernel_file(),
                           this->kernel_line());
    }

    MUDA_INLINE MUDA_GENERIC void check_segment(int offset, int size) const
    {
        MUDA_KERNEL_ASSERT(offset + size <= m_size,
                           "DenseVectorViewerBase [%s:%s]: segment out of range, m_size=%d, offset=%d, size=%d. %s(%d)",
                           this->name(),
                           this->kernel_name(),
                           m_size,
                           offset,
                           size,
                           this->kernel_file(),
                           this->kernel_line());
    }
};

template <typename T>
using DenseVectorViewer = DenseVectorViewerT<false, T>;

template <typename T>
using CDenseVectorViewer = DenseVectorViewerT<true, T>;
}  // namespace muda

#include "details/dense_vector_viewer.inl"