#pragma once
#include <muda/buffer/device_buffer.h>
#include <muda/ext/linear_system/type_mapper/data_type_mapper.h>
#include <muda/ext/linear_system/bcoo_vector_view.h>
#include <muda/ext/linear_system/device_doublet_vector.h>

namespace muda
{
template <typename T, int N>
class DeviceBCOOVector : public DeviceDoubletVector<T, N>
{
    template <typename U, int M_, int N_>
    friend class details::MatrixFormatConverter;

  public:
    using SegmentVector = Eigen::Matrix<T, N, 1>;

    DeviceBCOOVector()                                   = default;
    ~DeviceBCOOVector()                                  = default;
    DeviceBCOOVector(const DeviceBCOOVector&)            = default;
    DeviceBCOOVector(DeviceBCOOVector&&)                 = default;
    DeviceBCOOVector& operator=(const DeviceBCOOVector&) = default;
    DeviceBCOOVector& operator=(DeviceBCOOVector&&)      = default;

    auto non_zeros() const { return this->m_values.size(); }
};

template <typename T>
class DeviceBCOOVector<T, 1> : public DeviceDoubletVector<T, 1>
{
    template <typename U, int M, int N>
    friend class details::MatrixFormatConverter;

  protected:
    mutable cusparseSpVecDescr_t m_descr = nullptr;

  public:
    DeviceBCOOVector() = default;
    ~DeviceBCOOVector() { destroy_descr(); }

    DeviceBCOOVector(const DeviceBCOOVector& other)
        : DeviceDoubletVector<T, 1>(other)
        , m_descr(nullptr)
    {
    }

    DeviceBCOOVector(DeviceBCOOVector&& other)
        : DeviceDoubletVector<T, 1>(std::move(other))
        , m_descr(other.m_descr)
    {
        other.m_descr = nullptr;
    }

    DeviceBCOOVector& operator=(const DeviceBCOOVector& other)
    {
        DeviceDoubletVector<T, 1>::operator=(other);
        destroy_descr();
        return *this;
    }

    DeviceBCOOVector& operator=(DeviceBCOOVector&& other)
    {
        DeviceDoubletVector<T, 1>::operator=(std::move(other));
        destroy_descr();
        m_descr       = other.m_descr;
        other.m_descr = nullptr;
        return *this;
    }

    auto non_zeros() const { return this->m_values.size(); }

    auto descr() const
    {
        if(!m_descr)
        {
            checkCudaErrors(cusparseCreateSpVec(
                &m_descr,
                this->m_size,
                this->m_values.size(),
                (int*)this->m_indices.data(),
                (T*)this->m_values.data(),
                cusparse_index_type<decltype(this->m_indices)::value_type>(),
                CUSPARSE_INDEX_BASE_ZERO,
                cuda_data_type<T>()));
        }
        return m_descr;
    }

  private:
    void destroy_descr() const
    {
        if(m_descr)
        {
            checkCudaErrors(cusparseDestroySpVec(m_descr));
            m_descr = nullptr;
        }
    }
};

template <typename T>
using DeviceCOOVector = DeviceBCOOVector<T, 1>;
}  // namespace muda


#include "details/device_bcoo_vector.inl"
