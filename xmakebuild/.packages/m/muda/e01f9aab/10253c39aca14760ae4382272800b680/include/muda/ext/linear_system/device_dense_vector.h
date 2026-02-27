#pragma once
#include <muda/ext/linear_system/dense_vector_view.h>
#include <muda/buffer/device_buffer.h>
#include <cusparse.h>

namespace muda::details
{
template <typename T, int M, int N>
class MatrixFormatConverter;
}

namespace muda
{
template <typename T>
class DeviceDenseVector
{
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                  "now only support real number");

    muda::DeviceBuffer<T> m_data;
    cusparseDnVecDescr_t  m_descr = nullptr;

  public:
    DeviceDenseVector() = default;
    DeviceDenseVector(size_t size);
    ~DeviceDenseVector();

    DeviceDenseVector(const DeviceDenseVector&);
    DeviceDenseVector(DeviceDenseVector&&);
    DeviceDenseVector& operator=(const DeviceDenseVector&);
    DeviceDenseVector& operator=(DeviceDenseVector&&);

    void reserve(size_t size);
    void resize(size_t size);
    void fill(T value);
    void copy_to(Eigen::VectorX<T>& vec) const;
    void copy_to(std::vector<T>& vec) const;

    DeviceDenseVector(const Eigen::VectorX<T>& vec);
    DeviceDenseVector& operator=(const Eigen::VectorX<T>& vec);

    DenseVectorViewer<T>  viewer() { return view().viewer(); }
    CDenseVectorViewer<T> viewer() const { return view().cviewer(); }

    auto capacity() const { return m_data.capacity(); }
    auto size() const { return m_data.size(); }
    auto buffer_view() const { return m_data.view(); }
    auto buffer_view() { return m_data.view(); }

    CDenseVectorView<T> view() const;
    DenseVectorView<T>  view();
    CDenseVectorView<T> cview() const { return view(); }

    operator CDenseVectorView<T>() const { return view(); }
    operator DenseVectorView<T>() { return view(); }

    cusparseDnVecDescr_t descr() const { return m_descr; }
};
}  // namespace muda

#include "details/device_dense_vector.inl"
