#include <muda/check/check_cusparse.h>
#include <muda/ext/linear_system/type_mapper/data_type_mapper.h>

namespace muda
{
template <typename T>
DeviceDenseVector<T>::DeviceDenseVector(size_t size)
{
    this->resize(size);
}
template <typename T>
DeviceDenseVector<T>::~DeviceDenseVector()
{
    if(m_descr)
        checkCudaErrors(cusparseDestroyDnVec(m_descr));
}
template <typename T>
DeviceDenseVector<T>::DeviceDenseVector(const DeviceDenseVector<T>& other)
    : m_data{other.m_data}
{
    checkCudaErrors(cusparseCreateDnVec(
        &m_descr, m_data.size(), m_data.data(), cuda_data_type<T>()));
}
template <typename T>
DeviceDenseVector<T>::DeviceDenseVector(DeviceDenseVector<T>&& other)
    : m_data{std::move(other.m_data)}
    , m_descr{other.m_descr}
{
    other.m_descr = nullptr;
}
template <typename T>
DeviceDenseVector<T>& DeviceDenseVector<T>::operator=(const DeviceDenseVector<T>& other)
{
    if(this != &other)
    {
        m_data = other.m_data;
        if(m_descr)
            checkCudaErrors(cusparseDestroyDnVec(m_descr));
        checkCudaErrors(cusparseCreateDnVec(
            &m_descr, m_data.size(), m_data.data(), cuda_data_type<T>()));
    }
    return *this;
}
template <typename T>
DeviceDenseVector<T>& DeviceDenseVector<T>::operator=(DeviceDenseVector<T>&& other)
{
    if(this != &other)
    {
        m_data        = std::move(other.m_data);
        m_descr       = other.m_descr;
        other.m_descr = nullptr;
    }
    return *this;
}
template <typename T>
void DeviceDenseVector<T>::reserve(size_t size)
{
    m_data.reserve(size);
}
template <typename T>
void DeviceDenseVector<T>::resize(size_t size)
{
    if(m_descr)
    {
        checkCudaErrors(cusparseDestroyDnVec(m_descr));
    }

    m_data.resize(size);

    checkCudaErrors(
        cusparseCreateDnVec(&m_descr, size, m_data.data(), cuda_data_type<T>()));
}
template <typename T>
void DeviceDenseVector<T>::fill(T value)
{
    m_data.fill(value);
}
template <typename T>
void DeviceDenseVector<T>::copy_to(Eigen::VectorX<T>& vec) const
{
    vec.resize(m_data.size());
    m_data.view().copy_to(vec.data());
}
template <typename T>
void DeviceDenseVector<T>::copy_to(std::vector<T>& vec) const
{
    vec.resize(m_data.size());
    m_data.view().copy_to(vec.data());
}
template <typename T>
DeviceDenseVector<T>::DeviceDenseVector(const Eigen::VectorX<T>& vec)
{
    this->resize(vec.size());
    m_data.view().copy_from(vec.data());
}
template <typename T>
DeviceDenseVector<T>& DeviceDenseVector<T>::operator=(const Eigen::VectorX<T>& vec)
{
    this->resize(vec.size());
    m_data.view().copy_from(vec.data());
    return *this;
}

template <typename T>
CDenseVectorView<T> DeviceDenseVector<T>::view() const
{
    return CDenseVectorView<T>{m_data.data(), descr(), 0, 1, (int)size(), (int)size()};
}

template <typename T>
DenseVectorView<T> DeviceDenseVector<T>::view()
{
    return DenseVectorView<T>{m_data.data(), descr(), 0, 1, (int)size(), (int)size()};
}
}  // namespace muda
