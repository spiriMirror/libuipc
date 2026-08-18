#pragma once
#include <muda/ext/linear_system/linear_system_handles.h>
#include <muda/ext/linear_system/device_dense_matrix.h>
#include <muda/ext/linear_system/device_dense_vector.h>
#include <muda/ext/linear_system/device_triplet_matrix.h>
#include <muda/ext/linear_system/device_doublet_vector.h>
#include <muda/ext/linear_system/device_bcoo_matrix.h>
#include <muda/ext/linear_system/device_bcoo_vector.h>
#include <muda/ext/linear_system/device_bsr_matrix.h>
#include <muda/ext/linear_system/device_csr_matrix.h>

namespace muda::details
{
class MatrixFormatConverterBase;
template <typename T, int M, int N>
class MatrixFormatConverter;

class MatrixFormatConverterType
{
  public:
    cudaDataType_t data_type;
    int            M;
    int            N;
    bool friend    operator==(const MatrixFormatConverterType& lhs,
                           const MatrixFormatConverterType& rhs)
    {
        return lhs.data_type == rhs.data_type && lhs.M == rhs.M && lhs.N == rhs.N;
    }
};
}  // namespace muda::details

namespace std
{
template <>
struct hash<muda::details::MatrixFormatConverterType>
{
    size_t operator()(const muda::details::MatrixFormatConverterType& x) const
    {
        return (std::hash<int>()(x.data_type) << 16)
               ^ (std::hash<int>()(x.M) << 8) ^ std::hash<int>()(x.N);
    }
};
}  // namespace std


namespace muda
{

class MatrixFormatConverter
{
    template <typename T>
    using U = std::unique_ptr<T>;
    LinearSystemHandles& m_handles;
    using TypeN = std::pair<cudaDataType_t, int>;
    std::unordered_map<details::MatrixFormatConverterType, U<details::MatrixFormatConverterBase>> m_impls;
    details::MatrixFormatConverterBase* current = nullptr;
    template <typename T, int M, int N>
    details::MatrixFormatConverter<T, M, N>& impl();

  public:
    MatrixFormatConverter(LinearSystemHandles& handles)
        : m_handles(handles)
    {
    }
    ~MatrixFormatConverter();

    // Triplet -> BCOO
    template <typename T, int M, int N>
    void convert(const DeviceTripletMatrix<T, M, N>& from, DeviceBCOOMatrix<T, M, N>& to);

    // BCOO -> Dense Matrix
    template <typename T, int M, int N>
    void convert(const DeviceBCOOMatrix<T, M, N>& from,
                 DeviceDenseMatrix<T>&            to,
                 bool                             clear_dense_matrix = true);

    // BCOO -> COO
    template <typename T, int M, int N>
    void convert(const DeviceBCOOMatrix<T, M, N>& from, DeviceCOOMatrix<T>& to);

    // BCOO -> BSR
    template <typename T, int N>
    void convert(const DeviceBCOOMatrix<T, N, N>& from, DeviceBSRMatrix<T, N>& to);

    // Doublet -> BCOO
    template <typename T, int N>
    void convert(const DeviceDoubletVector<T, N>& from, DeviceBCOOVector<T, N>& to);

    // BCOO -> Dense Vector
    template <typename T, int N>
    void convert(const DeviceBCOOVector<T, N>& from,
                 DeviceDenseVector<T>&         to,
                 bool                          clear_dense_vector = true);

    // Doublet -> Dense Vector
    template <typename T, int N>
    void convert(const DeviceDoubletVector<T, N>& from,
                 DeviceDenseVector<T>&            to,
                 bool                             clear_dense_vector = true);

    // BSR -> CSR
    template <typename T, int N>
    void convert(const DeviceBSRMatrix<T, N>& from, DeviceCSRMatrix<T>& to);

    // Triplet -> COO
    template <typename T>
    void convert(const DeviceTripletMatrix<T, 1>& from, DeviceCOOMatrix<T>& to);

    // COO -> Dense Matrix
    template <typename T>
    void convert(const DeviceCOOMatrix<T>& from,
                 DeviceDenseMatrix<T>&     to,
                 bool                      clear_dense_matrix = true);

    // COO -> CSR
    template <typename T>
    void convert(const DeviceCOOMatrix<T>& from, DeviceCSRMatrix<T>& to);
    template <typename T>
    void convert(DeviceCOOMatrix<T>&& from, DeviceCSRMatrix<T>& to);

    // Doublet -> COO
    template <typename T>
    void convert(const DeviceDoubletVector<T, 1>& from, DeviceCOOVector<T>& to);

    // COO -> Dense Vector
    template <typename T>
    void convert(const DeviceCOOVector<T>& from,
                 DeviceDenseVector<T>&     to,
                 bool                      clear_dense_vector = true);

    // Doublet -> Dense Vector
    template <typename T>
    void convert(const DeviceDoubletVector<T, 1>& from,
                 DeviceDenseVector<T>&            to,
                 bool                             clear_dense_vector = true);
};
}  // namespace muda

#include "details/matrix_format_converter.inl"
