#pragma once
#include <cublas_v2.h>
#include <cusparse_v2.h>
#include <cusolverDn.h>
#include <cusolverSp.h>
#include <list>
#include <muda/buffer/device_buffer.h>
#include <muda/literal/unit.h>
#include <muda/mstl/span.h>
#include <muda/ext/linear_system/dense_vector_view.h>
#include <muda/ext/linear_system/dense_matrix_view.h>
#include <muda/ext/linear_system/matrix_format_converter.h>
#include <muda/ext/linear_system/linear_system_handles.h>
#include <muda/ext/linear_system/linear_system_solve_tolerance.h>
#include <muda/ext/linear_system/linear_system_solve_reorder.h>
namespace muda
{
class LinearSystemContextCreateInfo
{
  public:
    cudaStream_t stream = nullptr;
    // base size of temp buffer, if buffer is not enough
    // we create a new buffer with size = buffer_byte_size_base * 2 / 4 / 8 / 16 / ...
    // and we will not release the old buffer because of safety
    size_t buffer_byte_size_base = 256_M;
};
class LinearSystemContext
{
  private:
    LinearSystemHandles                m_handles;
    std::list<DeviceBuffer<std::byte>> m_buffers;
    std::list<std::vector<std::byte>>  m_host_buffers;
    DeviceBuffer<std::byte>            m_scalar_buffer;

    LinearSystemContextCreateInfo    m_create_info;
    std::list<std::function<void()>> m_sync_callbacks;
    std::string                      m_current_label;

    void                  set_pointer_mode_device();
    void                  set_pointer_mode_host();
    void                  shrink_temp_buffers();
    void                  add_sync_callback(std::function<void()>&& callback);
    BufferView<std::byte> temp_buffer(size_t size);
    span<std::byte>       temp_host_buffer(size_t size);
    template <typename T>
    BufferView<T> temp_buffer(size_t size);
    template <typename T>
    span<T> temp_host_buffer(size_t size);
    template <typename T>
    std::vector<T*> temp_buffers(size_t size_in_buffer, size_t num_buffer);
    template <typename T>
    std::vector<T*> temp_host_buffers(size_t size_in_buffer, size_t num_buffer);

    LinearSystemSolveTolerance m_tolerance;
    LinearSystemSolveReorder   m_reorder;
    MatrixFormatConverter      m_converter;

  private:
    auto cublas() const { return m_handles.cublas(); }
    auto cusparse() const { return m_handles.cusparse(); }
    auto cusolver_dn() const { return m_handles.cusolver_dn(); }
    auto cusolver_sp() const { return m_handles.cusolver_sp(); }

  public:
    LinearSystemContext(const LinearSystemContextCreateInfo& info = {});
    LinearSystemContext(const LinearSystemContext&)            = delete;
    LinearSystemContext& operator=(const LinearSystemContext&) = delete;
    LinearSystemContext(LinearSystemContext&&)                 = delete;
    LinearSystemContext& operator=(LinearSystemContext&&)      = delete;
    ~LinearSystemContext();

    void label(std::string_view label) { m_current_label = label; }
    auto label() const -> std::string_view { return m_current_label; }
    auto stream() const { return m_handles.stream(); }
    void stream(cudaStream_t stream);
    void sync();

    /***********************************************************************************************
                                                Settings
    ***********************************************************************************************/

    auto& tolerance() { return m_tolerance; }
    auto& reorder() { return m_reorder; }
    auto  reserve_ratio() const { return m_handles.m_reserve_ratio; }
    void  reserve_ratio(float ratio) { m_handles.m_reserve_ratio = ratio; }


  public:
    /***********************************************************************************************
                                              Converter
    ***********************************************************************************************/
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

  public:
    /***********************************************************************************************
                                                Norm
    ***********************************************************************************************/
    template <typename T>
    T norm(CDenseVectorView<T> x);
    template <typename T>
    void norm(CDenseVectorView<T> x, VarView<T> result);
    template <typename T>
    void norm(CDenseVectorView<T> x, T* result);

    /***********************************************************************************************
                                                Dot
    ***********************************************************************************************/
    template <typename T>
    T dot(CDenseVectorView<T> x, CDenseVectorView<T> y);
    template <typename T>
    void dot(CDenseVectorView<T> x, CDenseVectorView<T> y, VarView<T> result);
    template <typename T>
    void dot(CDenseVectorView<T> x, CDenseVectorView<T> y, T* result);

    /***********************************************************************************************
                                              Max/Min
    ***********************************************************************************************/
    //TODO:


    /***********************************************************************************************
                                               Axpby
                                      y = alpha * x + beta * y
    ***********************************************************************************************/
    // y = alpha * x + beta * y
    template <typename T>
    void axpby(const T& alpha, CDenseVectorView<T> x, const T& beta, DenseVectorView<T> y);
    // y = alpha * x + beta * y
    template <typename T>
    void axpby(CVarView<T> alpha, CDenseVectorView<T> x, CVarView<T> beta, DenseVectorView<T> y);
    // z = x + y
    template <typename T>
    void plus(CDenseVectorView<T> x, CDenseVectorView<T> y, DenseVectorView<T> z);

    /***********************************************************************************************
                                                Spmv
                                        y = a * A * x + b * y
    ***********************************************************************************************/
    // BSR
    template <typename T, int N>
    void spmv(const T&             a,
              CBSRMatrixView<T, N> A,
              CDenseVectorView<T>  x,
              const T&             b,
              DenseVectorView<T>&  y);
    template <typename T, int N>
    void spmv(CBSRMatrixView<T, N> A, CDenseVectorView<T> x, DenseVectorView<T> y);
    // CSR
    template <typename T>
    void spmv(const T& a, CCSRMatrixView<T> A, CDenseVectorView<T> x, const T& b, DenseVectorView<T>& y);
    template <typename T>
    void spmv(CCSRMatrixView<T> A, CDenseVectorView<T> x, DenseVectorView<T> y);
    // BCOO & Triplet
    template <typename T, int M, int N>
    void spmv(const T&                    a,
              CTripletMatrixView<T, M, N> A,
              CDenseVectorView<T>         x,
              const T&                    b,
              DenseVectorView<T>&         y);
    template <typename T, int M, int N>
    void spmv(CTripletMatrixView<T, M, N> A, CDenseVectorView<T> x, DenseVectorView<T> y);
    // COO
    template <typename T>
    void spmv(const T& a, CCOOMatrixView<T> A, CDenseVectorView<T> x, const T& b, DenseVectorView<T>& y);
    template <typename T>
    void spmv(CCOOMatrixView<T> A, CDenseVectorView<T> x, DenseVectorView<T> y);


    /***********************************************************************************************
                                                 Mv
                                        y = a * A * x + b * y
    ***********************************************************************************************/
    template <typename T>
    void mv(CDenseMatrixView<T> A,
            const T&            alpha,
            CDenseVectorView<T> x,
            const T&            beta,
            DenseVectorView<T>  y);
    template <typename T>
    void mv(CDenseMatrixView<T> A,
            CVarView<T>         alpha,
            CDenseVectorView<T> x,
            CVarView<T>         beta,
            DenseVectorView<T>  y);
    template <typename T>
    void mv(CDenseMatrixView<T> A, CDenseVectorView<T> x, DenseVectorView<T> y);

    /***********************************************************************************************
                                                Solve
                                              A * x = b
    ***********************************************************************************************/
    // solve Ax = b, A will be modified for factorization
    // and b will be modified to store the solution
    template <typename T>
    void solve(DenseMatrixView<T> A_to_fact, DenseVectorView<T> b_to_x);
    // solve Ax = b
    // A is the CSR Matrix
    template <typename T>
    void solve(DenseVectorView<T> x, CCSRMatrixView<T> A, CDenseVectorView<T> b);

  private:
    template <typename T>
    void generic_spmv(const T&                  a,
                      cusparseOperation_t       op,
                      cusparseSpMatDescr_t      A,
                      const cusparseDnVecDescr* x,
                      const T&                  b,
                      cusparseDnVecDescr_t      y);
    template <typename T>
    void sysv(DenseMatrixView<T> A_to_fact, DenseVectorView<T> b_to_x);
    template <typename T>
    void gesv(DenseMatrixView<T> A_to_fact, DenseVectorView<T> b_to_x);
};
}  // namespace muda

#include "details/linear_system_context.inl"
#include "details/routines/convert.inl"
#include "details/routines/norm.inl"
#include "details/routines/dot.inl"
#include "details/routines/axpby.inl"
#include "details/routines/spmv.inl"
#include "details/routines/mv.inl"
#include "details/routines/solve.inl"
#include "details/routines/mm.inl"
