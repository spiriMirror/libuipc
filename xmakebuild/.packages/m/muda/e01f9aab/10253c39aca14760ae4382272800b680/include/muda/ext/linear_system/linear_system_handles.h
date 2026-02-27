#pragma once
#include <cublas_v2.h>
#include <cusparse_v2.h>
#include <cusolverDn.h>
#include <cusolverSp.h>
#include <muda/muda_def.h>
#include <muda/check/check_cusparse.h>
#include <muda/check/check_cublas.h>
#include <muda/check/check_cusolver.h>
#include <muda/check/check.h>

namespace muda
{
class LinearSystemContext;
class LinearSystemHandles
{
    friend class LinearSystemContext;
    cudaStream_t       m_stream              = nullptr;
    cublasHandle_t     m_cublas              = nullptr;
    cusparseHandle_t   m_cusparse            = nullptr;
    cusolverDnHandle_t m_cusolver_dn         = nullptr;
    cusolverSpHandle_t m_cusolver_sp         = nullptr;
    bool               m_pointer_mode_device = false;
    float              m_reserve_ratio         = 1.5f;

  public:
    LinearSystemHandles(cudaStream_t s)
        : m_stream(s)
    {
        checkCudaErrors(cusparseCreate(&m_cusparse));
        checkCudaErrors(cublasCreate(&m_cublas));
        checkCudaErrors(cusolverDnCreate(&m_cusolver_dn));
        checkCudaErrors(cusparseSetStream(m_cusparse, m_stream));
        checkCudaErrors(cublasSetStream(m_cublas, m_stream));
        checkCudaErrors(cusolverDnSetStream(m_cusolver_dn, m_stream));
        checkCudaErrors(cusolverSpCreate(&m_cusolver_sp));
        checkCudaErrors(cusolverSpSetStream(m_cusolver_sp, m_stream));
        set_pointer_mode_host();
    }
    ~LinearSystemHandles()
    {
        if(m_cusparse)
            checkCudaErrors(cusparseDestroy(m_cusparse));
        if(m_cublas)
            checkCudaErrors(cublasDestroy(m_cublas));
        if(m_cusolver_dn)
            checkCudaErrors(cusolverDnDestroy(m_cusolver_dn));
        if(m_cusolver_sp)
            checkCudaErrors(cusolverSpDestroy(m_cusolver_sp));
    }

    void stream(cudaStream_t s)
    {
        m_stream = s;
        checkCudaErrors(cusparseSetStream(m_cusparse, m_stream));
        checkCudaErrors(cublasSetStream(m_cublas, m_stream));
        checkCudaErrors(cusolverDnSetStream(m_cusolver_dn, m_stream));
        checkCudaErrors(cusolverSpSetStream(m_cusolver_sp, m_stream));
    }

    MUDA_INLINE void set_pointer_mode_device()
    {
        if(m_pointer_mode_device)
            return;
        checkCudaErrors(cusparseSetPointerMode(m_cusparse, CUSPARSE_POINTER_MODE_DEVICE));
        checkCudaErrors(cublasSetPointerMode(m_cublas, CUBLAS_POINTER_MODE_DEVICE));
        m_pointer_mode_device = true;
    }

    MUDA_INLINE void set_pointer_mode_host()
    {
        if(!m_pointer_mode_device)
            return;
        checkCudaErrors(cusparseSetPointerMode(m_cusparse, CUSPARSE_POINTER_MODE_HOST));
        checkCudaErrors(cublasSetPointerMode(m_cublas, CUBLAS_POINTER_MODE_HOST));
        m_pointer_mode_device = false;
    }

    cudaStream_t       stream() const { return m_stream; }
    cublasHandle_t     cublas() const { return m_cublas; }
    cusparseHandle_t   cusparse() const { return m_cusparse; }
    cusolverDnHandle_t cusolver_dn() const { return m_cusolver_dn; }
    cusolverSpHandle_t cusolver_sp() const { return m_cusolver_sp; }
    auto reserve_ratio() const { return m_reserve_ratio; }
};
}  // namespace muda