#pragma once
// Kernel-side streaming output for debugging (device printf based).
#include <cuda_tool/stream.h>
#include <Eigen/Core>

namespace uipc::backend::cuda_tool
{
// Device-capturable handle streamed with `cout << ...` inside kernels.
// Each fragment is emitted with an immediate device printf (debug-only aid).
class LoggerViewer
{
  public:
    LoggerViewer() = default;

    __device__ const LoggerViewer& operator<<(const char* s) const
    {
        printf("%s", s);
        return *this;
    }
    __device__ const LoggerViewer& operator<<(char c) const
    {
        printf("%c", c);
        return *this;
    }
    __device__ const LoggerViewer& operator<<(int v) const
    {
        printf("%d", v);
        return *this;
    }
    __device__ const LoggerViewer& operator<<(unsigned int v) const
    {
        printf("%u", v);
        return *this;
    }
    __device__ const LoggerViewer& operator<<(long long v) const
    {
        printf("%lld", v);
        return *this;
    }
    __device__ const LoggerViewer& operator<<(unsigned long long v) const
    {
        printf("%llu", v);
        return *this;
    }
    __device__ const LoggerViewer& operator<<(float v) const
    {
        printf("%f", (double)v);
        return *this;
    }
    __device__ const LoggerViewer& operator<<(double v) const
    {
        printf("%f", v);
        return *this;
    }
    // Eigen dense matrices/vectors/expressions: printed as [a, b, ...] rows
    template <typename Derived>
    __device__ const LoggerViewer& operator<<(const Eigen::MatrixBase<Derived>& M) const
    {
        for(int i = 0; i < M.rows(); ++i)
        {
            printf("[");
            for(int j = 0; j < M.cols(); ++j)
                printf("%f%s", (double)M(i, j), j + 1 < M.cols() ? ", " : "");
            printf("]%s", i + 1 < M.rows() ? "\n" : "");
        }
        return *this;
    }
};

// A thread-local kernel console used across the backend.
class KernelCout
{
  public:
    KernelCout() = default;

    static LoggerViewer viewer() { return {}; }
};
}  // namespace uipc::backend::cuda_tool
