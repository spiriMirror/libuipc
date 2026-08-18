#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cstdio>
#include <muda/muda_def.h>
#include <muda/tools/debug_log.h>
#include <muda/exception.h>
#include <string>
#include <muda/check/check_cusparse.h>
#include <muda/check/check_cublas.h>
#include <muda/check/check_cusolver.h>

MUDA_INLINE MUDA_GENERIC const char* mudaCudaGetErrorEnum(cudaError_t error)
{
//#ifdef __CUDA_ARCH__
//    return "<muda: not impl yet>";
//#else
    return cudaGetErrorName(error);
//#endif
}

namespace muda
{
template <typename T = cudaError_t>
class cuda_error : public exception
{
    T           m_error;
    std::string m_error_string;
    std::string m_file;

    size_t      m_line;
    std::string m_func;

  public:
    cuda_error(T error, std::string error_string, const std::string& file, size_t line, const std::string& func)
        : exception("CUDA error at " + file + ":" + std::to_string(line) + " code="
                    + std::to_string((int)error) + "(" + error_string + ")" + func)
        , m_error(error)
        , m_error_string(error_string)
        , m_file(file)
        , m_line(line)
        , m_func(func)
    {
    }

    T                  error() const { return m_error; }
    const std::string& error_string() const { return m_error_string; }
    const std::string& file() const { return m_file; }
    size_t             line() const { return m_line; }
    const std::string& func() const { return m_func; }
};

template <typename T>
__host__ __device__ inline void check(T                 result,
                                      char const* const func,
                                      const char* const file,
                                      int const         line)
{
#ifdef __CUDA_ARCH__
    if(result)
    {
        printf("CUDA error at %s:%d code=%d(%s) \"%s\" \n",
               file,
               line,
               static_cast<unsigned int>(result),
               mudaCudaGetErrorEnum(result),
               func);
        if constexpr(TRAP_ON_ERROR)
            trap();
    }
#else
    if(result)
    {
        std::fprintf(stderr,
                     "CUDA error at %s:%d code=%d(%s) \"%s\" \n",
                     file,
                     line,
                     static_cast<unsigned int>(result),
                     mudaCudaGetErrorEnum(result),
                     func);
        throw cuda_error<T>(result, mudaCudaGetErrorEnum(result), file, line, func);
    }
#endif
}
}  // namespace muda