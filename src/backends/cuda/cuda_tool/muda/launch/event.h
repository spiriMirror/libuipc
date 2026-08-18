#pragma once
#include <muda/tools/flag.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <device_launch_parameters.h>
#include <muda/check/check_cuda_errors.h>

namespace muda
{
/// <summary>
/// RAII wrapper for cudaEvent
/// </summary>
class Event
{
    cudaEvent_t m_handle = nullptr;

  public:
    enum class Bit : unsigned int
    {
        eDefault = cudaEventDefault,           /**< Default event flag */
        eBlockingSync = cudaEventBlockingSync, /**< Event uses blocking synchronization */
        eDisableTiming = cudaEventDisableTiming, /**< Event will not record timing data */
        eInterprocess = cudaEventInterprocess /**< Event is suitable for interprocess use. cudaEventDisableTiming must be set */
    };

    enum class QueryResult
    {
        eFinished = cudaSuccess,       /**< The event has been recorded */
        eNotReady = cudaErrorNotReady, /**< The event has not been recorded yet */
    };

    Event(Flags<Bit> flag = Bit::eDisableTiming);
    ~Event();

    QueryResult query() const;
    // elapsed time (in ms) between two events
    static float elapsed_time(cudaEvent_t start, cudaEvent_t stop);

    operator cudaEvent_t() { return m_handle; }
    cudaEvent_t viewer() const { return m_handle; }

    // delete copy constructor and assignment operator
    Event(const Event&)            = delete;
    Event& operator=(const Event&) = delete;

    // allow move constructor
    Event(Event&& o) MUDA_NOEXCEPT;
    // delete move assignment operator
    Event& operator=(Event&& o) MUDA_NOEXCEPT;
};
}  // namespace muda

#include "details/event.inl"