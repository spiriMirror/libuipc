#pragma once
#include <muda/launch/launch_base.h>
#include <muda/muda_config.h>
#include <muda/tools/extent.h>
#include <muda/buffer/buffer_fwd.h>

namespace muda
{
template <typename T>
class ComputeGraphVar;

class BufferLaunch : public LaunchBase<BufferLaunch>
{
    int m_grid_dim  = 0;
    int m_block_dim = -1;  // we use automatic block dim choose as default.

  public:
    // default config
    MUDA_HOST BufferLaunch(cudaStream_t s = nullptr) MUDA_NOEXCEPT : LaunchBase(s)
    {
    }

    MUDA_HOST BufferLaunch(int block_dim, cudaStream_t s = nullptr) MUDA_NOEXCEPT
        : LaunchBase(s),
          m_block_dim(block_dim)
    {
    }

    MUDA_HOST BufferLaunch(int grid_dim, int block_dim, cudaStream_t s = nullptr) MUDA_NOEXCEPT
        : LaunchBase(s),
          m_grid_dim(grid_dim),
          m_block_dim(block_dim)
    {
    }

    /**********************************************************************************************
    * 
    * Buffer API
    * 0D DeviceVar
    * 1D DeviceBuffer
    * 2D DeviceBuffer2D
    * 3D DeviceBuffer3D
    * 
    ***********************************************************************************************/
    template <typename T>
    MUDA_HOST BufferLaunch& resize(DeviceBuffer<T>& buffer, size_t size);
    template <typename T>
    MUDA_HOST BufferLaunch& resize(DeviceBuffer2D<T>& buffer, Extent2D extent);
    template <typename T>
    MUDA_HOST BufferLaunch& resize(DeviceBuffer3D<T>& buffer, Extent3D extent);

    template <typename T>
    MUDA_HOST BufferLaunch& reserve(DeviceBuffer<T>& buffer, size_t capacity);
    template <typename T>
    MUDA_HOST BufferLaunch& reserve(DeviceBuffer2D<T>& buffer, Extent2D capacity);
    template <typename T>
    MUDA_HOST BufferLaunch& reserve(DeviceBuffer3D<T>& buffer, Extent3D capacity);


    template <typename T>
    MUDA_HOST BufferLaunch& resize(DeviceBuffer<T>& buffer, size_t size, const T& val);
    template <typename T>
    MUDA_HOST BufferLaunch& resize(DeviceBuffer2D<T>& buffer, Extent2D extent, const T& val);
    template <typename T>
    MUDA_HOST BufferLaunch& resize(DeviceBuffer3D<T>& buffer, Extent3D extent, const T& val);


    template <typename T>
    MUDA_HOST BufferLaunch& clear(DeviceBuffer<T>& buffer);
    template <typename T>
    MUDA_HOST BufferLaunch& clear(DeviceBuffer2D<T>& buffer);
    template <typename T>
    MUDA_HOST BufferLaunch& clear(DeviceBuffer3D<T>& buffer);


    template <typename T>
    MUDA_HOST BufferLaunch& alloc(DeviceBuffer<T>& buffer, size_t n);
    template <typename T>
    MUDA_HOST BufferLaunch& alloc(DeviceBuffer2D<T>& buffer, Extent2D extent);
    template <typename T>
    MUDA_HOST BufferLaunch& alloc(DeviceBuffer3D<T>& buffer, Extent3D extent);


    template <typename T>
    MUDA_HOST BufferLaunch& free(DeviceBuffer<T>& buffer);
    template <typename T>
    MUDA_HOST BufferLaunch& free(DeviceBuffer2D<T>& buffer);
    template <typename T>
    MUDA_HOST BufferLaunch& free(DeviceBuffer3D<T>& buffer);


    template <typename T>
    MUDA_HOST BufferLaunch& shrink_to_fit(DeviceBuffer<T>& buffer);
    template <typename T>
    MUDA_HOST BufferLaunch& shrink_to_fit(DeviceBuffer2D<T>& buffer);
    template <typename T>
    MUDA_HOST BufferLaunch& shrink_to_fit(DeviceBuffer3D<T>& buffer);

    /**********************************************************************************************
    * 
    * BufferView Copy: Device <- Device
    * 
    **********************************************************************************************/
    template <typename T>
    MUDA_HOST BufferLaunch& copy(VarView<T> dst, CVarView<T> src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(BufferView<T> dst, CBufferView<T> src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(Buffer2DView<T> dst, CBuffer2DView<T> src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(Buffer3DView<T> dst, CBuffer3DView<T> src);

    template <typename T>
    MUDA_HOST BufferLaunch& copy(ComputeGraphVar<VarView<T>>&       dst,
                                 const ComputeGraphVar<VarView<T>>& src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(ComputeGraphVar<BufferView<T>>&       dst,
                                 const ComputeGraphVar<BufferView<T>>& src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(ComputeGraphVar<Buffer2DView<T>>&       dst,
                                 const ComputeGraphVar<Buffer2DView<T>>& src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(ComputeGraphVar<Buffer3DView<T>>&       dst,
                                 const ComputeGraphVar<Buffer3DView<T>>& src);

    /**********************************************************************************************
    * 
    * BufferView Copy: Host <- Device
    * 
    **********************************************************************************************/
    template <typename T>
    MUDA_HOST BufferLaunch& copy(T* dst, CVarView<T> src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(T* dst, CBufferView<T> src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(T* dst, CBuffer2DView<T> src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(T* dst, CBuffer3DView<T> src);

    template <typename T>
    MUDA_HOST BufferLaunch& copy(ComputeGraphVar<T*>&                  dst,
                                 const ComputeGraphVar<BufferView<T>>& src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(ComputeGraphVar<T*>&                    dst,
                                 const ComputeGraphVar<Buffer2DView<T>>& src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(ComputeGraphVar<T*>&               dst,
                                 const ComputeGraphVar<VarView<T>>& src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(ComputeGraphVar<T*>&                    dst,
                                 const ComputeGraphVar<Buffer3DView<T>>& src);

    /**********************************************************************************************
    * 
    * BufferView Copy: Device <- Host
    * 
    **********************************************************************************************/
    template <typename T>
    MUDA_HOST BufferLaunch& copy(VarView<T> dst, const T* src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(BufferView<T> dst, const T* src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(Buffer2DView<T> dst, const T* src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(Buffer3DView<T> dst, const T* src);

    template <typename T>
    MUDA_HOST BufferLaunch& copy(ComputeGraphVar<BufferView<T>>& dst,
                                 const ComputeGraphVar<T*>&      src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(ComputeGraphVar<Buffer2DView<T>>& dst,
                                 const ComputeGraphVar<T*>&        src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(ComputeGraphVar<VarView<T>>& dst,
                                 const ComputeGraphVar<T*>&   src);
    template <typename T>
    MUDA_HOST BufferLaunch& copy(ComputeGraphVar<Buffer3DView<T>>& dst,
                                 const ComputeGraphVar<T*>&        src);

    /**********************************************************************************************
    * 
    * BufferView Scatter: Device <- Host
    * 
    **********************************************************************************************/
    template <typename T>
    MUDA_HOST BufferLaunch& fill(VarView<T> buffer, const T& val);
    template <typename T>
    MUDA_HOST BufferLaunch& fill(BufferView<T> buffer, const T& val);
    template <typename T>
    MUDA_HOST BufferLaunch& fill(Buffer2DView<T> buffer, const T& val);
    template <typename T>
    MUDA_HOST BufferLaunch& fill(Buffer3DView<T> buffer, const T& val);

    template <typename T>
    MUDA_HOST BufferLaunch& fill(ComputeGraphVar<VarView<T>>& buffer,
                                 const ComputeGraphVar<T>&    val);
    template <typename T>
    MUDA_HOST BufferLaunch& fill(ComputeGraphVar<BufferView<T>>& buffer,
                                 const ComputeGraphVar<T>&       val);
    template <typename T>
    MUDA_HOST BufferLaunch& fill(ComputeGraphVar<Buffer2DView<T>>& buffer,
                                 const ComputeGraphVar<T>&         val);
    template <typename T>
    MUDA_HOST BufferLaunch& fill(ComputeGraphVar<Buffer3DView<T>>& buffer,
                                 const ComputeGraphVar<T>&         val);

  private:
    template <typename T, typename FConstruct>
    MUDA_HOST BufferLaunch& resize(DeviceBuffer<T>& buffer, size_t new_size, FConstruct&& fct);

    template <typename T, typename FConstruct>
    MUDA_HOST BufferLaunch& resize(DeviceBuffer2D<T>& buffer, Extent2D new_extent, FConstruct&& fct);

    template <typename T, typename FConstruct>
    MUDA_HOST BufferLaunch& resize(DeviceBuffer3D<T>& buffer, Extent3D new_extent, FConstruct&& fct);
};
}  // namespace muda

#include "details/buffer_launch.inl"