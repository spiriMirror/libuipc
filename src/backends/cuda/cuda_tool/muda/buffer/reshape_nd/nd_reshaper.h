#pragma once
#include <cuda.h>

namespace muda
{
template <typename T>
class DeviceBuffer;

template <typename T>
class DeviceBuffer2D;

template <typename T>
class DeviceBuffer3D;

class NDReshaper
{
  public:
    template <typename T, typename FConstruct>
    static MUDA_HOST void resize(int              grid_dim,
                                 int              block_dim,
                                 cudaStream_t     stream,
                                 DeviceBuffer<T>& buffer,
                                 size_t           new_size,
                                 FConstruct&&     fct);

    template <typename T>
    static MUDA_HOST void shrink_to_fit(int              grid_dim,
                                        int              block_dim,
                                        cudaStream_t     stream,
                                        DeviceBuffer<T>& buffer);

    template <typename T>
    static MUDA_HOST void reserve(int              grid_dim,
                                  int              block_dim,
                                  cudaStream_t     stream,
                                  DeviceBuffer<T>& buffer,
                                  size_t           new_capacity);


    //using T          = float;
    //using FConstruct = std::function<void(Buffer2DView<T>)>;
    template <typename T, typename FConstruct>
    static MUDA_HOST void resize(int                grid_dim,
                                 int                block_dim,
                                 cudaStream_t       stream,
                                 DeviceBuffer2D<T>& buffer,
                                 Extent2D           new_extent,
                                 FConstruct&&       fct);

    template <typename T>
    static MUDA_HOST void shrink_to_fit(int                grid_dim,
                                        int                block_dim,
                                        cudaStream_t       stream,
                                        DeviceBuffer2D<T>& buffer);

    template <typename T>
    static MUDA_HOST void reserve(int                grid_dim,
                                  int                block_dim,
                                  cudaStream_t       stream,
                                  DeviceBuffer2D<T>& buffer,
                                  Extent2D           new_capacity);

    template <typename T, typename FConstruct>
    static MUDA_HOST void resize(int                grid_dim,
                                 int                block_dim,
                                 cudaStream_t       stream,
                                 DeviceBuffer3D<T>& buffer,
                                 Extent3D           new_extent,
                                 FConstruct&&       fct);

    template <typename T>
    static MUDA_HOST void shrink_to_fit(int                grid_dim,
                                        int                block_dim,
                                        cudaStream_t       stream,
                                        DeviceBuffer3D<T>& buffer);

    template <typename T>
    static MUDA_HOST void reserve(int                grid_dim,
                                  int                block_dim,
                                  cudaStream_t       stream,
                                  DeviceBuffer3D<T>& buffer,
                                  Extent3D           new_capacity);
};
}  // namespace muda

#include "details/nd_reshaper.inl"