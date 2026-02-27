/*****************************************************************/ /**
 * \file   launch.h
 * \brief  cuda kernel launch in muda style.
 * 
 * \author MuGdxy
 * \date   January 2024
 *********************************************************************/

#pragma once
#include <muda/launch/launch_base.h>
#include <muda/type_traits/always.h>
#include <muda/launch/kernel_tag.h>
namespace muda
{
namespace details
{
    template <typename F>
    struct LaunchCallable
    {
        F    callable;
        dim3 dim;
        template <typename U>
        LaunchCallable(U&& f, const dim3& d)
            : callable(std::forward<U>(f))
            , dim(d)
        {
        }
    };

    template <typename F, typename UserTag = DefaultTag>
    MUDA_GLOBAL void generic_kernel(LaunchCallable<F> f);

    template <typename F, typename UserTag = DefaultTag>
    MUDA_GLOBAL void generic_kernel_with_range(LaunchCallable<F> f);
}  // namespace details

// using details::generic_kernel;

dim3 cube(int x) MUDA_NOEXCEPT;
dim3 square(int x) MUDA_NOEXCEPT;

/** 
 * \class Launch
 * 
 * \ingroup Launcher
 * 
 * \brief **A wrapper of raw cuda kernel launch in muda style**, removing the `<<<>>>` usage, for better intellisense support.
 * 
 * \details
 * A raw cuda kernel define and launch:
 * \code{.cpp}
 *  __global__ void cuda_kernel() {}
 * 
 *  int main()
 *  {
 *      cuda_kernel<<<4,64>>>();
 *  }
 * \endcode
 * 
 * The muda style kernel launch:
 * \code{.cpp}
 *  // muda kernel launch
 *  Launch(4,64)
 *      .kernel_name("kernel_name") // optional
 *      .apply([]__device__(){}); // kernel body
 * \endcode
 * 
 * A more complicated but more convincing example, to show why
 * using muda style kernel launch is better than raw cuda kernel launch.
 * \code{.cpp}
 *  DeviceBuffer3D<float> volume{10,10,10};
 *  Launch(dim3{8,8,8}) // blockDim
 *      .kernel_name("write_volume") // optional, for better debug info
 *      .apply(volume.extent(), 
 *          [
 *              volume = volume.viewer().name("volume") // name is optional, for better debug info
 *          ] __device__(int3 xyz) mutable
 *          {
 *              volume(xyz) = 1.0f;
 *          });
 * \endcode
 * 
 * \sa \ref device_buffer_3d.h \ref parallel_for.h
 */
class Launch : public LaunchBase<Launch>
{
    dim3   m_grid_dim;
    dim3   m_block_dim;
    size_t m_shared_mem_size;

  public:
    template <typename F>
    using NodeParms = KernelNodeParms<details::LaunchCallable<raw_type_t<F>>>;

    MUDA_HOST Launch(dim3 gridDim, dim3 blockDim, size_t sharedMemSize = 0, cudaStream_t stream = nullptr) MUDA_NOEXCEPT
        : LaunchBase(stream),
          m_grid_dim(gridDim),
          m_block_dim(blockDim),
          m_shared_mem_size(sharedMemSize)
    {
    }

    MUDA_HOST Launch(int          gridDim       = 1,
                     int          blockDim      = 1,
                     size_t       sharedMemSize = 0,
                     cudaStream_t stream        = nullptr) MUDA_NOEXCEPT
        : LaunchBase(stream),
          m_grid_dim(gridDim),
          m_block_dim(blockDim),
          m_shared_mem_size(sharedMemSize)
    {
    }

    MUDA_HOST Launch(dim3 blockDim, size_t sharedMemSize = 0, cudaStream_t stream = nullptr) MUDA_NOEXCEPT
        : LaunchBase(stream),
          m_grid_dim(0),
          m_block_dim(blockDim),
          m_shared_mem_size(sharedMemSize)
    {
    }

    template <typename F, typename UserTag = Default>
    MUDA_HOST Launch& apply(F&& f);
    template <typename F, typename UserTag = Default>
    MUDA_HOST Launch& apply(F&& f, Tag<UserTag>);

    template <typename F, typename UserTag = Default>
    MUDA_HOST Launch& apply(const dim3& active_dim, F&& f);

    template <typename F, typename UserTag = Default>
    MUDA_HOST Launch& apply(const dim3& active_dim, F&& f, Tag<UserTag>);

    template <typename F, typename UserTag = Default>
    MUDA_HOST MUDA_NODISCARD auto as_node_parms(F&& f) -> S<NodeParms<F>>;

    template <typename F, typename UserTag = Default>
    MUDA_HOST MUDA_NODISCARD auto as_node_parms(F&& f, Tag<UserTag>)
        -> S<NodeParms<F>>;

    template <typename F, typename UserTag = Default>
    MUDA_HOST MUDA_NODISCARD auto as_node_parms(const dim3& active_dim, F&& f)
        -> S<NodeParms<F>>;

    template <typename F, typename UserTag = Default>
    MUDA_HOST MUDA_NODISCARD auto as_node_parms(const dim3& active_dim, F&& f, Tag<UserTag>)
        -> S<NodeParms<F>>;


  private:
    template <typename F, typename UserTag = Default>
    MUDA_HOST void invoke(F&& f);

    template <typename F, typename UserTag = Default>
    MUDA_HOST void invoke(const dim3& active_dim, F&& f);

    MUDA_GENERIC dim3 calculate_grid_dim(const dim3& active_dim) const MUDA_NOEXCEPT;

    MUDA_GENERIC void check_input_with_range() const MUDA_NOEXCEPT;

    MUDA_GENERIC void check_input() const MUDA_NOEXCEPT;
};
}  // namespace muda

#include "details/launch.inl"