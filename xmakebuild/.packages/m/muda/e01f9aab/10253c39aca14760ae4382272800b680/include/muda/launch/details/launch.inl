#include <muda/compute_graph/compute_graph_builder.h>
#include <muda/compute_graph/nodes/compute_graph_kernel_node.h>
#include <muda/launch/kernel.h>
namespace muda
{
namespace details
{
    template <typename F, typename UserTag>
    MUDA_GLOBAL void generic_kernel(LaunchCallable<F> f)
    {
        static_assert(std::is_invocable_v<F>, "f:void (void)");
        f.callable();
    }

    template <typename F, typename UserTag>
    MUDA_GLOBAL void generic_kernel_with_range(LaunchCallable<F> f)
    {
        auto x = blockIdx.x * blockDim.x + threadIdx.x;
        auto y = blockIdx.y * blockDim.y + threadIdx.y;
        auto z = blockIdx.z * blockDim.z + threadIdx.z;

        if(x < f.dim.x && y < f.dim.y && z < f.dim.z)
        {
            if constexpr(std::is_invocable_v<F, int2>)
            {
                f.callable(int2{static_cast<int>(x), static_cast<int>(y)});
            }
            else if constexpr(std::is_invocable_v<F, int3>)
            {
                f.callable(int3{static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)});
            }
            else if constexpr(std::is_invocable_v<F, uint1>)
            {
                f.callable(uint1{x});
            }
            else if constexpr(std::is_invocable_v<F, uint2>)
            {
                f.callable(uint2{x, y});
            }
            else if constexpr(std::is_invocable_v<F, uint3>)
            {
                f.callable(uint3{x, y, z});
            }
            else if constexpr(std::is_invocable_v<F, dim3>)
            {
                f.callable(dim3{x, y, z});
            }
            else if constexpr(std::is_invocable_v<F, int>
                              || std::is_invocable_v<F, unsigned int>)
            {
                static_assert("You should use `ParallelFor()` instead of `Launch()` for better semantics");
            }
            else
            {
                static_assert(always_false_v<F>,
                              "invalid callable, it should be:"
                              "void (uint1) or"
                              "void (unsigned int) or"
                              "void (uint2) or"
                              "void (uint3) or"
                              "void (dim3)");
            }
        }
    }
}  // namespace details

MUDA_INLINE dim3 cube(int x) MUDA_NOEXCEPT
{
    return dim3(x, x, x);
}

MUDA_INLINE dim3 square(int x) MUDA_NOEXCEPT
{
    return dim3(x, x, 1);
}

template <typename F, typename UserTag>
MUDA_INLINE MUDA_HOST auto Launch::as_node_parms(F&& f) -> S<NodeParms<F>>
{
    check_input();

    using CallableType = raw_type_t<F>;
    auto parms = std::make_shared<NodeParms<F>>(std::forward<F>(f), dim3{0});

    parms->func((void*)details::generic_kernel<CallableType, UserTag>);
    parms->grid_dim(m_grid_dim);
    parms->block_dim(m_block_dim);
    parms->shared_mem_bytes(static_cast<uint32_t>(m_shared_mem_size));
    parms->parse([](details::LaunchCallable<CallableType>& p) -> std::vector<void*>
                 { return {&p}; });
    return parms;
}

template <typename F, typename UserTag>
MUDA_HOST MUDA_NODISCARD auto Launch::as_node_parms(F&& f, Tag<UserTag>)
    -> S<NodeParms<F>>
{
    return as_node_parms<F, UserTag>(std::forward<F>(f));
}

template <typename F, typename UserTag>
MUDA_INLINE MUDA_HOST auto Launch::as_node_parms(const dim3& active_dim, F&& f)
    -> S<NodeParms<F>>
{
    check_input_with_range();

    auto grid_dim = calculate_grid_dim(active_dim);

    using CallableType = raw_type_t<F>;
    auto parms = std::make_shared<NodeParms<F>>(std::forward<F>(f), active_dim);

    parms->func((void*)details::generic_kernel_with_range<CallableType, UserTag>);
    parms->grid_dim(grid_dim);
    parms->block_dim(m_block_dim);
    parms->shared_mem_bytes(m_shared_mem_size);
    parms->parse([](details::LaunchCallable<CallableType>& p) -> std::vector<void*>
                 { return {&p}; });
    return parms;
}

template <typename F, typename UserTag>
MUDA_HOST MUDA_NODISCARD auto Launch::as_node_parms(const dim3& active_dim, F&& f, Tag<UserTag>)
    -> S<NodeParms<F>>
{
    return as_node_parms<F, UserTag>(active_dim, std::forward<F>(f));
}

template <typename F, typename UserTag>
MUDA_HOST void Launch::invoke(F&& f)
{
    check_input();

    using CallableType = raw_type_t<F>;
    auto callable = details::LaunchCallable<CallableType>{std::forward<F>(f), dim3{0}};
    details::generic_kernel<CallableType, UserTag>
        <<<m_grid_dim, m_block_dim, m_shared_mem_size, m_stream>>>(callable);
}

template <typename F, typename UserTag>
MUDA_HOST void Launch::invoke(const dim3& active_dim, F&& f)
{
    check_input_with_range();

    dim3 grid_dim = calculate_grid_dim(active_dim);

    using CallableType = raw_type_t<F>;
    auto callable = details::LaunchCallable<CallableType>{std::forward<F>(f), active_dim};
    details::generic_kernel_with_range<CallableType, UserTag>
        <<<grid_dim, m_block_dim, m_shared_mem_size, m_stream>>>(callable);
}

template <typename F, typename UserTag>
MUDA_HOST Launch& Launch::apply(F&& f)
{
    if constexpr(COMPUTE_GRAPH_ON)
    {
        using CallableType = raw_type_t<F>;
        ComputeGraphBuilder::invoke_phase_actions(
            [&] { invoke<F, UserTag>(std::forward<F>(f)); },
            [&]
            {
                auto parms = this->as_node_parms<F, UserTag>(std::forward<F>(f));
                details::ComputeGraphAccessor().set_kernel_node(parms);
            },
            [&]
            {
                details::ComputeGraphAccessor().set_kernel_node<KernelNodeParms<CallableType>>(nullptr);
            });
    }
    else
    {
        invoke<F, UserTag>(std::forward<F>(f));
    }
    pop_kernel_label();
    return *this;
}

template <typename F, typename UserTag>
MUDA_HOST Launch& Launch::apply(F&& f, Tag<UserTag>)
{
    return apply<F, UserTag>(std::forward<F>(f));
}
template <typename F, typename UserTag>
MUDA_HOST Launch& muda::Launch::apply(const dim3& active_dim, F&& f)
{
    if constexpr(COMPUTE_GRAPH_ON)
    {
        using CallableType = raw_type_t<F>;
        ComputeGraphBuilder::invoke_phase_actions(
            [&] { invoke<F, UserTag>(active_dim, std::forward<F>(f)); },
            [&]
            {
                auto parms =
                    this->as_node_parms<F, UserTag>(active_dim, std::forward<F>(f));
                details::ComputeGraphAccessor().set_kernel_node(parms);
            },
            [&]
            {
                details::ComputeGraphAccessor().set_kernel_node<KernelNodeParms<CallableType>>(nullptr);
            });
    }
    else
    {
        invoke<F, UserTag>(active_dim, std::forward<F>(f));
    }
    pop_kernel_label();

    return *this;
}

template <typename F, typename UserTag>
MUDA_HOST Launch& Launch::apply(const dim3& active_dim, F&& f, Tag<UserTag>)
{
    return apply<F, UserTag>(active_dim, std::forward<F>(f));
}

MUDA_INLINE MUDA_GENERIC dim3 Launch::calculate_grid_dim(const dim3& active_dim) const MUDA_NOEXCEPT
{
    dim3 ret;

    ret.x = (active_dim.x + m_block_dim.x - 1) / m_block_dim.x;
    ret.y = (active_dim.y + m_block_dim.y - 1) / m_block_dim.y;
    ret.z = (active_dim.z + m_block_dim.z - 1) / m_block_dim.z;
    return ret;
}

MUDA_INLINE MUDA_GENERIC void Launch::check_input_with_range() const MUDA_NOEXCEPT
{
    MUDA_ASSERT(m_grid_dim.x == 0, "grid_dim should be `dim3{0}`");
}

MUDA_INLINE MUDA_GENERIC void Launch::check_input() const MUDA_NOEXCEPT
{
    MUDA_ASSERT(m_grid_dim.x > 0 && m_grid_dim.y > 0 && m_grid_dim.z > 0,
                "grid_dim should be non-zero");
}
}  // namespace muda