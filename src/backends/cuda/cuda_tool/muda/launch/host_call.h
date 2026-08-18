#pragma once
#include <muda/launch/launch_base.h>

namespace muda
{
namespace details
{
    template <typename F, typename UserTag>
    MUDA_HOST void CUDARTAPI generic_host_call(void* userdata)
    {
        auto f = reinterpret_cast<F*>(userdata);
        (*f)();
    }

    template <typename F, typename UserTag>
    MUDA_HOST void CUDARTAPI delete_function_object(void* userdata)
    {
        auto f = reinterpret_cast<F*>(userdata);
        delete f;
    }
}  // namespace details


class HostCall : public LaunchBase<HostCall>
{
  public:
    MUDA_HOST HostCall(cudaStream_t stream = nullptr)
        : LaunchBase(stream)
    {
    }

    template <typename F, typename UserTag = DefaultTag>
    MUDA_HOST HostCall& apply(F&& f, UserTag tag = {})
    {
        MUDA_ASSERT(ComputeGraphBuilder::is_phase_none(),
                    "HostCall must be can't appear in a compute graph");
        using CallableType = raw_type_t<F>;
        static_assert(std::is_invocable_v<CallableType>, "f:void (void)");
        auto userdata = new CallableType(std::forward<F>(f));
        checkCudaErrors(cudaLaunchHostFunc(
            this->stream(), details::generic_host_call<CallableType, UserTag>, userdata));
        checkCudaErrors(cudaLaunchHostFunc(
            this->stream(), details::delete_function_object<CallableType, UserTag>, userdata));
        return *this;
    }

    /// <summary>
    ///
    /// </summary>
    /// <typeparam name="F"></typeparam>
    /// <typeparam name="UserTag"></typeparam>
    /// <param name="f"></param>
    /// <param name="tag"></param>
    /// <returns></returns>
    template <typename F, typename UserTag = DefaultTag>
    MUDA_NODISCARD MUDA_HOST auto as_node_parms(F&& f, UserTag tag = {})
    {
        using CallableType = raw_type_t<F>;
        auto parms = std::make_shared<HostNodeParms<CallableType>>(std::forward<F>(f));
        parms->fn((cudaHostFn_t)details::generic_host_call<CallableType, UserTag>);
        return parms;
    }
};
}  // namespace muda