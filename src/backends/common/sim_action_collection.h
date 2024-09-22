#pragma once
#include <backends/common/sim_action.h>
#include <uipc/common/span.h>
#include <uipc/common/list.h>
#include <uipc/common/vector.h>
#include <uipc/common/log.h>

namespace uipc::backend
{
template <typename F = void()>
class SimActionCollection
{
  public:
    using SimActionT = SimAction<F>;

    void register_action(SimActionT&& action);

    template <typename Callable>
    void register_action(ISimSystem& system, Callable&& f);


    span<SimActionT>       view() noexcept;
    span<const SimActionT> view() const noexcept;

  private:
    void                       lazy_init() const;
    mutable list<SimActionT>   m_action_buffer;
    mutable vector<SimActionT> m_actions;
    mutable bool               built = false;
};
}  // namespace uipc::backend

#include "details/sim_action_collection.inl"