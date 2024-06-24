#pragma once
#include <type_define.h>
#include <uipc/common/unordered_map.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/format.h>
#include <i_sim_system.h>

namespace uipc::backend::cuda
{
class SimSystemCollection
{
    friend struct fmt::formatter<SimSystemCollection>;
    friend class SimEngine;

  public:
    void create(U<ISimSystem> system);
    template <std::derived_from<ISimSystem> T>
    T*   find();
    Json to_json() const;

  private:
    unordered_map<U64, U<ISimSystem>> m_sim_systems;
};
}  // namespace uipc::backend::cuda

namespace fmt
{
template <>
struct formatter<uipc::backend::cuda::SimSystemCollection> : public formatter<string_view>
{
    appender format(const uipc::backend::cuda::SimSystemCollection& s,
                    format_context&                                 ctx) const;
};
}  // namespace fmt
#include "details/sim_system_collection.inl"