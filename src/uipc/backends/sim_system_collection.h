#pragma once
#include <uipc/common/list.h>
#include <uipc/common/unordered_map.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/format.h>
#include <uipc/backends/i_sim_system.h>

namespace uipc::backend
{
class SimSystemCollection
{
    friend struct fmt::formatter<SimSystemCollection>;

  public:
    Json to_json() const;

  private:
    friend class SimEngine;
    friend class SimSystem;
    void create(U<ISimSystem> system);
    template <std::derived_from<ISimSystem> T>
    T* find();

    unordered_map<uint64_t, U<ISimSystem>> m_sim_systems;
    list<U<ISimSystem>>                    m_invalid_systems;

    void cleanup_invalid_systems();
    void build_systems();
};
}  // namespace uipc::backend

namespace fmt
{
template <>
struct formatter<uipc::backend::SimSystemCollection> : public formatter<string_view>
{
    appender format(const uipc::backend::SimSystemCollection& s, format_context& ctx) const;
};
}  // namespace fmt
#include "details/sim_system_collection.inl"