#pragma once
#include <uipc/common/list.h>
#include <uipc/common/unordered_map.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/format.h>
#include <uipc/common/vector.h>
#include <uipc/common/span.h>
#include <backends/common/i_sim_system.h>

namespace uipc::backend
{
class SimSystemCollection
{
    friend struct fmt::formatter<SimSystemCollection>;

  public:
    Json                    to_json() const;
    span<ISimSystem* const> systems() const;

    void create(U<ISimSystem> system);
    void build_systems();

    class QueryOptions
    {
      public:
        /**
         * @brief Indicates whether the query should be exact or compatible.
         * 
         * 1) true, the query will return first exact system.
         * 2) false, the query will return first exact system, if not found, it will return first compatible system.
         */
        bool exact = true;
    };
    template <std::derived_from<ISimSystem> T>
    T* find(const QueryOptions& options = {.exact = true});

  private:
    mutable bool                           built = false;
    unordered_map<uint64_t, U<ISimSystem>> m_sim_system_map;
    vector<ISimSystem*>                    m_valid_systems;
    list<U<ISimSystem>>                    m_invalid_systems;

    void cleanup_invalid_systems();
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