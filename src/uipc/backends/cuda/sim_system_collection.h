#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/unordered_map.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/format.h>
namespace uipc::backend::cuda
{
class SimSystem;

class SimSystemCollection
{
    friend struct fmt::formatter<SimSystemCollection>;
    friend class SimEngine;
  public:
    void create(U<SimSystem> system);
    template <std::derived_from<SimSystem> T>
    T* find();

  private:
    unordered_map<U64, U<SimSystem>> m_sim_systems;
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