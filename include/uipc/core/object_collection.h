#pragma once
#include <uipc/common/unordered_map.h>
#include <uipc/core/object.h>
#include <uipc/common/set.h>
namespace uipc::core
{
class UIPC_CORE_API IObjectCollection
{
  public:
    virtual ~IObjectCollection() = default;
};

class UIPC_CORE_API ObjectCollection : public IObjectCollection
{
    friend class Scene;
    friend class SceneFactory;
    friend class SceneSnapshot;
    friend class SceneSnapshotCommit;
    friend struct fmt::formatter<ObjectCollection>;

  public:
    ObjectCollection() = default;
    S<Object> emplace(Object&& object);

    S<Object>       find(IndexT id) noexcept;
    S<const Object> find(IndexT id) const noexcept;

    void destroy(IndexT id) noexcept;

    void   reserve(SizeT size) noexcept;
    SizeT  size() const noexcept;
    IndexT next_id() const noexcept;

  private:
    mutable IndexT                   m_next_id = 0;
    unordered_map<IndexT, S<Object>> m_objects;

    unordered_map<IndexT, S<Object>>&       objects();
    const unordered_map<IndexT, S<Object>>& objects() const;

    void build_from(span<S<Object>> objects) noexcept;
};
}  // namespace uipc::core

namespace fmt
{
template <>
struct UIPC_CORE_API formatter<uipc::core::ObjectCollection> : formatter<string_view>
{
    appender format(const uipc::core::ObjectCollection& c, format_context& ctx) const;
};
}  // namespace fmt
