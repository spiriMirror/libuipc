#pragma once
#include <uipc/common/unordered_map.h>
#include <uipc/core/object.h>
#include <uipc/core/object_snapshot.h>
#include <uipc/common/set.h>

namespace uipc::core::internal
{
class Scene;
}

namespace uipc::core
{
class ObjectCollectionSnapshot;

class UIPC_CORE_API IObjectCollection
{
  public:
    virtual ~IObjectCollection() = default;
};

class UIPC_CORE_API ObjectCollection : public IObjectCollection
{
    friend class Scene;
    friend class internal::Scene;
    friend class SceneFactory;
    friend class SceneSnapshot;
    friend class SceneSnapshotCommit;
    friend class ObjectCollectionSnapshot;
    friend struct fmt::formatter<ObjectCollection>;

  public:
    ObjectCollection() = default;
    S<Object> emplace(Object&& object);

    S<Object>               find(IndexT id) noexcept;
    S<const Object>         find(IndexT id) const noexcept;
    vector<S<Object>>       find(std::string_view name) noexcept;
    vector<S<const Object>> find(std::string_view name) const noexcept;

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
    void update_from(internal::Scene& scene, const ObjectCollectionSnapshot& snapshot) noexcept;
};

class UIPC_CORE_API ObjectCollectionSnapshot
{
    friend class SceneFactory;
    friend class ObjectCollection;
    friend UIPC_CORE_API void to_json(Json& j, const ObjectCollectionSnapshot& obj);
    friend UIPC_CORE_API void from_json(const Json& j, ObjectCollectionSnapshot& obj);

  public:
    ObjectCollectionSnapshot() = default;
    ObjectCollectionSnapshot(const ObjectCollection& dst);

  private:
    unordered_map<IndexT, ObjectSnapshot> m_objects;
    IndexT                                m_next_id = 0;
};

void UIPC_CORE_API to_json(Json& j, const ObjectCollectionSnapshot& obj);
void UIPC_CORE_API from_json(const Json& j, ObjectCollectionSnapshot& obj);
}  // namespace uipc::core

namespace fmt
{
template <>
struct UIPC_CORE_API formatter<uipc::core::ObjectCollection> : formatter<string_view>
{
    appender format(const uipc::core::ObjectCollection& c, format_context& ctx) const;
};
}  // namespace fmt
