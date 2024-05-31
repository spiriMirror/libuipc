#pragma once
#include <uipc/common/unordered_map.h>
#include <uipc/world/object_slot.h>
#include <uipc/common/set.h>
namespace uipc::world
{
class IObjectCollection
{
  public:
    virtual ~IObjectCollection() = default;
};

class ObjectCollection : public IObjectCollection
{
    friend class Scene;

  public:
    ObjectCollection() = default;
    P<ObjectSlot> emplace(Object&& object);
    P<ObjectSlot> pending_emplace(Object&& object);

    P<ObjectSlot>       find(IndexT id) noexcept;
    P<const ObjectSlot> find(IndexT id) const noexcept;

    void destroy(IndexT id) noexcept;
    void pending_destroy(IndexT id) noexcept;

    void  reserve(SizeT size) noexcept;
    SizeT size() const noexcept;

    void solve_pending() noexcept;

  private:
    IndexT m_current_id = 0;

    unordered_map<IndexT, S<ObjectSlot>> m_objects;
    unordered_map<IndexT, S<ObjectSlot>> m_pending_create;
    set<IndexT>                          m_pending_destroy;
};
}  // namespace uipc::world
