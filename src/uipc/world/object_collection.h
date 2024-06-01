#pragma once
#include <uipc/common/unordered_map.h>
#include <uipc/world/object.h>
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
    P<Object> emplace(Object&& object);

    P<Object>       find(IndexT id) noexcept;
    P<const Object> find(IndexT id) const noexcept;

    void destroy(IndexT id) noexcept;

    void  reserve(SizeT size) noexcept;
    SizeT size() const noexcept;

  private:
    IndexT                           m_next_id = 0;
    unordered_map<IndexT, S<Object>> m_objects;
};
}  // namespace uipc::world
