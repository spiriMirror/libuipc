#pragma once
#include <uipc/common/unordered_map.h>
#include <uipc/world/object.h>
#include <uipc/common/set.h>
namespace uipc::world
{
class UIPC_CORE_API IObjectCollection
{
  public:
    virtual ~IObjectCollection() = default;
};

class UIPC_CORE_API ObjectCollection : public IObjectCollection
{
    friend class Scene;

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
    IndexT                           m_next_id = 0;
    unordered_map<IndexT, S<Object>> m_objects;
};
}  // namespace uipc::world
