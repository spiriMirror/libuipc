#pragma once
#include <uipc/world/object.h>

namespace uipc::world
{
class IObjectSlot
{
  public:
    virtual ~IObjectSlot() = default;
    [[nodiscard]] IndexT id() const noexcept;

  private:
    virtual IndexT get_id() const noexcept = 0;
};

class ObjectSlot : public IObjectSlot
{
  public:
    ObjectSlot(IndexT id, std::string_view name = "") noexcept;
    ObjectSlot(IndexT id, Object&& object) noexcept;

    Object::CGeometries geometries() const;
    Object::Geometries  geometries();


  protected:
    virtual [[nodiscard]] IndexT get_id() const noexcept;

  private:
    IndexT m_id;
    Object m_object;
};
}  // namespace uipc::world
