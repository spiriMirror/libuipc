#pragma once
#include <uipc/geometry/geometry.h>
namespace uipc::world
{
class IGeometryCollection
{
  public:
    virtual ~IGeometryCollection() = default;
    [[nodiscard]] SizeT size() const noexcept;
    void                clear() noexcept;
    void                resize(SizeT size) noexcept;
    void                reserve(SizeT size) noexcept;

  protected:
    virtual SizeT get_size() const noexcept       = 0;
    virtual void  do_clear() noexcept             = 0;
    virtual void  do_resize(SizeT size) noexcept  = 0;
    virtual void  do_reserve(SizeT size) noexcept = 0;
};

template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_const_v<GeometryT>)
class GeometryCollection : public IGeometryCollection
{
  public:
    GeometryCollection() = default;
    void                                push_back(const GeometryT& geometry);
    void                                push_back(const GeometryT&& geometry);
    span<GeometryT>                     view() noexcept;
    [[nodiscard]] span<const GeometryT> view() const noexcept;

  protected:
    virtual void  do_resize(SizeT size) noexcept override;
    virtual void  do_reserve(SizeT size) noexcept override;
    virtual void  do_clear() noexcept override;
    virtual SizeT get_size() const noexcept override;

  private:
    vector<GeometryT> m_geometries;
};
}  // namespace uipc::world

#include "details/geometry_collection.inl"