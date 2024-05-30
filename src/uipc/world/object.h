#pragma once
#include <string_view>
#include <uipc/common/vector.h>
#include <uipc/geometry/geometry.h>
#include <uipc/common/unordered_map.h>
#include <uipc/world/geometry_collection.h>

namespace uipc::world
{
class IObject
{
  public:
    virtual ~IObject() = default;
    [[nodiscard]] std::string_view name() const noexcept;

  protected:
    virtual std::string_view get_name() const noexcept = 0;
};

class Object;

template <std::derived_from<geometry::Geometry> GeometryT>
class ObjectGeometries
{
  private:
    using CollectionT = GeometryCollection<std::remove_const_t<GeometryT>>;
    static constexpr bool IsConst = std::is_const_v<GeometryT>;

    using AutoCollectionT = std::conditional_t<IsConst, const CollectionT, CollectionT>;
    using AutoGeometryT = std::conditional_t<IsConst, const GeometryT, GeometryT>;

    friend class Object;
    AutoCollectionT& m_geometries;
    AutoCollectionT& m_rest_geometries;
    ObjectGeometries(AutoCollectionT& geometries, AutoCollectionT& rest_geometries) noexcept;

  public:
    class Geometries
    {
      public:
        [[nodiscard]] span<AutoGeometryT> view() && noexcept;

      private:
        friend class ObjectGeometries;
        Geometries(AutoCollectionT& geometries) noexcept;

        AutoCollectionT& m_geometries;
    };

    [[nodiscard]] Geometries geometries() noexcept;
    [[nodiscard]] Geometries rest_geometries() noexcept;
};

class Object : public IObject
{
  public:
    Object(std::string_view name = "") noexcept;

    template <std::derived_from<geometry::Geometry> GeometryT>
        requires(!std::is_abstract_v<GeometryT>)
    ObjectGeometries<GeometryT> find();

    template <std::derived_from<geometry::Geometry> GeometryT>
        requires(!std::is_abstract_v<GeometryT>)
    ObjectGeometries<const GeometryT> find() const;

    template <std::derived_from<geometry::Geometry> GeometryT>
        requires(!std::is_abstract_v<GeometryT>)
    ObjectGeometries<GeometryT> push_back(const GeometryT& geometry);

    template <std::derived_from<geometry::Geometry> GeometryT>
        requires(!std::is_abstract_v<GeometryT>)
    ObjectGeometries<GeometryT> push_back(const GeometryT& rest_geometry,
                                          const GeometryT& geometry);

  protected:
    [[nodiscard]] std::string_view get_name() const noexcept override;

  private:
    template <std::derived_from<geometry::Geometry> GeometryT>
        requires(!std::is_abstract_v<GeometryT>)
    U64 find_or_create() const;

    std::string m_name;

    mutable unordered_map<std::size_t, U64>     m_uid_to_index;
    mutable std::vector<U<IGeometryCollection>> m_geometry;
    mutable std::vector<U<IGeometryCollection>> m_rest_geometry;
};
}  // namespace uipc::world


#include "details/object.inl"