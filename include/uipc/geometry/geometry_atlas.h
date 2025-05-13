#pragma once
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/geometry_slot.h>
#include <uipc/geometry/geometry_collection.h>

namespace uipc::geometry
{
class UIPC_CORE_API GeometryAtlas
{
    class Impl;

  public:
    GeometryAtlas();
    ~GeometryAtlas();

    /**
     * @brief Create a geometry in the atlas
     */
    IndexT create(const Geometry& geo);

    /**
     * @brief Find the geometry slot by id
     * 
     * Only return const version, not allow to modify the geometry.
     */
    const GeometrySlot* find(IndexT id) const;

    SizeT geometry_count() const noexcept;

    /**
     * @brief Create a named AttributeCollection in the atlas
     */
    void create(std::string_view name, const AttributeCollection& ac);

    /**
     * @brief Find the AttributeCollection by name
     * 
     * Only return const version, not allow to modify the geometry.
     */
    const AttributeCollection* find(std::string_view name) const;

    SizeT attribute_collection_count() const noexcept;

    vector<std::string> attribute_collection_names() const noexcept;

    /**
     * @brief Create json representation of the geometry atlas
     */
    Json to_json() const;

    /**
     * @brief Create geometry atlas from json
     * 
     * @param j The json representation of the geometry atlas
     * @param attributes The attributes to be used in the geometry
     */
    void from_json(const Json& j);

  private:
    U<Impl> m_impl;
};
}  // namespace uipc::geometry
