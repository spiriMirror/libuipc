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
    IndexT create(const Geometry& geo, bool evolving_only = false);

    /**
     * @brief Find the geometry slot by id
     * 
     * Only return const version, not allow to modify the geometry.
     */
    S<const GeometrySlot> find(IndexT id) const;

    SizeT geometry_count() const noexcept;

    /**
     * @brief Create a named AttributeCollection in the atlas
     */
    void create(std::string_view name, const AttributeCollection& ac, bool evolving_only = false);

    /**
     * @brief Find the AttributeCollection by name
     * 
     * Only return const version, not allow to modify the geometry.
     */
    S<const AttributeCollection> find(std::string_view name) const;

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

class UIPC_CORE_API GeometryAtlasCommit
{
    class Impl;

  public:
    GeometryAtlasCommit();
    ~GeometryAtlasCommit();

    IndexT                  create(const GeometryCommit& geo_commit);
    S<const GeometryCommit> find(IndexT id) const;
    void create(std::string_view name, const AttributeCollectionCommit& ac_commit);
    S<const AttributeCollectionCommit> find(std::string_view name) const;
    vector<std::string> attribute_collection_names() const noexcept;
    SizeT               attribute_collection_count() const noexcept;
    SizeT               geometry_count() const noexcept;

    Json to_json() const;

    void from_json(const Json& j);


    U<Impl> m_impl;
};
}  // namespace uipc::geometry
