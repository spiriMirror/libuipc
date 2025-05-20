#include <uipc/core/object_snapshot.h>
#include <uipc/core/object.h>

namespace uipc::core
{
ObjectSnapshot::ObjectSnapshot(const Object& object)
    : m_id{object.id()}
    , m_name{object.name()}
    , m_geometries{object.m_geometry_ids}
{
}

void to_json(Json& j, const ObjectSnapshot& snapshot)
{
    j["id"]         = snapshot.m_id;
    j["name"]       = snapshot.m_name;
    j["geometries"] = snapshot.m_geometries;
}

void from_json(const Json& j, ObjectSnapshot& snapshot)
{
    snapshot.m_id         = j["id"].get<IndexT>();
    snapshot.m_name       = j["name"].get<std::string>();
    snapshot.m_geometries = j["geometries"].get<vector<IndexT>>();
}
}  // namespace uipc::core