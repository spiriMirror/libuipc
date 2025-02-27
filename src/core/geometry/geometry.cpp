#include <uipc/geometry/geometry.h>
#include <Eigen/Geometry>
#include <uipc/builtin/attribute_name.h>

namespace uipc::geometry
{
std::string_view IGeometry::type() const noexcept
{
    return get_type();
}

Json IGeometry::to_json() const
{
    return do_to_json();
}

void IGeometry::collect_attribute_collections(vector<std::string>& names,
                                              vector<AttributeCollection*>& collections)
{
    do_collect_attribute_collections(names, collections);
}

Geometry::Geometry()
{
    m_meta.resize(1);      // only one meta for one geometries
    m_intances.resize(1);  // default only one instance
}

auto Geometry::meta() -> MetaAttributes
{
    return MetaAttributes{m_meta};
}

auto Geometry::meta() const -> CMetaAttributes
{
    return CMetaAttributes{m_meta};
}

auto Geometry::instances() -> InstanceAttributes
{
    return InstanceAttributes{m_intances};
}

auto Geometry::instances() const -> CInstanceAttributes
{
    return CInstanceAttributes{m_intances};
}

Json Geometry::do_to_json() const
{
    Json j;
    j["meta"]      = m_meta.to_json();
    j["instances"] = m_intances.to_json();
    return j;
}
void Geometry::do_collect_attribute_collections(vector<std::string>& names,
                                                vector<AttributeCollection*>& collections)
{
    names.push_back("meta");
    collections.push_back(&m_meta);

    names.push_back("instances");
    collections.push_back(&m_intances);
}
}  // namespace uipc::geometry

namespace fmt
{
appender formatter<uipc::geometry::Geometry>::format(const uipc::geometry::Geometry& geo,
                                                     format_context& ctx) const
{
    return fmt::format_to(ctx.out(),
                          R"(type:<{}>;
meta:{};
instances({}):{};)",
                          geo.type(),
                          geo.meta(),
                          geo.instances().size(),
                          geo.instances());
}
}  // namespace fmt
