#include <uipc/geometry/geometry.h>
#include <Eigen/Geometry>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/zip.h>

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

S<IGeometry> IGeometry::clone() const
{
    return do_clone();
}

void IGeometry::collect_attribute_collections(vector<std::string>& names,
                                              vector<AttributeCollection*>& collections)
{
    do_collect_attribute_collections(names, collections);
}

void IGeometry::build_from_attribute_collections(span<std::string> names,
                                                 span<AttributeCollection*> collections) noexcept
{
    UIPC_ASSERT(names.size() == collections.size(),
                "The size of names and collections must be the same. names: {}, collections: {}",
                names.size(),
                collections.size());
    do_build_from_attribute_collections(names, collections);
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

void Geometry::do_build_from_attribute_collections(span<std::string> names,
                                                   span<AttributeCollection*> collections) noexcept
{
    m_meta.clear();
    m_intances.clear();

    for(auto&& [name, ac] : zip(names, collections))
    {
        if(name == "meta")
        {
            m_meta = *ac;
        }
        else if(name == "instances")
        {
            m_intances = *ac;
        }
    }
}

S<IGeometry> Geometry::do_clone() const
{
    return std::make_shared<Geometry>(*this);
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
