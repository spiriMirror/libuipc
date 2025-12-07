#include <uipc/geometry/geometry.h>
#include <uipc/common/log.h>
#include <Eigen/Geometry>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/zip.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/geometry/geometry_commit.h>

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
                                              vector<const AttributeCollection*>& collections) const
{
    do_collect_attribute_collections(names, collections);
}

void IGeometry::collect_attribute_collections(vector<std::string>& names,
                                              vector<AttributeCollection*>& collections)
{
    do_collect_attribute_collections(names, collections);
}

void IGeometry::build_from_attribute_collections(span<const std::string> names,
                                                 span<const AttributeCollection*> collections)
{
    do_build_from_attribute_collections(names, collections);
}

void IGeometry::update_from(const GeometryCommit& commit)
{
    do_update_from(commit);
}

Geometry::Geometry()
{
    m_meta = create("meta");
    m_meta->resize(1);
    m_intances = create("instances");
    m_intances->resize(1);
}

Geometry::Geometry(const Geometry& o)
{
    for(auto&& [name, ac] : o.m_attribute_collections)
    {
        m_attribute_collections[name] = uipc::make_shared<AttributeCollection>(*ac);
    }
    m_meta     = find("meta");
    m_intances = find("instances");
    UIPC_ASSERT(m_meta && m_intances,
                "Meta and instances attribute collections should be created in the constructor");
}

auto Geometry::meta() -> MetaAttributes
{

    return MetaAttributes{*m_meta};
}

auto Geometry::meta() const -> CMetaAttributes
{
    return CMetaAttributes{*m_meta};
}

auto Geometry::instances() -> InstanceAttributes
{
    return InstanceAttributes{*m_intances};
}

auto Geometry::instances() const -> CInstanceAttributes
{
    return CInstanceAttributes{*m_intances};
}

S<AttributeCollection> Geometry::operator[](std::string_view name)
{
    auto ac = find(name);
    if(!ac)
        ac = create(name);
    return ac;
}

S<const AttributeCollection> Geometry::operator[](std::string_view name) const
{
    return find(name);
}

S<AttributeCollection> Geometry::create(std::string_view name)
{
    auto s_name = std::string{name};
    UIPC_ASSERT(!m_attribute_collections.contains(s_name),
                "Attribute collection with name '{}' already exists.",
                s_name);
    auto ret                        = uipc::make_shared<AttributeCollection>();
    m_attribute_collections[s_name] = ret;
    return ret;
}

S<const AttributeCollection> Geometry::find(std::string_view name) const
{
    auto it = m_attribute_collections.find(std::string{name});
    if(it == m_attribute_collections.end())
        return nullptr;
    return it->second;
}

S<AttributeCollection> Geometry::find(std::string_view name)
{
    auto it = m_attribute_collections.find(std::string{name});
    if(it == m_attribute_collections.end())
        return nullptr;
    return it->second;
}

Json Geometry::do_to_json() const
{
    Json j = Json::object();
    for(auto&& [name, ac] : m_attribute_collections)
    {
        j[name] = ac->to_json();
    }
    return j;
}

void Geometry::do_collect_attribute_collections(vector<std::string>& names,
                                                vector<const AttributeCollection*>& collections) const
{
    names.reserve(m_attribute_collections.size());
    collections.reserve(m_attribute_collections.size());
    for(auto&& [name, ac] : m_attribute_collections)
    {
        names.push_back(name);
        collections.push_back(ac.get());
    }
}

void Geometry::do_collect_attribute_collections(vector<std::string>& names,
                                                vector<AttributeCollection*>& collections)
{
    names.reserve(m_attribute_collections.size());
    collections.reserve(m_attribute_collections.size());
    for(auto&& [name, ac] : m_attribute_collections)
    {
        names.push_back(name);
        collections.push_back(ac.get());
    }
}

void Geometry::do_build_from_attribute_collections(span<const std::string> names,
                                                   span<const AttributeCollection*> collections)
{
    for(auto&& [name, ac] : zip(names, collections))
    {
        m_attribute_collections[name] = uipc::make_shared<AttributeCollection>(*ac);
    }
}

void Geometry::do_update_from(const GeometryCommit& commit)
{
    for(auto&& [name, cmt] : commit.m_attribute_collections)
    {
        auto& this_ac = *m_attribute_collections[name];
        this_ac += *cmt;  // update the attribute collection
    }
}

S<IGeometry> Geometry::do_clone() const
{
    return uipc::make_shared<Geometry>(*this);
}

std::string_view Geometry::get_type() const noexcept
{
    return builtin::Geometry;
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
