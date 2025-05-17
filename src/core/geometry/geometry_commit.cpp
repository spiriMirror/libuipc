#include <uipc/geometry/geometry_commit.h>
#include <uipc/common/zip.h>
#include <uipc/geometry/geometry_factory.h>

namespace uipc::geometry
{
GeometryCommit::GeometryCommit(const Geometry& dst, const Geometry& src)
{
    m_is_valid     = true;
    m_new_geometry = nullptr;  // no new geometry

    do  // For the sake of break
    {
        if(dst.type() != src.type())
        {
            m_is_valid = false;
            UIPC_WARN_WITH_LOCATION(
                "GeometryCommit: The type of the geometries are not the same, "
                "dst type is {}, src type is {}, set invalid GeometryCommit!",
                dst.type(),
                src.type());
            break;
        }

        m_type = dst.type();

        vector<std::string>                dst_names;
        vector<const AttributeCollection*> dst_acs;
        static_cast<const IGeometry&>(dst).collect_attribute_collections(dst_names, dst_acs);

        vector<std::string>                src_names;
        vector<const AttributeCollection*> src_acs;
        static_cast<const IGeometry&>(src).collect_attribute_collections(src_names, src_acs);

        if(dst_acs.size() != src_acs.size())
        {
            m_is_valid = false;
            UIPC_WARN_WITH_LOCATION(
                "GeometryCommit: The size of the attribute collections are not the same, "
                "dst size is {}, src size is {}, set invalid GeometryCommit!",
                dst_acs.size(),
                src_acs.size());
            break;
        }

        std::ranges::sort(dst_names);
        std::ranges::sort(src_names);

        if(!std::ranges::equal(dst_names, src_names))
        {
            m_is_valid = false;
            UIPC_WARN_WITH_LOCATION(
                "GeometryCommit: The names of the attribute collections are not the same, "
                "dst names are [{}], src names are [{}], set invalid GeometryCommit!",
                fmt::join(dst_names, ","),
                fmt::join(src_names, ","));
            break;
        }

        m_names.reserve(dst_names.size());
        m_commits.reserve(dst_acs.size());

        for(auto&& [name, dst_ac, src_ac] : zip(dst_names, dst_acs, src_acs))
        {
            m_names.push_back(name);
            m_commits.push_back(*dst_ac - *src_ac);
        }

    } while(0);

    if(!m_is_valid)
    {
        m_commits.clear();
    }
}

GeometryCommit::GeometryCommit(const Geometry& dst)
    : m_is_valid{true}
{
    m_new_geometry = std::static_pointer_cast<Geometry>(dst.clone());
}

GeometryCommit operator-(const Geometry& dst, const Geometry& src)
{
    return GeometryCommit(dst, src);
}

Geometry& operator+=(Geometry& base, const GeometryCommit& inc)
{
    GeometryFactory gf;
    gf.update_from(base, inc);
    return base;
}
}  // namespace uipc::geometry