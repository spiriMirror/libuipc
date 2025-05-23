#include <uipc/geometry/geometry_commit.h>
#include <uipc/common/zip.h>
#include <uipc/geometry/geometry_factory.h>

namespace uipc::geometry
{
GeometryCommit::GeometryCommit(const Geometry& dst, const Geometry& src)
{
    m_is_valid     = true;
    m_new_geometry = nullptr;  // no new geometry

    auto create_commit = [&]()
    {
        if(dst.type() != src.type())
        {
            m_is_valid = false;
            UIPC_WARN_WITH_LOCATION(
                "GeometryCommit: The type of the geometries are not the same, "
                "dst type is {}, src type is {}, set invalid GeometryCommit!",
                dst.type(),
                src.type());
            return;
        }

        m_type = dst.type();

        auto& dst_acs = dst.m_attribute_collections;
        auto& src_acs = src.m_attribute_collections;

        if(dst_acs.size() != src_acs.size())
        {
            m_is_valid = false;
            UIPC_WARN_WITH_LOCATION(
                "GeometryCommit: The size of the attribute collections are not the same, "
                "dst size is {}, src size is {}, set invalid GeometryCommit!",
                dst_acs.size(),
                src_acs.size());
            return;
        }

        for(auto&& [name, dst_ac] : dst_acs)
        {
            auto src_it = src_acs.find(name);
            if(src_it == src_acs.end())
            {
                m_is_valid = false;
                UIPC_WARN_WITH_LOCATION(
                    "GeometryCommit: The attribute collection [{}] is not found in the source geometry, "
                    "set invalid GeometryCommit!",
                    name);
                return;
            }
            auto& src_ac = src_it->second;
            auto  ac_commit =
                uipc::make_shared<AttributeCollectionCommit>(*dst_ac - *src_ac);
            m_attribute_collections.insert({name, std::move(ac_commit)});
        }
    };

    create_commit();

    if(!m_is_valid)
    {
        m_attribute_collections.clear();
    }
}

GeometryCommit::GeometryCommit(const GeometryCommit& gc)
    : m_is_valid(gc.m_is_valid)
    , m_type(gc.m_type)
{
    if(gc.m_new_geometry)
    {
        m_new_geometry = std::static_pointer_cast<Geometry>(gc.m_new_geometry->clone());
    }
    else
    {
        m_new_geometry = nullptr;
    }

    for(auto&& [name, ac] : gc.m_attribute_collections)
    {
        m_attribute_collections[name] =
            uipc::make_shared<AttributeCollectionCommit>(*ac);
    }
}

//GeometryCommit& GeometryCommit::operator=(const GeometryCommit& gc)
//{
//    if(this != &gc)
//    {
//        m_attribute_collections = gc.m_attribute_collections;
//        m_is_valid              = gc.m_is_valid;
//        m_type                  = gc.m_type;
//
//        if(gc.m_new_geometry)
//        {
//            m_new_geometry =
//                std::static_pointer_cast<Geometry>(gc.m_new_geometry->clone());
//        }
//        else
//        {
//            m_new_geometry = nullptr;
//        }
//    }
//    return *this;
//}

GeometryCommit::GeometryCommit(const Geometry& dst)
    : m_is_valid{true}
    , m_new_geometry{std::static_pointer_cast<Geometry>(dst.clone())}
{
}

GeometryCommit operator-(const Geometry& dst, const Geometry& src)
{
    return GeometryCommit(dst, src);
}

Geometry& operator+=(Geometry& base, const GeometryCommit& inc)
{
    base.update_from(inc);
    return base;
}
}  // namespace uipc::geometry