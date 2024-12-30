#include <uipc/geometry/utils/bvh.h>
#include <uipc/common/enumerate.h>
#include <bvh/BVH.hpp>

namespace uipc::geometry
{
class BVH::Impl
{
  public:
    void build(span<const AABB> aabbs) { m_impl.init(aabbs); }
    void clear() { m_impl.clear(); }
    void query(span<const AABB> aabbs, std::function<void(IndexT, IndexT)>&& QF) const
    {
        if(aabbs.empty() || m_impl.boxes().empty())
            return;
        m_indices.reserve(aabbs.size());
        for(auto&& [i, aabb] : enumerate(aabbs))
        {
            m_indices.clear();
            m_impl.intersect(aabb, m_indices);

            for(const auto& index : m_indices)
            {
                QF(i, index);
            }
        }
    }

    SimpleBVH::BVH               m_impl;
    mutable vector<unsigned int> m_indices;
};

BVH::BVH()
    : m_impl(uipc::make_unique<Impl>())
{
}

BVH::~BVH() {}

void BVH::build(span<const AABB> aabbs)
{
    m_impl->build(aabbs);
}

void BVH::clear()
{
    m_impl->clear();
}

void BVH::query(span<const AABB> aabbs, std::function<void(IndexT, IndexT)>&& QF) const
{
    m_impl->query(aabbs, std::move(QF));
}
}  // namespace uipc::geometry
