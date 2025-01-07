#include <uipc/geometry/utils/octree.h>
#include <uipc/common/vector.h>
#include <uipc/common/enumerate.h>
#include <Octree/octree.h>
#include <Octree/adaptor.eigen.h>
#include <Eigen/Dense>

namespace uipc::geometry
{
class Octree::Impl
{
  public:
    void build(span<const AABB> aabbs)
    {
        octree = Eigen::OctreeBox3d{aabbs};
        leafs.resize(aabbs.size());
        std::copy(aabbs.begin(), aabbs.end(), leafs.begin());
    }

    void clear() { octree.Clear(); }

    void query(span<const AABB> aabbs, std::function<void(IndexT, IndexT)>&& QF) const
    {
        for(auto&& [I, aabb] : enumerate(aabbs))
        {
            auto result = octree.RangeSearch<false>(aabb, leafs);
            for(auto J : result)
            {
                QF(static_cast<IndexT>(I), static_cast<IndexT>(J));
            }
        }
    }

    void detect(std::function<void(IndexT, IndexT)>&& QF) const
    {
        auto results = octree.CollisionDetection(leafs);
        for(auto&& [I, J] : results)
        {
            QF(static_cast<IndexT>(I), static_cast<IndexT>(J));
        }
    }

    Eigen::OctreeBox3d octree;
    vector<AABB>       leafs;
};

Octree::Octree()
    : m_impl(uipc::make_unique<Impl>())
{
}

Octree::~Octree() = default;

void Octree::build(span<const AABB> aabbs)
{
    m_impl->build(aabbs);
}

void Octree::clear()
{
    m_impl->clear();
}

void Octree::query(span<const AABB> aabbs, std::function<void(IndexT, IndexT)>&& QF) const
{
    m_impl->query(aabbs, std::move(QF));
}

void Octree::detect(std::function<void(IndexT, IndexT)>&& QF) const
{
    m_impl->detect(std::move(QF));
}
}  // namespace uipc::geometry
