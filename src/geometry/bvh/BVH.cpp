#include "BVH.hpp"
#include "Morton.hpp"

namespace SimpleBVH
{
void BVH::clear()
{
    boxlist.clear();
    new2old.clear();
    n_corners = -1;
}

void BVH::intersect(const AABB& box, vector<unsigned int>& list) const
{
    m_tmp.clear();
    assert(n_corners >= 0);
    box_search_recursive(box, m_tmp, 1, 0, n_corners);

    list.resize(m_tmp.size());
    for(int i = 0; i < m_tmp.size(); ++i)
        list[i] = new2old[m_tmp[i]];
}

void BVH::init_boxes_recursive(span<const AABB> cornerlist, int node_index, int b, int e)
{
    assert(b != e);
    assert(node_index < boxlist.size());

    if(b + 1 == e)
    {
        boxlist[node_index] = cornerlist[b];
        return;
    }
    int m      = b + (e - b) / 2;
    int childl = 2 * node_index;
    int childr = 2 * node_index + 1;

    assert(childl < boxlist.size());
    assert(childr < boxlist.size());

    init_boxes_recursive(cornerlist, childl, b, m);
    init_boxes_recursive(cornerlist, childr, m, e);

    assert(childl < boxlist.size());
    assert(childr < boxlist.size());

    boxlist[node_index].extend(boxlist[childl]);
    boxlist[node_index].extend(boxlist[childr]);
}

void BVH::box_search_recursive(const AABB& box, vector<unsigned int>& list, int n, int b, int e) const
{
    assert(e != b);

    assert(n < boxlist.size());
    bool cut = box_intersects_box(box, n);

    if(cut == false)
        return;

    // Leaf case
    if(e == b + 1)
    {
        list.emplace_back(b);
        return;
    }

    int m      = b + (e - b) / 2;
    int childl = 2 * n;
    int childr = 2 * n + 1;

    // assert(childl < boxlist.size());
    // assert(childr < boxlist.size());

    // Traverse the "nearest" child first, so that it has more chances
    // to prune the traversal of the other child.
    box_search_recursive(box, list, childl, b, m);
    box_search_recursive(box, list, childr, m, e);
}

int BVH::max_node_index(int node_index, int b, int e)
{
    assert(e > b);
    if(b + 1 == e)
    {
        return node_index;
    }
    int m      = b + (e - b) / 2;
    int childl = 2 * node_index;
    int childr = 2 * node_index + 1;
    return std::max(max_node_index(childl, b, m), max_node_index(childr, m, e));
}

void BVH::init(span<const AABB> cornerlist)
{
    if(cornerlist.empty())
    {
        clear();
        return;
    }

    n_corners = cornerlist.size();
    m_tmp.resize(n_corners);

    Eigen::MatrixXd box_centers(n_corners, 3);
    for(int i = 0; i < n_corners; ++i)
    {
        box_centers.row(i) = (cornerlist[i].min() + cornerlist[i].max()) / 2;
    }

    const Eigen::RowVector3d vmin   = box_centers.colwise().minCoeff();
    const Eigen::RowVector3d vmax   = box_centers.colwise().maxCoeff();
    const Eigen::RowVector3d center = (vmin + vmax) / 2;
    for(int i = 0; i < n_corners; i++)
    {
        // make box centered at origin
        box_centers.row(i) -= center;
    }

    // after placing box at origin, vmax and vmin are symetric.
    const Eigen::Vector3d scale_point = vmax - center;
    const double          scale       = scale_point.lpNorm<Eigen::Infinity>();
    // if the box is too big, resize it
    if(scale > 100)
    {
        box_centers /= scale;
    }

    struct sortstruct
    {
        int                     order;
        Resorting::MortonCode64 morton;
    };
    std::vector<sortstruct> list;
    const int               multi = 1000;
    list.resize(n_corners);

    for(int i = 0; i < n_corners; i++)
    {
        const Eigen::MatrixXd tmp = box_centers.row(i) * multi;

        list[i].morton = Resorting::MortonCode64(int(tmp(0)), int(tmp(1)), int(tmp(2)));
        list[i].order = i;
    }

    const auto morton_compare = [](const sortstruct& a, const sortstruct& b)
    { return (a.morton < b.morton); };
    std::sort(list.begin(), list.end(), morton_compare);

    new2old.resize(n_corners);
    for(int i = 0; i < n_corners; i++)
    {
        new2old[i] = list[i].order;
    }

    std::vector<AABB> sorted_cornerlist(n_corners);

    for(int i = 0; i < n_corners; i++)
    {
        sorted_cornerlist[i] = cornerlist[list[i].order];
    }

    boxlist.resize(max_node_index(1, 0, n_corners) + 1  // <-- this is because size == max_index + 1 !!!
    );

    init_boxes_recursive(sorted_cornerlist, 1, 0, n_corners);
}

bool BVH::box_intersects_box(const AABB& box, int index) const
{
    const AABB& box2 = boxlist[index];
    return box.intersects(box2);
}
}  // namespace SimpleBVH
