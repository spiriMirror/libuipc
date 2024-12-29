/****************************************************************************************
NOTE:
This is a modified version of the original code, which is licensed under the MIT License. 
Ref: https://github.com/geometryprocessing/SimpleBVH/tree/main 

The original code is modified to fit the needs of this project. We thank the original authors
for thier excellent work, sincerely.
*****************************************************************************************/

#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/span.h>
#include <uipc/common/vector.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <array>
#include <cassert>

namespace SimpleBVH
{
using uipc::Float;
using uipc::span;
using uipc::vector;
using AABB = Eigen::AlignedBox<Float, 3>;
using VectorMax3d = Eigen::Matrix<Float, Eigen::Dynamic, 1, Eigen::ColMajor, 3, 1>;

class BVH
{
  public:
    void init(span<const AABB> cornerlist);

    void clear();

    void intersect(const AABB& box, vector<unsigned int>& list) const;

    span<const AABB> boxes() const { return boxlist; }

  private:
    void init_boxes_recursive(span<const AABB> cornerlist, int node_index, int b, int e);

    void box_search_recursive(const AABB& box, vector<unsigned int>& list, int n, int b, int e) const;

    bool box_intersects_box(const AABB& box, int index) const;

    static int max_node_index(int node_index, int b, int e);

    
    vector<AABB>                 boxlist;
    vector<int>                  new2old;
    size_t                       n_corners = -1;
    mutable vector<unsigned int> m_tmp;
};
}  // namespace SimpleBVH
