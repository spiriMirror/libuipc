#pragma once
// Xiao Cheng, 2025
#include "bound.h"
#include "thrust/device_vector.h"
#include "thrust/device_ptr.h"
#include <memory>
#include "typedef.h"

namespace culbvh
{
using aabb = Bound<float>;
struct __align__(16) stacklessnode
{
    int  lc;
    int  escape;
    aabb bound;
};

class LBVHStackless
{
  public:
    using vec_type = float3;

    LBVHStackless();
    ~LBVHStackless();

    bool is_valid() const { return numObjs > 0; }

    size_t size() const { return numObjs; }

    void compute(aabb* devicePtr, size_t size);

    size_t query();

    size_t queryOther(aabb* devicePtr, size_t size);

    int type = 0;  // 0: quant node 16 bytes,   1: 32 bytes

    thrust::device_ptr<aabb> d_objs = nullptr;

    thrust::device_vector<aabb>     d_scene_box;  ///< external bounding boxes
    thrust::device_vector<uint32_t> d_flags;
    thrust::device_vector<uint32_t> d_mtcode;  ///< external morton codes
    thrust::device_vector<int32_t>  d_sorted_id;
    thrust::device_vector<int32_t>  d_primMap;
    thrust::device_vector<int>      d_metric;
    thrust::device_vector<uint32_t> d_count;
    thrust::device_vector<int>      d_tkMap;
    thrust::device_vector<uint32_t> d_offsetTable;

    thrust::device_vector<aabb>     d_ext_aabb;
    thrust::device_vector<int>      d_ext_idx;
    thrust::device_vector<int>      d_ext_lca;
    thrust::device_vector<uint32_t> d_ext_mark;
    thrust::device_vector<uint32_t> d_ext_par;

    thrust::device_vector<int>      d_int_lc;
    thrust::device_vector<int>      d_int_rc;
    thrust::device_vector<int>      d_int_par;
    thrust::device_vector<int>      d_int_range_x;
    thrust::device_vector<int>      d_int_range_y;
    thrust::device_vector<uint32_t> d_int_mark;
    thrust::device_vector<aabb>     d_int_aabb;

    thrust::device_vector<ulonglong2> d_quantNode;

    thrust::device_vector<stacklessnode> d_nodes;

    aabb   rootBounds;
    size_t numObjs{0};

    //cub storage
    size_t temp_storage_bytes = 0;
    size_t max_storage_bytes  = 0;
    void*  d_temp_storage     = nullptr;


    //result
    int   max_cpNum = 5000000;
    int   h_cpNum;
    int*  d_cpNum;
    int2* d_cpRes;
    bool  allocated{false};

    unsigned int* d_queryMtCode;
    aabb*         d_querySceneBox;
    int*          d_querySortedId;
    int           queryNum = 0;
};

void testStacklessLBVH();
}  // namespace culbvh