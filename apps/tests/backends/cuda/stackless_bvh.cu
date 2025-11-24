#include <type_define.h>  // to use Eigen in CUDA
#include <uipc/common/enumerate.h>
#include <app/test_common.h>
#include <app/asset_dir.h>
#include <collision_detection/stackless_bvh.h>
#include <uipc/geometry.h>
#include <uipc/geometry/simplicial_complex.h>
#include <muda/cub/device/device_scan.h>
#include <muda/viewer/viewer_base.h>
#include <uipc/uipc.h>
#include <uipc/common/timer.h>
#include "culbvh/stacklessbvh.cuh"

using namespace muda;
using namespace uipc;
using namespace uipc::geometry;
using namespace uipc::backend::cuda;


namespace test_stackless_bvh
{
template <typename T>
void compare(thrust::device_vector<T>& l, thrust::device_vector<T>& r)
{
    std::vector<T> hl(l.size());
    std::vector<T> hr(r.size());
    thrust::copy(l.begin(), l.end(), hl.begin());
    thrust::copy(r.begin(), r.end(), hr.begin());
    UIPC_ASSERT(hl.size() == hr.size(), "error");

    bool error = false;
    for(size_t i = 0; i < hl.size(); ++i)
    {
        if(hl[i] != hr[i])
        {
            error = true;
            break;
        }
    }

    if(error)
    {
        for(size_t i = 0; i < hl.size(); ++i)
        {
            if(hl[i] != hr[i])
            {
                fmt::println("mismatch at index {}: lhs={}, rhs={}", i, hl[i], hr[i]);
            }
        }
    }

    UIPC_ASSERT(!error, "error");
}

template <typename T1, typename T2, typename F>
void compare(thrust::device_vector<T1>& l, thrust::device_vector<T2>& r, F&& f)
{
    std::vector<T1> hl(l.size());
    std::vector<T2> hr(r.size());
    thrust::copy(l.begin(), l.end(), hl.begin());
    thrust::copy(r.begin(), r.end(), hr.begin());
    REQUIRE(hl.size() == hr.size());

    for(size_t i = 0; i < hl.size(); ++i)
    {
        UIPC_ASSERT(f(hl[i], hr[i]), "error");
    }
}

template <typename T>
void compare(muda::CBufferView<T> l, const T* r)
{
    std::vector<T> hl;
    hl.resize(l.size());
    l.copy_to(hl.data());

    muda::CBufferView<T> r_view(r, l.size());
    std::vector<T>       hr;
    hr.resize(r_view.size());
    r_view.copy_to(hr.data());

    UIPC_ASSERT(hl.size() == hr.size(), "error");

    bool error = false;
    for(size_t i = 0; i < hl.size(); ++i)
    {
        if(hl[i] != hr[i])
        {
            error = true;
            break;
        }
    }

    if(error)
    {
        for(size_t i = 0; i < hl.size(); ++i)
        {
            if(hl[i] != hr[i])
            {
                fmt::println("mismatch at index {}: lhs={}, rhs={}", i, hl[i], hr[i]);
            }
        }
    }

    UIPC_ASSERT(!error, "error");
}

std::vector<Vector2i> brute_froce_cp(span<const AABB> aabbs)
{
    std::vector<Vector2i> pairs;
    for(auto&& [i, aabb0] : enumerate(aabbs))
    {
        for(int j = i + 1; j < aabbs.size(); ++j)
        {
            auto aabb1 = aabbs[j];
            if(aabb1.intersects(aabb0))
            {
                pairs.push_back(Vector2i(i, j));
            }
        }
    }
    return pairs;
}

// check if the test result is conservative (i.e., no false negative)
void check_cp_conservative(span<Vector2i> test, span<Vector2i> gt)
{
    auto compare = [](const Vector2i& lhs, const Vector2i& rhs)
    { return lhs[0] < rhs[0] || (lhs[0] == rhs[0] && lhs[1] < rhs[1]); };

    std::ranges::sort(test, compare);
    std::ranges::sort(gt, compare);

    std::list<Vector2i> diff;
    std::set_difference(
        gt.begin(), gt.end(), test.begin(), test.end(), std::back_inserter(diff), compare);

    CHECK(diff.empty());

    fmt::println("test.size()={}", test.size());

    // fmt::println("test:");
    //for(auto&& d : test)
    //{
    //    fmt::println("{} {}", d[0], d[1]);
    //}

    fmt::println("ground_truth.size()={}", gt.size());

    //if(!diff.empty())
    //{
    //    fmt::println("diff:");
    //    for(auto&& d : diff)
    //    {
    //        fmt::println("{} {}", d[0], d[1]);
    //    }
    //}
}

std::vector<Vector2i> stackless_bvh_cp(span<const AABB> aabbs)
{
    std::vector<Vector2i> cps;

    DeviceBuffer<AABB> d_aabbs(aabbs.size());
    d_aabbs.view().copy_from(aabbs.data());

    StacklessBVH::Config cfg;
    StacklessBVH         bvh{cfg};
    bvh.build(d_aabbs);

    StacklessBVH::QueryBuffer qbuffer;
    qbuffer.reserve(1024);
    bvh.detect([] __device__(int i, int j) { return true; }, qbuffer);

    cps.resize(qbuffer.size());
    qbuffer.view().copy_to(cps.data());

    ::culbvh::LBVHStackless cbvh;
    cbvh.type = 1;

    std::vector<::culbvh::aabb> cul_aabbs(aabbs.size());
    for(auto&& [i, aabb] : enumerate(aabbs))
    {
        cul_aabbs[i].min = make_float3(aabb.min()[0], aabb.min()[1], aabb.min()[2]);
        cul_aabbs[i].max = make_float3(aabb.max()[0], aabb.max()[1], aabb.max()[2]);
    }
    DeviceBuffer<::culbvh::aabb> d_cul_aabbs = cul_aabbs;


    cbvh.compute(d_cul_aabbs.data(), d_cul_aabbs.size());
    //auto num = cbvh.query();

    auto f = [](const ::culbvh::aabb& lhs, const AABB& rhs)
    {
        bool result = lhs.max.x == rhs.max().x()     //
                      && lhs.max.y == rhs.max().y()  //
                      && lhs.max.z == rhs.max().z()  //
                      && lhs.min.x == rhs.min().x()  //
                      && lhs.min.y == rhs.min().y()  //
                      && lhs.min.z == rhs.min().z();

        return result;
    };

    //compare(cbvh.d_scene_box, bvh.d_scene_box, f);
    //compare(cbvh.d_flags, bvh.d_flags);
    //compare(cbvh.d_mtcode, bvh.d_mtcode);
    //compare(cbvh.d_sorted_id, bvh.d_sorted_id);
    //compare(cbvh.d_primMap, bvh.d_primMap);
    //compare(cbvh.d_metric, bvh.d_metric);
    //compare(cbvh.d_count, bvh.d_count);
    //compare(cbvh.d_tkMap, bvh.d_tkMap);
    //compare(cbvh.d_offsetTable, bvh.d_offsetTable);
    //compare(cbvh.d_ext_aabb, bvh.d_ext_aabb, f);
    //compare(cbvh.d_ext_idx, bvh.d_ext_idx);
    //compare(cbvh.d_ext_lca, bvh.d_ext_lca);
    //compare(cbvh.d_ext_mark, bvh.d_ext_mark);
    //compare(cbvh.d_ext_par, bvh.d_ext_par);
    //compare(cbvh.d_int_lc, bvh.d_int_lc);
    //compare(cbvh.d_int_rc, bvh.d_int_rc);
    //compare(cbvh.d_int_par, bvh.d_int_par);
    //compare(cbvh.d_int_range_x, bvh.d_int_range_x);
    //compare(cbvh.d_int_range_y, bvh.d_int_range_y);
    //compare(cbvh.d_int_mark, bvh.d_int_mark);
    //compare(cbvh.d_int_aabb, bvh.d_int_aabb, f);

    //auto ulonglong_f = [](const ulonglong2 a, const ulonglong2 b)
    //{ return a.x == b.x && a.y == b.y; };

    //compare(cbvh.d_quantNode, bvh.d_quantNode, ulonglong_f);

    return cps;
}

std::vector<Vector2i> brute_froce_query_cp(span<const AABB> aabbs)
{
    std::vector<Vector2i> pairs;
    for(auto&& [i, aabb0] : enumerate(aabbs))
    {
        for(int j = 0; j < aabbs.size(); ++j)
        {
            auto aabb1 = aabbs[j];
            if(aabb1.intersects(aabb0))
            {
                pairs.push_back(Vector2i(i, j));
            }
        }
    }
    return pairs;
}

std::vector<Vector2i> stackless_bvh_query_cp(span<const AABB> aabbs)
{
    std::vector<Vector2i> cps;

    DeviceBuffer<AABB> d_aabbs(aabbs.size());
    d_aabbs.view().copy_from(aabbs.data());

    StacklessBVH::Config cfg;
    StacklessBVH         bvh{cfg};


    StacklessBVH::QueryBuffer qbuffer;

    for(int I = 0; I < 10000; ++I)
    {
        bvh.build(d_aabbs);
        bvh.query(d_aabbs, [] __device__(int i, int j) { return true; }, qbuffer);
    }


    cps.resize(qbuffer.size());
    qbuffer.view().copy_to(cps.data());

    ::culbvh::LBVHStackless cbvh;
    cbvh.type = 1;

    std::vector<::culbvh::aabb> cul_aabbs(aabbs.size());
    for(auto&& [i, aabb] : enumerate(aabbs))
    {
        cul_aabbs[i].min = make_float3(aabb.min()[0], aabb.min()[1], aabb.min()[2]);
        cul_aabbs[i].max = make_float3(aabb.max()[0], aabb.max()[1], aabb.max()[2]);
    }
    DeviceBuffer<::culbvh::aabb> d_cul_aabbs = cul_aabbs;


    cbvh.compute(d_cul_aabbs.data(), d_cul_aabbs.size());
    auto num = cbvh.queryOther(d_cul_aabbs.data(), d_cul_aabbs.size());
    std::cout << "num pairs: " << num << std::endl;

    compare(std::as_const(qbuffer.m_querySortedId).view(), cbvh.d_querySortedId);
    compare(std::as_const(qbuffer.m_queryMtCode).view(), cbvh.d_queryMtCode);


    auto f = [](const ::culbvh::aabb& lhs, const AABB& rhs)
    {
        bool result = lhs.max.x == rhs.max().x()     //
                      && lhs.max.y == rhs.max().y()  //
                      && lhs.max.z == rhs.max().z()  //
                      && lhs.min.x == rhs.min().x()  //
                      && lhs.min.y == rhs.min().y()  //
                      && lhs.min.z == rhs.min().z();

        return result;
    };

    compare(cbvh.d_scene_box, bvh.d_scene_box, f);
    compare(cbvh.d_flags, bvh.d_flags);
    compare(cbvh.d_mtcode, bvh.d_mtcode);
    compare(cbvh.d_sorted_id, bvh.d_sorted_id);
    compare(cbvh.d_primMap, bvh.d_primMap);
    compare(cbvh.d_metric, bvh.d_metric);
    compare(cbvh.d_count, bvh.d_count);
    compare(cbvh.d_tkMap, bvh.d_tkMap);
    compare(cbvh.d_offsetTable, bvh.d_offsetTable);
    compare(cbvh.d_ext_aabb, bvh.d_ext_aabb, f);
    compare(cbvh.d_ext_idx, bvh.d_ext_idx);
    compare(cbvh.d_ext_lca, bvh.d_ext_lca);
    compare(cbvh.d_ext_mark, bvh.d_ext_mark);
    compare(cbvh.d_ext_par, bvh.d_ext_par);
    compare(cbvh.d_int_lc, bvh.d_int_lc);
    compare(cbvh.d_int_rc, bvh.d_int_rc);
    compare(cbvh.d_int_par, bvh.d_int_par);
    compare(cbvh.d_int_range_x, bvh.d_int_range_x);
    compare(cbvh.d_int_range_y, bvh.d_int_range_y);
    compare(cbvh.d_int_mark, bvh.d_int_mark);
    compare(cbvh.d_int_aabb, bvh.d_int_aabb, f);

    auto ulonglong_f = [](const ulonglong2 a, const ulonglong2 b)
    { return a.x == b.x && a.y == b.y; };

    compare(cbvh.d_quantNode, bvh.d_quantNode, ulonglong_f);

    return cps;
}


void stackless_bvh_test(const SimplicialComplex& mesh)
{
    Timer::enable_all();
    Timer::set_sync_func([]() { muda::wait_device(); });

    std::cout << "num_aabb=" << mesh.triangles().size() << std::endl;

    auto pos_view = mesh.positions().view();
    auto tri_view = mesh.triangles().topo().view();

    std::vector<AABB> aabbs(tri_view.size());
    for(auto&& [i, tri] : enumerate(tri_view))
    {
        auto p0 = pos_view[tri[0]];
        auto p1 = pos_view[tri[1]];
        auto p2 = pos_view[tri[2]];
        aabbs[i].extend(p0.cast<float>()).extend(p1.cast<float>()).extend(p2.cast<float>());
    }


    auto bcp = brute_froce_cp(aabbs);
    auto scp = stackless_bvh_cp(aabbs);
    check_cp_conservative(scp, bcp);

    auto bqcp = brute_froce_query_cp(aabbs);
    auto sqcp = stackless_bvh_query_cp(aabbs);
    check_cp_conservative(sqcp, bqcp);

    Timer::set_sync_func(nullptr);

    Timer::report();
}


SimplicialComplex tet()
{
    std::vector           Vs = {Vector3{0.0, 0.0, 0.0},
                                Vector3{1.0, 0.0, 0.0},
                                Vector3{0.0, 1.0, 0.0},
                                Vector3{0.0, 0.0, 1.0}};
    std::vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    return tetmesh(Vs, Ts);
}
}  // namespace test_stackless_bvh


TEST_CASE("stackless_bvh", "[collision detection]")
{
    using namespace test_stackless_bvh;
    muda::Debug::debug_sync_all(true);
    //SECTION("tet")
    //{
    //    fmt::println("tet:");
    //    stackless_bvh_test(tet());
    //}

    //SECTION("cube.obj")
    //{
    //    fmt::println("cube.obj:");
    //    SimplicialComplexIO io;
    //    auto mesh = io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    //    stackless_bvh_test(mesh);
    //}

    //SECTION("cube.msh")
    //{
    //    fmt::println("cube.msh:");
    //    SimplicialComplexIO io;
    //    auto mesh = io.read(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    //    stackless_bvh_test(mesh);
    //}

    //SECTION("link.msh")
    //{
    //    fmt::println("link.msh:");
    //    SimplicialComplexIO io;
    //    auto mesh = io.read(fmt::format("{}link.msh", AssetDir::tetmesh_path()));
    //    stackless_bvh_test(mesh);
    //}

    //SECTION("ball.msh")
    //{
    //    fmt::println("ball.msh:");
    //    SimplicialComplexIO io;
    //    auto mesh = io.read(fmt::format("{}ball.msh", AssetDir::tetmesh_path()));
    //    stackless_bvh_test(mesh);
    //}

    SECTION("bunny0.msh")
    {
        fmt::println("bunny0.msh:");
        SimplicialComplexIO io;
        auto mesh = io.read(fmt::format("{}bunny0.msh", AssetDir::tetmesh_path()));
        stackless_bvh_test(mesh);
    }
}
