#include <muda/ext/eigen/eigen_core_cxx20.h>  // to use Eigen in CUDA
#include <app/test_common.h>
#include <app/asset_dir.h>
#include <linear_bvh.h>
#include <uipc/geometry.h>
#include <muda/cub/device/device_scan.h>
#include <uipc/common/enumerate.h>
#include <muda/viewer/viewer_base.h>

using namespace muda;
using namespace uipc;
using namespace uipc::geometry;
using namespace uipc::backend::cuda;

void tree_consistency_test(const DeviceBuffer<LinearBVHNode>& d_a,
                           const DeviceBuffer<LinearBVHNode>& d_b,
                           const DeviceBuffer<LinearBVHAABB>& d_a_AABB,
                           const DeviceBuffer<LinearBVHAABB>& d_b_AABB)
{
    std::vector<LinearBVHNode> a;
    d_a.copy_to(a);

    std::vector<LinearBVHNode> b;
    d_b.copy_to(b);

    std::vector<LinearBVHAABB> a_AABB;
    d_a_AABB.copy_to(a_AABB);

    std::vector<LinearBVHAABB> b_AABB;
    d_b_AABB.copy_to(b_AABB);

    {
        auto it = std::mismatch(a.begin(),
                                a.end(),
                                b.begin(),
                                b.end(),
                                [](const auto& lhs, const auto& rhs)
                                {
                                    return lhs.parent_idx == rhs.parent_idx
                                           && lhs.left_idx == rhs.left_idx
                                           && lhs.right_idx == rhs.right_idx
                                           && lhs.object_idx == rhs.object_idx;
                                });

        REQUIRE(it.first == a.end());
        REQUIRE(it.second == b.end());

        if(it.first != a.end() || it.second != b.end())
        {
            std::cout << "tree inconsistency detected:" << std::endl;

            for(int i = 0; i < a.size(); ++i)
            {
                auto f_node = a[i];
                auto s_node = b[i];

                std::cout
                    << "node=" << i << "parent=(" << f_node.parent_idx << ", "
                    << s_node.parent_idx << ")"
                    << "left=(" << f_node.left_idx << ", " << s_node.left_idx << ")"
                    << "right=(" << f_node.right_idx << ", " << s_node.right_idx << ")"
                    << "obj=(" << f_node.object_idx << ", " << s_node.object_idx
                    << ")" << std::endl;
            }
        }
    }

    {
        auto it = std::mismatch(a_AABB.begin(),
                                a_AABB.end(),
                                b_AABB.begin(),
                                b_AABB.end(),
                                [](const auto& lhs, const auto& rhs) {
                                    return lhs.min() == rhs.min()
                                           && lhs.max() == rhs.max();
                                });

        CHECK(it.first == a_AABB.end());
        CHECK(it.second == b_AABB.end());

        while(it.first != a_AABB.end() || it.second != b_AABB.end())
        {
            std::cout << "AABB inconsistency detected: id="
                      << std::distance(a_AABB.begin(), it.first) << std::endl;

            auto f_aabb = *it.first;
            auto s_aabb = *it.second;

            std::cout << "aabb=(" << f_aabb.min().transpose() << ", "
                      << f_aabb.max().transpose() << ")\n"
                      << "aabb=(" << s_aabb.min().transpose() << ", "
                      << s_aabb.max().transpose() << ")" << std::endl;

            it = std::mismatch(++it.first,
                               a_AABB.end(),
                               ++it.second,
                               b_AABB.end(),
                               [](const auto& lhs, const auto& rhs) {
                                   return lhs.min() == rhs.min()
                                          && lhs.max() == rhs.max();
                               });
        }
    }
}


std::vector<Vector2i> LinearBVH_cp(span<const LinearBVHAABB> aabbs)
{
    DeviceBuffer<LinearBVHAABB> d_aabbs(aabbs.size());
    d_aabbs.view().copy_from(aabbs.data());

    // enlarge the aabbs by a 0.1 * diagonal length
    ParallelFor()
        .kernel_name("LinearBVHTest::Enlarge")
        .apply(aabbs.size(),
               [aabbs = d_aabbs.viewer().name("aabbs")] __device__(int i) mutable
               {
                   auto aabb = aabbs(i);
                   auto diag = aabb.sizes().norm();
                   //aabb.min().array() -= 0.1 * diag;
                   //aabb.max().array() += 0.1 * diag;
               });


    LinearBVH lbvh;
    lbvh.build(d_aabbs, muda::Stream::Default());

    DeviceBuffer<IndexT> counts(aabbs.size() + 1ull);
    DeviceBuffer<IndexT> offsets(aabbs.size() + 1ull);

    //for(int i = 0; i < aabbs.size(); ++i)
    //{
    //    auto aabb = aabbs[i];
    //    std::cout << "[" << aabb.min().transpose() << "],"
    //              << "[" << aabb.max().transpose() << "]" << std::endl;
    //}

    ParallelFor()
        .kernel_name("LinearBVHTest::Query")
        .apply(aabbs.size(),
               [LinearBVH = lbvh.viewer().name("LinearBVH"),
                aabbs     = d_aabbs.viewer().name("aabbs"),
                counts = counts.viewer().name("counts")] __device__(int i) mutable
               {
                   auto N = aabbs.total_size();

                   auto aabb  = aabbs(i);
                   auto count = 0;
                   LinearBVH.query(aabb,
                                   [&](uint32_t id)
                                   {
                                       if(id > i)
                                           count++;
                                   });
                   counts(i) = count;

                   if(i == 0)
                   {
                       counts(N) = 0;
                   }
               });

    DeviceScan().ExclusiveSum(counts.data(), offsets.data(), counts.size());
    IndexT total;
    offsets.view(aabbs.size()).copy_to(&total);

    DeviceBuffer<Vector2i> pairs(total);


    ParallelFor()
        .kernel_name("LinearBVHTest::Query")
        .apply(aabbs.size(),
               [LinearBVH = lbvh.viewer().name("LinearBVH"),
                aabbs     = d_aabbs.viewer().name("aabbs"),
                counts    = counts.viewer().name("counts"),
                offsets   = offsets.viewer().name("offsets"),
                pairs = pairs.viewer().name("pairs")] __device__(int i) mutable
               {
                   auto N = aabbs.total_size();

                   auto aabb   = aabbs(i);
                   auto count  = counts(i);
                   auto offset = offsets(i);

                   auto pair = pairs.subview(offset, count);
                   int  j    = 0;
                   LinearBVH.query(aabb,
                                   [&](uint32_t id)
                                   {
                                       if(id > i)
                                           pair(j++) = Vector2i(i, id);
                                   });
                   MUDA_ASSERT(j == count, "j = %d, count=%d", j, count);
               });

    DeviceBuffer<LinearBVHNode> nodes_1 = LinearBVHVisitor(lbvh).nodes();
    DeviceBuffer<LinearBVHAABB> aabbs_1 = LinearBVHVisitor(lbvh).aabbs();

    lbvh.build(d_aabbs);  // build again, the internal nodes should be the same.
    DeviceBuffer<LinearBVHNode> nodes_2 = LinearBVHVisitor(lbvh).nodes();
    DeviceBuffer<LinearBVHAABB> aabbs_2 = LinearBVHVisitor(lbvh).aabbs();

    tree_consistency_test(nodes_1, nodes_2, aabbs_1, aabbs_2);

    LinearBVH lbvh2;
    lbvh2.build(d_aabbs);

    DeviceBuffer<LinearBVHNode> nodes_3 = LinearBVHVisitor(lbvh2).nodes();
    DeviceBuffer<LinearBVHAABB> aabbs_3 = LinearBVHVisitor(lbvh2).aabbs();

    tree_consistency_test(nodes_1, nodes_3, aabbs_1, aabbs_3);

    std::vector<Vector2i> pairs_host;
    pairs.copy_to(pairs_host);

    //std::vector<LinearBVHAABB> aabbs_host(aabbs.size());
    //LinearBVHVisitor(lbvh).aabbs().copy_to(aabbs_host.data());
    //for(auto&& [i, aabb] : enumerate(aabbs_host))
    //{
    //    std::cout << "[" << aabb.min().transpose() << "],"
    //              << "[" << aabb.max().transpose() << "]" << std::endl;
    //}

    //std::vector<LinearBVHNode> nodes_host(2 * aabbs.size() - 1);
    //LinearBVHVisitor(lbvh).nodes().copy_to(nodes_host.data());
    //for(auto&& [i, node] : enumerate(nodes_host))
    //{
    //    std::cout << "node=" << i << "[" << aabbs_host[i].min().transpose() << "],"
    //              << "[" << aabbs_host[i].max().transpose() << "]"
    //              << ", parent=" << node.parent_idx
    //              << ", left=" << node.left_idx << ", right=" << node.right_idx
    //              << ", obj=" << node.object_idx << std::endl;
    //}

    return pairs_host;
}

std::vector<Vector2i> brute_froce_cp(span<const LinearBVHAABB> aabbs)
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


void brute_froce_gpu(span<const LinearBVHAABB> aabbs)
{
    DeviceBuffer<LinearBVHAABB> d_aabbs(aabbs.size());
    d_aabbs.view().copy_from(aabbs.data());

    ParallelFor()
        .kernel_name("BruteForce::Query")
        .apply(aabbs.size(),
               [aabbs = d_aabbs.viewer().name("aabbs")] __device__(int i) mutable
               {
                   auto N = aabbs.total_size();

                   auto aabb0 = aabbs(i);
                   for(int j = i + 1; j < N; ++j)
                   {
                       auto aabb1 = aabbs(j);
                       if(aabb1.intersects(aabb0))
                       {
                           printf("i=%d, j=%d\n", i, j);
                       }
                   }
               });
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


void lbvh_test(const SimplicialComplex& mesh)
{
    std::cout << "num_aabb=" << mesh.triangles().size() << std::endl;

    auto pos_view = mesh.positions().view();
    auto tri_view = mesh.triangles().topo().view();

    //tri_view = tri_view.subspan(0, 5);

    std::vector<LinearBVHAABB> aabbs(tri_view.size());
    for(auto&& [i, tri] : enumerate(tri_view))
    {
        auto p0 = pos_view[tri[0]];
        auto p1 = pos_view[tri[1]];
        auto p2 = pos_view[tri[2]];
        aabbs[i].extend(p0).extend(p1).extend(p2);
    }


    auto LinearBVH_pairs = LinearBVH_cp(aabbs);


    auto bf_pairs = brute_froce_cp(aabbs);

    // brute_froce_gpu(aabbs);


    auto compare = [](const Vector2i& lhs, const Vector2i& rhs)
    { return lhs[0] < rhs[0] || (lhs[0] == rhs[0] && lhs[1] < rhs[1]); };

    std::ranges::sort(LinearBVH_pairs, compare);
    std::ranges::sort(bf_pairs, compare);


    auto check_unique = [](auto begin, auto end)
    {
        for(auto it = begin; it != end; ++it)
        {
            if(it + 1 != end && *it == *(it + 1))
            {
                return false;
            }
        }
        return true;
    };

    CHECK(check_unique(LinearBVH_pairs.begin(), LinearBVH_pairs.end()));


    std::list<Vector2i> diff;

    std::set_difference(bf_pairs.begin(),
                        bf_pairs.end(),
                        LinearBVH_pairs.begin(),
                        LinearBVH_pairs.end(),
                        std::back_inserter(diff),
                        compare);

    CHECK(diff.empty());

    if(!diff.empty())
    {
        std::cout << "LinearBVH_pairs.size()=" << LinearBVH_pairs.size() << std::endl;
        std::cout << "bf_pairs.size()=" << bf_pairs.size() << std::endl;
        std::cout << "diff:" << std::endl;
        for(auto&& d : diff)
        {
            std::cout << d.transpose() << std::endl;
        }
    }
}

TEST_CASE("LinearBVH", "[muda]")
{
    SECTION("tet")
    {
        fmt::println("tet:");
        lbvh_test(tet());
    }

    SECTION("cube.obj")
    {
        fmt::println("cube.obj:");
        SimplicialComplexIO io;
        auto mesh = io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
        lbvh_test(mesh);
    }

    SECTION("cube.msh")
    {
        fmt::println("cube.msh:");
        SimplicialComplexIO io;
        auto mesh = io.read(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
        lbvh_test(mesh);
    }

    SECTION("cylinder_hole.msh")
    {
        fmt::println("cylinder_hole.msh:");
        SimplicialComplexIO io;
        auto                mesh =
            io.read(fmt::format("{}cylinder_hole.msh", AssetDir::tetmesh_path()));
        lbvh_test(mesh);
    }

    SECTION("simple_axle.msh")
    {
        fmt::println("simple_axle.msh:");
        SimplicialComplexIO io;
        auto mesh = io.read(fmt::format("{}simple_axle.msh", AssetDir::tetmesh_path()));
        lbvh_test(mesh);
    }

    SECTION("wheel_axle.msh")
    {
        fmt::println("wheel_axle.msh:");
        SimplicialComplexIO io;
        auto mesh = io.read(fmt::format("{}wheel_axle.msh", AssetDir::tetmesh_path()));
        lbvh_test(mesh);
    }

    SECTION("bunny0.msh")
    {
        fmt::println("bunny0.msh:");
        SimplicialComplexIO io;
        auto mesh = io.read(fmt::format("{}bunny0.msh", AssetDir::tetmesh_path()));
        lbvh_test(mesh);
    }
}
