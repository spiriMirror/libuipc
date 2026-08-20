#include <type_define.h>  // to use Eigen in CUDA
#include <app/app.h>
#include <app/asset_dir.h>
#include <collision_detection/linear_bvh.h>
#include <uipc/geometry.h>
#include <cuda_tool/cuda_tool.h>
#include <uipc/common/enumerate.h>
#include <cuda_tool/cuda_tool.h>
#include <uipc/uipc.h>
#include <uipc/common/timer.h>
#include <algorithm>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace cuda_tool;
using namespace uipc;
using namespace uipc::geometry;
using namespace uipc::backend::cuda;

namespace test_lbvh
{
namespace
{
    __global__ void two_step_lbvh_cp_count_kernel(
        LinearBVHViewer                   LinearBVH,
        cuda_tool::BufferView<LinearBVHAABB> aabbs,
        cuda_tool::BufferView<IndexT>     counts,
        int                               n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
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
    }

    __global__ void two_step_lbvh_cp_fill_kernel(
        LinearBVHViewer                   LinearBVH,
        cuda_tool::BufferView<LinearBVHAABB> aabbs,
        cuda_tool::BufferView<IndexT>     counts,
        cuda_tool::BufferView<IndexT>     offsets,
        cuda_tool::BufferView<Vector2i>   pairs,
        int                               n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
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
        UIPC_KERNEL_ASSERT(j == count, "j = %d, count=%d", j, count);
    }

    __global__ void lbvh_query_point_points_kernel(
        cuda_tool::BufferView<Vector3>       points,
        cuda_tool::BufferView<LinearBVHAABB> aabbs,
        int                                  n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        points(i) = aabbs(i).center().cast<Float>();
    }

    __global__ void lbvh_query_point_count_kernel(
        LinearBVHViewer               LinearBVH,
        cuda_tool::BufferView<Vector3> points,
        cuda_tool::BufferView<IndexT> counts,
        int                           n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto N = points.total_size();

        auto point = points(i);
        counts(i)  = LinearBVH.query(point);

        if(i == 0)
        {
            counts(N) = 0;
        }
    }

    __global__ void lbvh_query_point_fill_kernel(
        LinearBVHViewer                 m_lbvh,
        cuda_tool::BufferView<Vector3>  points,
        cuda_tool::BufferView<IndexT>   counts,
        cuda_tool::BufferView<IndexT>   offsets,
        cuda_tool::BufferView<Vector2i> m_pairs,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto point  = points(i);
        auto count  = counts(i);
        auto offset = offsets(i);
        auto pair   = m_pairs.subview(offset, count);
        int  j      = 0;
        m_lbvh.query(point,
                     [&](uint32_t id) { pair(j++) = Vector2i(i, id); });
        UIPC_KERNEL_ASSERT(j == count, "j = %d, count=%d", j, count);
    }

    __global__ void adaptive_lbvh_cp_query_kernel(
        LinearBVHViewer                   m_lbvh,
        cuda_tool::BufferView<LinearBVHAABB> aabbs,
        cuda_tool::Dense<int>             cp_num,
        cuda_tool::BufferView<Vector2i>   pairs,
        int                               n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto N = aabbs.total_size();

        auto aabb = aabbs(i);
        m_lbvh.query(aabb,
                     [&](uint32_t id)
                     {
                         if(id > i)
                         {
                             auto last =
                                 cuda_tool::atomic_add(cp_num.data(), 1);
                             if(last < pairs.total_size())
                             {
                                 pairs(last) = Vector2i(i, id);
                             }
                         }
                     });
    }
}  // namespace

// check if two trees are the same
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
        auto it = std::mismatch(
            a_AABB.begin(),
            a_AABB.end(),
            b_AABB.begin(),
            b_AABB.end(),
            [](const auto& lhs, const auto& rhs)
            { return lhs.min() == rhs.min() && lhs.max() == rhs.max(); });

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

            it = std::mismatch(
                ++it.first,
                a_AABB.end(),
                ++it.second,
                b_AABB.end(),
                [](const auto& lhs, const auto& rhs)
                { return lhs.min() == rhs.min() && lhs.max() == rhs.max(); });
        }
    }
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
    fmt::println("ground_truth.size()={}", gt.size());

    if(!diff.empty())
    {
        fmt::println("diff:");
        for(auto&& d : diff)
        {
            fmt::println("{} {}", d[0], d[1]);
        }
    }
}

std::vector<Vector2i> two_step_lbvh_cp(span<const LinearBVHAABB> aabbs)
{
    DeviceBuffer<LinearBVHAABB> d_aabbs(aabbs.size());
    d_aabbs.view().copy_from(aabbs.data());

    LinearBVH              lbvh;
    DeviceBuffer<Vector2i> pairs;

    {
        Timer timer{"2_step_lbvh"};

        {
            Timer timer{"build_tree"};
            lbvh.build(d_aabbs);
        }
        {
            Timer timer{"rebuild_tree"};
            lbvh.build(d_aabbs);
        }
        {
            Timer timer{"update_aabbs"};
            lbvh.update(d_aabbs);
        }
        DeviceBuffer<IndexT> counts(aabbs.size() + 1ull);
        DeviceBuffer<IndexT> offsets(aabbs.size() + 1ull);

        //for(int i = 0; i < aabbs.size(); ++i)
        //{
        //    auto aabb = aabbs[i];
        //    std::cout << "[" << aabb.min().transpose() << "],"
        //              << "[" << aabb.max().transpose() << "]" << std::endl;
        //}


        {
            Timer timer{"unlucky_build_cp"};

            {
                auto k = two_step_lbvh_cp_count_kernel;
                int  n = (int)aabbs.size();
                if(n > 0)
                {
                    k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                        lbvh.viewer(), d_aabbs.viewer(), counts.viewer(), n);
                }
            }

            DeviceScan().ExclusiveSum(counts.data(), offsets.data(), counts.size());
            IndexT total;
            offsets.view(aabbs.size()).copy_to(&total);

            pairs.resize(total);

            {
                auto k = two_step_lbvh_cp_fill_kernel;
                int  n = (int)aabbs.size();
                if(n > 0)
                {
                    k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                        lbvh.viewer(),
                        d_aabbs.viewer(),
                        counts.viewer(),
                        offsets.viewer(),
                        pairs.viewer(),
                        n);
                }
            }
        }

        {
            Timer timer{"lucky_build_cp"};

            {
                auto k = two_step_lbvh_cp_count_kernel;
                int  n = (int)aabbs.size();
                if(n > 0)
                {
                    k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                        lbvh.viewer(), d_aabbs.viewer(), counts.viewer(), n);
                }
            }

            DeviceScan().ExclusiveSum(counts.data(), offsets.data(), counts.size());
            IndexT total;
            offsets.view(aabbs.size()).copy_to(&total);

            pairs.resize(total);

            {
                auto k = two_step_lbvh_cp_fill_kernel;
                int  n = (int)aabbs.size();
                if(n > 0)
                {
                    k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                        lbvh.viewer(),
                        d_aabbs.viewer(),
                        counts.viewer(),
                        offsets.viewer(),
                        pairs.viewer(),
                        n);
                }
            }
        }
    }

    DeviceBuffer<LinearBVHNode> nodes_1;
    nodes_1.copy_from(LinearBVHVisitor(lbvh).nodes());
    DeviceBuffer<LinearBVHAABB> aabbs_1;
    aabbs_1.copy_from(LinearBVHVisitor(lbvh).aabbs());

    lbvh.build(d_aabbs);  // do_build again, the internal nodes should be the same.
    DeviceBuffer<LinearBVHNode> nodes_2;
    nodes_2.copy_from(LinearBVHVisitor(lbvh).nodes());
    DeviceBuffer<LinearBVHAABB> aabbs_2;
    aabbs_2.copy_from(LinearBVHVisitor(lbvh).aabbs());

    tree_consistency_test(nodes_1, nodes_2, aabbs_1, aabbs_2);

    LinearBVH lbvh2;
    lbvh2.build(d_aabbs);

    DeviceBuffer<LinearBVHNode> nodes_3;
    nodes_3.copy_from(LinearBVHVisitor(lbvh2).nodes());
    DeviceBuffer<LinearBVHAABB> aabbs_3;
    aabbs_3.copy_from(LinearBVHVisitor(lbvh2).aabbs());

    tree_consistency_test(nodes_1, nodes_3, aabbs_1, aabbs_3);

    std::vector<Vector2i> pairs_host;
    pairs.copy_to(pairs_host);

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

std::vector<Vector2i> lbvh_query_point(span<const LinearBVHAABB> aabbs)
{
    DeviceBuffer<LinearBVHAABB> d_aabbs(aabbs.size());
    d_aabbs.view().copy_from(aabbs.data());

    LinearBVH m_lbvh;
    m_lbvh.build(d_aabbs, cuda_tool::Stream::Default());

    DeviceBuffer<IndexT> counts(aabbs.size() + 1ull);
    DeviceBuffer<IndexT> offsets(aabbs.size() + 1ull);


    DeviceBuffer<Vector3> points(aabbs.size());
    {
        auto k = lbvh_query_point_points_kernel;
        int  n = (int)aabbs.size();
        if(n > 0)
        {
            k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                points.viewer(), d_aabbs.viewer(), n);
        }
    }

    {
        auto k = lbvh_query_point_count_kernel;
        int  n = (int)aabbs.size();
        if(n > 0)
        {
            k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                m_lbvh.viewer(), points.viewer(), counts.viewer(), n);
        }
    }

    DeviceScan().ExclusiveSum(counts.data(), offsets.data(), counts.size());
    IndexT total;
    offsets.view(aabbs.size()).copy_to(&total);

    DeviceBuffer<Vector2i> pairs(total);

    {
        auto k = lbvh_query_point_fill_kernel;
        int  n = (int)aabbs.size();
        if(n > 0)
        {
            k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                m_lbvh.viewer(),
                points.viewer(),
                counts.viewer(),
                offsets.viewer(),
                pairs.viewer(),
                n);
        }
    }

    std::vector<Vector2i> pairs_host;
    pairs.copy_to(pairs_host);

    return pairs_host;
}

std::vector<Vector2i> brute_froce_query_point(span<const LinearBVHAABB> aabbs)
{
    std::vector<Vector2i> pairs;

    std::vector<Vector3> points(aabbs.size());

    std::ranges::transform(aabbs,
                           points.begin(),
                           [](const auto& aabb)
                           {
                               return Eigen::Vector3f{aabb.center()}.cast<Float>();
                           });

    for(auto&& [i, point0] : enumerate(points))
    {
        for(auto&& [j, aabb] : enumerate(aabbs))
        {
            if(aabb.contains(point0.cast<float>()))
            {
                pairs.push_back(Vector2i(i, j));
            }
        }
    }

    return pairs;
}

std::vector<Vector2i> adaptive_lbvh_cp(span<const LinearBVHAABB> aabbs)
{
    DeviceBuffer<LinearBVHAABB> d_aabbs(aabbs.size());
    d_aabbs.view().copy_from(aabbs.data());

    LinearBVH m_lbvh;

    DeviceVar<int>         cp_num{0};
    DeviceBuffer<Vector2i> pairs;
    // prepare size with aabbs.size()
    pairs.resize(aabbs.size());
    fmt::println("adaptive_lbvh, prepared_size={}", pairs.size());

    {
        Timer timer{"adaptive_lbvh"};
        {
            Timer timer{"build_tree"};
            m_lbvh.build(d_aabbs);
        }
        {
            Timer timer{"rebuild_tree"};
            m_lbvh.build(d_aabbs);
        }
        {
            Timer timer{"update_aabbs"};
            m_lbvh.update(d_aabbs);
        }


        auto do_query = [&]
        {
            cp_num = 0;
            auto k = adaptive_lbvh_cp_query_kernel;
            int  n = (int)aabbs.size();
            if(n > 0)
            {
                k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                    m_lbvh.viewer(),
                    d_aabbs.viewer(),
                    cp_num.viewer(),
                    pairs.viewer(),
                    n);
            }
        };

        {
            Timer timer{"unlucky_build_cp"};

            {
                Timer timer{"try_build_cp"};
                // try to query with prepared size
                do_query();
            }

            {
                Timer timer{"rebuild_cp"};

                int h_cp_num = cp_num;
                if(h_cp_num > pairs.size())  // if failed, resize and retry
                {
                    fmt::println("try query with prepared_size={} (but too small), we resize it to the detected cp_num={}",
                                 pairs.size(),
                                 h_cp_num);

                    pairs.resize(h_cp_num);
                    do_query();
                    h_cp_num = cp_num;
                    CHECK(h_cp_num == pairs.size());
                }
            }
        }


        {
            Timer timer{"lucky_build_cp"};
            do_query();
        }
    }

    std::vector<Vector2i> pairs_host;
    pairs.copy_to(pairs_host);

    return pairs_host;
}


void lbvh_test(const SimplicialComplex& mesh)
{
    Timer::enable_all();
    Timer::set_sync_func([]() { cuda_tool::wait_device(); });

    std::cout << "num_aabb=" << mesh.triangles().size() << std::endl;

    auto pos_view = mesh.positions().view();
    auto tri_view = mesh.triangles().topo().view();

    std::vector<LinearBVHAABB> aabbs(tri_view.size());
    for(auto&& [i, tri] : enumerate(tri_view))
    {
        auto p0 = pos_view[tri[0]];
        auto p1 = pos_view[tri[1]];
        auto p2 = pos_view[tri[2]];
        aabbs[i].extend(p0.cast<float>()).extend(p1.cast<float>()).extend(p2.cast<float>());
    }

    auto lbvh_pairs = two_step_lbvh_cp(aabbs);
    auto bf_pairs   = brute_froce_cp(aabbs);

    check_cp_conservative(lbvh_pairs, bf_pairs);

    auto lbvh_qp = lbvh_query_point(aabbs);
    auto bf_qp   = brute_froce_query_point(aabbs);
    check_cp_conservative(lbvh_qp, bf_qp);

    auto adaptive = adaptive_lbvh_cp(aabbs);
    check_cp_conservative(adaptive, bf_pairs);

    Timer::set_sync_func(nullptr);

    Timer::report();
}

SimplicialComplex tet()
{
    std::vector<Vector3>  Vs = {Vector3{0.0, 0.0, 0.0},
                                Vector3{1.0, 0.0, 0.0},
                                Vector3{0.0, 1.0, 0.0},
                                Vector3{0.0, 0.0, 1.0}};
    std::vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    return tetmesh(Vs, Ts);
}
}  // namespace test_lbvh


TEST_CASE("lbvh", "[collision detection]")
{
    using namespace test_lbvh;
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

    SECTION("ball.msh")
    {
        fmt::println("ball.msh:");
        SimplicialComplexIO io;
        auto mesh = io.read(fmt::format("{}ball.msh", AssetDir::tetmesh_path()));
        lbvh_test(mesh);
    }

    SECTION("link.msh")
    {
        fmt::println("link.msh:");
        SimplicialComplexIO io;
        auto mesh = io.read(fmt::format("{}link.msh", AssetDir::tetmesh_path()));
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

