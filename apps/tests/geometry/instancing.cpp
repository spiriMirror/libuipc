#include <catch2/catch_all.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <algorithm>
#include <memory_resource>
#include <utility>

using namespace uipc;
using namespace uipc::geometry;


TEST_CASE("instancing", "[instance]")
{
    SimplicialComplexIO io;

    auto mesh = io.read(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));

    REQUIRE(mesh.instances().size() == 1);  // the initial mesh is an instance of itself

    mesh.instances().resize(5);

    auto trans      = mesh.instances().find<Matrix4x4>(builtin::transform);
    auto trans_view = trans->view();
    for(auto&& t : trans_view)
    {
        REQUIRE(t == Matrix4x4::Identity());
    }
}

namespace
{
class AllocationCounter final : public std::pmr::memory_resource
{
  public:
    size_t peak = 0;

  private:
    void* do_allocate(size_t bytes, size_t alignment) override
    {
        auto* result = m_upstream->allocate(bytes, alignment);
        m_current += bytes;
        peak = std::max(peak, m_current);
        return result;
    }

    void do_deallocate(void* pointer, size_t bytes, size_t alignment) override
    {
        m_upstream->deallocate(pointer, bytes, alignment);
        m_current -= bytes;
    }

    bool do_is_equal(const std::pmr::memory_resource& other) const noexcept override
    {
        return this == &other;
    }

    std::pmr::memory_resource* m_upstream = std::pmr::get_default_resource();
    size_t                     m_current  = 0;
};

class DefaultResourceScope
{
  public:
    explicit DefaultResourceScope(std::pmr::memory_resource& resource)
        : m_previous(std::pmr::set_default_resource(&resource))
    {
    }

    ~DefaultResourceScope() { std::pmr::set_default_resource(m_previous); }

    DefaultResourceScope(const DefaultResourceScope&)            = delete;
    DefaultResourceScope& operator=(const DefaultResourceScope&) = delete;

  private:
    std::pmr::memory_resource* m_previous;
};
}  // namespace

TEST_CASE("apply_transform preserves instance data within a linear allocation budget",
          "[instance][apply_transform]")
{
    const auto             count      = GENERATE(0, 1, 512);
    const vector<Vector3>  points     = {Vector3{0.0, 0.0, 0.0},
                                         Vector3{1.0, 0.0, 0.0},
                                         Vector3{0.0, 1.0, 0.0},
                                         Vector3{0.0, 0.0, 1.0}};
    const vector<Vector4i> tetrahedra = {Vector4i{0, 1, 2, 3}};
    auto                   source     = tetmesh(points, tetrahedra);
    source.instances().resize(count);
    auto ids = source.instances().create<IndexT>("instance_id", -1);
    ids->is_evolving(true);
    auto              id_view        = view(*ids);
    auto              transform_view = view(source.transforms());
    vector<Matrix4x4> transforms(count);
    for(IndexT i = 0; i < count; ++i)
    {
        Transform transform = Transform::Identity();
        if(i != 0)
        {
            transform.translate(Vector3{Float(i), 2.0, -3.0});
            transform.rotate(Eigen::AngleAxisd(0.3, Vector3::UnitZ()));
            transform.scale(Vector3{1.2, 0.8, 1.5});
        }
        transforms[i] = transform_view[i] = transform.matrix();
        id_view[i]                        = i + 7;
    }

    size_t peak = 0;
    {
        AllocationCounter    allocations;
        DefaultResourceScope resource_scope(allocations);
        auto                 expanded = apply_transform(source);
        peak                          = allocations.peak;
        REQUIRE(expanded.size() == count);
        for(IndexT i = 0; i < count; ++i)
        {
            const auto& instance = expanded[i];
            REQUIRE(instance.instances().size() == 1);
            REQUIRE(instance.transforms().view()[0].isIdentity());
            auto instance_ids = instance.instances().find<IndexT>("instance_id");
            REQUIRE(instance_ids);
            REQUIRE(instance_ids->view()[0] == i + 7);
            REQUIRE(instance_ids->is_evolving());
            REQUIRE(instance.tetrahedra().size() == 1);
            auto positions = instance.positions().view();
            for(size_t j = 0; j < points.size(); ++j)
                REQUIRE(positions[j].isApprox(Transform{transforms[i]} * points[j]));
        }
    }

    // Allow metadata and geometry copies, but not a full instance table per result.
    CAPTURE(count, peak);
    REQUIRE(peak <= size_t(std::max(count, 1)) * 16 * 1024);
    const auto& original = source;
    for(size_t j = 0; j < points.size(); ++j)
        REQUIRE(original.positions().view()[j].isApprox(points[j]));
    for(IndexT i = 0; i < count; ++i)
    {
        REQUIRE(original.transforms().view()[i].isApprox(transforms[i]));
        REQUIRE(original.instances().find<IndexT>("instance_id")->view()[i] == i + 7);
    }
}
