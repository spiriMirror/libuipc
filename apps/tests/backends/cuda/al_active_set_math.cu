#include <active_set_system/al_active_set_math.h>
#include <app/app.h>
#include <cuda_tool/cuda_tool.h>
#include <limits>

namespace cuda_tool  = uipc::backend::cuda_tool;
namespace al_details = uipc::backend::cuda::details;

namespace al_active_set_test
{
__global__ void atomic_min_candidate_toi_kernel(cuda_tool::CBufferView<uipc::Float> candidates,
                                                cuda_tool::VarView<uipc::Float> result,
                                                int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    al_details::al_atomic_min_candidate_toi(result.data(), candidates(i));
}
}  // namespace al_active_set_test

TEST_CASE("AL active-set filtering math", "[cuda][al-ipc][active-set]")
{
    using uipc::Float;

    SECTION("decay lifetime follows the one-percent cutoff")
    {
        CHECK(al_details::al_inactive_count_limit(0.9) == 43);
        CHECK(al_details::al_inactive_count_limit(0.3) == 3);
        CHECK(al_details::al_inactive_count_limit(0.1) == 2);
        CHECK(al_details::al_inactive_count_limit(std::nextafter(1.0, 0.0))
              == std::numeric_limits<int>::max() - 1);

        const int limit = al_details::al_inactive_count_limit(0.9);
        CHECK(std::pow(0.9, limit) >= al_details::ALConstraintDecayCutoff);
        CHECK(std::pow(0.9, limit + 1) < al_details::ALConstraintDecayCutoff);
    }

    SECTION("only an incident earliest collision is retained")
    {
        CHECK(al_details::al_candidate_has_collision(0.25));
        CHECK(al_details::al_candidate_has_collision(-1e-12));
        CHECK_FALSE(al_details::al_candidate_has_collision(1.0));
        CHECK_FALSE(al_details::al_candidate_has_collision(
            std::numeric_limits<Float>::quiet_NaN()));
        CHECK(al_details::al_keep_new_candidate(0.25, 0.25));
        CHECK(al_details::al_keep_new_candidate(-1e-12, 0.0));
        CHECK_FALSE(al_details::al_keep_new_candidate(0.5, 0.25));
        CHECK_FALSE(al_details::al_keep_new_candidate(1.0, 1.0));
        CHECK_FALSE(al_details::al_keep_new_candidate(
            std::numeric_limits<Float>::quiet_NaN(), 0.0));
    }

    SECTION("double-precision atomic minimum preserves the TOI value")
    {
        cuda_tool::DeviceVector<Float> candidates{0.75, 0.5, 1e-200, 0.25};
        cuda_tool::DeviceVar<Float>    result{2.0};
        const int count = static_cast<int>(candidates.size());
        al_active_set_test::atomic_min_candidate_toi_kernel<<<1, 32>>>(
            candidates.cview(), result.view(), count);
        CUDA_TOOL_CHECK(cudaGetLastError());
        CHECK(static_cast<Float>(result) == 1e-200);

        candidates = cuda_tool::DeviceVector<Float>{0.75, -1e-12, 0.5};
        result     = 2.0;
        al_active_set_test::atomic_min_candidate_toi_kernel<<<1, 32>>>(
            candidates.cview(), result.view(), static_cast<int>(candidates.size()));
        CUDA_TOOL_CHECK(cudaGetLastError());
        CHECK(static_cast<Float>(result) == 0.0);
    }
}
