#include <sanity_checker.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/io/spread_sheet_io.h>
#include <context.h>

namespace uipc::sanity_check
{
class InitSurfaceIntersectionCheck final : public SanityChecker
{
  public:
    using SanityChecker::SanityChecker;

  protected:
    virtual U64 get_id() const noexcept override { return 1; }

    virtual SanityCheckResult do_check(backend::SceneVisitor& scene) noexcept override
    {
        auto context = find<Context>();

        auto& secne_surface = context->scene_simplicial_surface();

        geometry::SpreadSheetIO io{workspace()};

        // io.write_json("InitSurfaceIntersectionCheck", secne_surface);

        return SanityCheckResult::Success;
    };
};

REGISTER_SANITY_CHECKER(InitSurfaceIntersectionCheck);
}  // namespace uipc::sanity_check
