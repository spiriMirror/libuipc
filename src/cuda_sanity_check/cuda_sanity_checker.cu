#include <cuda_sanity_checker.h>
#include <cuda_sanity_checker_collection.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/backend/visitors/sanity_check_message_visitor.h>
#include <uipc/common/demangle.h>
#include <uipc/core/internal/scene.h>

namespace uipc::cuda_sanity_check
{
CudaSanityChecker::CudaSanityChecker(CudaSanityCheckerCollection& c,
                                     core::internal::Scene&       s) noexcept
    : m_collection{c}
    , m_scene{S<core::Scene>{new core::Scene(s.shared_from_this())}}
{
}

std::string_view CudaSanityChecker::workspace() const noexcept
{
    return m_collection.workspace();
}

std::string CudaSanityChecker::this_output_path() const noexcept
{
    namespace fs = std::filesystem;

    fs::path path{workspace()};
    path /= fmt::format("{}", get_id());
    std::filesystem::exists(path) || std::filesystem::create_directories(path);
    return path.string();
}

std::string CudaSanityChecker::get_name() const noexcept
{
    return uipc::demangle(typeid(*this).name());
}

void CudaSanityChecker::build(backend::SceneVisitor& scene) {}

core::Scene::CObjects CudaSanityChecker::objects() const noexcept
{
    return std::as_const(*m_scene).objects();
}

void CudaSanityChecker::build()
{
    backend::SceneVisitor sv{*m_scene};
    build(sv);
}

SanityCheckResult CudaSanityChecker::do_check(core::SanityCheckMessage& msg)
{
    backend::SceneVisitor              sv{*m_scene};
    backend::SanityCheckMessageVisitor scmv{msg};

    scmv.id()     = get_id();
    scmv.name()   = get_name();
    scmv.result() = SanityCheckResult::Success;

    auto result   = do_check(sv, scmv);
    scmv.result() = result;

    return result;
}
}  // namespace uipc::cuda_sanity_check
