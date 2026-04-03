#include <cuda_sanity_checker_collection.h>
#include <cuda_sanity_checker_auto_register.h>
#include <cuda_sanity_checker_exception.h>
#include <context.h>
#include <spdlog/spdlog.h>
#include <filesystem>

namespace uipc::cuda_sanity_check
{
CudaSanityCheckerCollection::CudaSanityCheckerCollection(std::string_view workspace) noexcept
{
    namespace fs = std::filesystem;

    fs::path path{workspace};
    path /= "cuda_sanity_check";
    fs::exists(path) || fs::create_directories(path);
    m_workspace = path.string();
}

CudaSanityCheckerCollection::~CudaSanityCheckerCollection() = default;

std::string_view CudaSanityCheckerCollection::workspace() const noexcept
{
    return m_workspace;
}

void CudaSanityCheckerCollection::build(core::internal::Scene& s)
{
    for(const auto& creator : CudaSanityCheckerAutoRegister::creators().entries)
    {
        auto entry = creator(*this, s);
        if(entry)
        {
            m_entries.emplace_back(std::move(entry));
        }
    }

    for(const auto& entry : m_entries)
    {
        try
        {
            entry->build();
            m_valid_entries.emplace_back(entry.get());
        }
        catch(const CudaSanityCheckerException& e)
        {
            logger::debug("[{}] shutdown, reason: {}", entry->name(), e.what());
        }
    }

    auto ctx = find<Context>();
    UIPC_ASSERT(ctx != nullptr, "CudaSanityCheckBuild: Context not found");

    ctx->prepare();
}

SanityCheckResult CudaSanityCheckerCollection::check(core::SanityCheckMessageCollection& msgs) const
{
    auto ctx    = find<Context>();
    int  result = static_cast<int>(SanityCheckResult::Success);
    for(const auto& entry : m_valid_entries)
    {
        auto& msg = msgs.messages()[entry->id()];
        if(!msg)
            msg = uipc::make_shared<core::SanityCheckMessage>();
        int check = static_cast<int>(entry->check(*msg));
        if(check > result)
            result = check;
    }
    ctx->destroy();
    return static_cast<SanityCheckResult>(result);
}
}  // namespace uipc::cuda_sanity_check
