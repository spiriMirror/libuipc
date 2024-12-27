#include <sanity_checker_collection.h>
#include <sanity_checker_auto_register.h>
#include <context.h>
#include <spdlog/spdlog.h>
#include <filesystem>
namespace uipc::sanity_check
{
SanityCheckerCollection::SanityCheckerCollection(std::string_view workspace) noexcept
{
    namespace fs = std::filesystem;

    fs::path path{workspace};
    path /= "sanity_check";
    fs::exists(path) || fs::create_directories(path);
    m_workspace = path.string();
}

SanityCheckerCollection::~SanityCheckerCollection() {}

std::string_view SanityCheckerCollection::workspace() const noexcept
{
    return m_workspace;
}

void SanityCheckerCollection::build(core::Scene& s)
{
    for(const auto& creator : SanityCheckerAutoRegister::creators().entries)
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
        catch(const SanityCheckerException& e)
        {
            spdlog::debug("[{}] shutdown, reason: {}", entry->name(), e.what());
        }
    }

    auto ctx = find<Context>();
    UIPC_ASSERT(ctx != nullptr, "SanityCheckBuild: Context not found");

    ctx->prepare();
}

SanityCheckResult SanityCheckerCollection::check(core::SanityCheckMessageCollection& msgs) const
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
}  // namespace uipc::sanity_check
