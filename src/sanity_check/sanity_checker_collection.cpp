#include <sanity_checker_collection.h>
#include <sanity_checker_auto_register.h>
#include <context.h>
#include <spdlog/spdlog.h>

namespace uipc::sanity_check
{
SanityCheckerCollection::SanityCheckerCollection(std::string_view workspace) noexcept
{
    m_workspace = fmt::format("{}/sanity_check/", workspace);
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
            spdlog::info("SanityCheckBuild: {}", e.what());
        }
    }

    auto ctx = find<Context>();
    UIPC_ASSERT(ctx != nullptr, "SanityCheckBuild: Context not found");

    ctx->prepare();
}

SanityCheckResult SanityCheckerCollection::check() const
{
    auto ctx    = find<Context>();
    int  result = static_cast<int>(SanityCheckResult::Success);
    for(const auto& entry : m_valid_entries)
    {
        int check = static_cast<int>(entry->check());

        if(check > result)
            result = check;
    }
    ctx->destroy();
    return static_cast<SanityCheckResult>(result);
}
}  // namespace uipc::sanity_check
