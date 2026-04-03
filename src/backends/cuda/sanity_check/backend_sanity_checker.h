#pragma once
#include <uipc/core/i_sanity_checker.h>
#include <uipc/common/demangle.h>
#include <backends/common/sanity_checker_auto_register.h>
#include <filesystem>

namespace uipc::backend::cuda
{
using uipc::core::SanityCheckResult;

class BackendSanityChecker : public core::ISanityChecker
{
  public:
    explicit BackendSanityChecker(core::ISanityCheckContext& ctx) noexcept
        : m_ctx(ctx)
    {
    }

    std::string_view workspace() const noexcept { return m_ctx.workspace(); }

    std::string this_output_path() const noexcept
    {
        namespace fs = std::filesystem;
        fs::path path{workspace()};
        path /= fmt::format("{}", get_id());
        fs::exists(path) || fs::create_directories(path);
        return path.string();
    }

    S<const core::Object> find_object(IndexT id) const noexcept
    {
        return m_ctx.find_object(id);
    }

  protected:
    virtual std::string get_name() const noexcept override
    {
        return uipc::demangle(typeid(*this).name());
    }

    core::ISanityCheckContext&       context() noexcept { return m_ctx; }
    const core::ISanityCheckContext& context() const noexcept { return m_ctx; }

  private:
    core::ISanityCheckContext& m_ctx;
};

class BackendSanityCheckerException : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::backend::cuda
