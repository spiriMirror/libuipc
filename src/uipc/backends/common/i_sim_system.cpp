#include <typeinfo>
#include <uipc/backends/common/i_sim_system.h>
#include <uipc/backends/common/module.h>
#include <filesystem>

namespace uipc::backend
{
void ISimSystem::build()
{
    //spdlog::info("Building system: {}", name());
    do_build();
}

void ISimSystem::make_engine_aware()
{
    set_engine_aware();
}

void ISimSystem::invalidate() noexcept
{
    set_invalid();
}

bool ISimSystem::is_valid() const noexcept
{
    return get_valid();
}

bool ISimSystem::is_building() const noexcept
{
    return get_is_building();
}

span<ISimSystem* const> ISimSystem::dependencies() const noexcept
{
    return get_dependencies();
}

std::string_view ISimSystem::name() const noexcept
{
    return get_name();
}

bool ISimSystem::is_engine_aware() const noexcept
{
    return get_engine_aware();
}

Json ISimSystem::to_json() const
{
    return do_to_json();
}

bool ISimSystem::dump(DumpInfo& info)
{
    return do_dump(info);
}

bool ISimSystem::try_recover(RecoverInfo& info)
{
    return do_try_recover(info);
}

void ISimSystem::apply_recover(RecoverInfo& info)
{
    do_apply_recover(info);
}

void ISimSystem::clear_recover(RecoverInfo& info)
{
    do_clear_recover(info);
}

bool ISimSystem::do_dump(DumpInfo&)
{
    return true;
}

bool ISimSystem::do_try_recover(RecoverInfo&)
{
    return true;
}

void ISimSystem::do_apply_recover(RecoverInfo&) {}

void ISimSystem::do_clear_recover(RecoverInfo&) {}

ISimSystem::BaseInfo::BaseInfo(SizeT frame, std::string_view workspace, const Json& config) noexcept
    : m_frame(frame)
    , m_config(config)
    , m_workspace(workspace)
{
}

std::string_view ISimSystem::BaseInfo::workspace() const noexcept
{
    return m_workspace;
}

std::string ISimSystem::BaseInfo::dump_path(std::string_view _file_) const noexcept
{
    namespace fs = std::filesystem;

    fs::path workspace = m_workspace;

    fs::path out_path = fs::absolute(fs::path{workspace} / "sim_data");
    fs::path file_path{_file_};

    fs::path base         = UIPC_PROJECT_DIR;
    fs::path backends_dir = base / "src" / "uipc" / "backends";
    fs::path relative     = fs::relative(file_path, backends_dir);

    // remove the first folder in relative path, which is the backend name

    relative = relative.remove_filename();
    auto it  = relative.begin();
    auto end = relative.end();

    fs::path new_relative;
    while(++it != end)
        new_relative /= *it;
    new_relative /= file_path.filename();

    fs::path file_output_path = out_path / new_relative;
    // create all the intermediate directories if they don't exist
    if(!fs::exists(file_output_path))
        fs::create_directories(file_output_path);

    return (file_output_path / "").string();
}
const Json& ISimSystem::BaseInfo::config() const noexcept
{
    return m_config;
}

SizeT ISimSystem::BaseInfo::frame() const noexcept
{
    return m_frame;
}
}  // namespace uipc::backend
