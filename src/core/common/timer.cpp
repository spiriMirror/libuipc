#include <uipc/common/timer.h>
#include <uipc/common/log.h>
#include <fmt/ranges.h>

namespace uipc::details
{
void ScopedTimer::tick()
{
    start = std::chrono::high_resolution_clock::now();
}

void ScopedTimer::tock()
{
    end      = std::chrono::high_resolution_clock::now();
    duration = end - start;
}
double ScopedTimer::elapsed() const
{
    auto end      = std::chrono::high_resolution_clock::now();
    auto duration = end - start;
    return duration.count() / (1000.0 * 1000.0);
}

void ScopedTimer::traverse(Json& j)
{
    j["name"]     = name;
    j["duration"] = duration.count();
    j["children"] = Json::array();
    for(auto& child : children)
    {
        auto& child_json = j["children"].emplace_back();
        child->traverse(child_json);
    }
}

void ScopedTimer::setup_full_name()
{
    full_name = "/" + name;
    auto p    = parent;
    if(p)
        full_name = p->full_name + full_name;
}
}  // namespace uipc::details

namespace uipc
{
bool Timer::m_global_on = false;

std::function<void()> Timer::m_sync;

Timer::Timer(std::string_view blockName, bool force_on)
    : m_force_on(force_on)
{
    if(!GlobalTimer::current())
        return;
    if(!m_global_on && !m_force_on)
        return;

    sync();

    auto& t = GlobalTimer::current()->push_timer(blockName);
    m_timer = &t;
    t.tick();
}

void Timer::report(std::ostream& o)
{
    if(!GlobalTimer::current())
    {
        spdlog::warn("No timing information to report.");
        return;
    }

    GlobalTimer::current()->print_merged_timings(o);
    GlobalTimer::current()->clear();
}

Json Timer::report_as_json()
{
    if(!GlobalTimer::current())
    {
        spdlog::warn("No timing information to report.");
        return Json::object();
    }
    Json json = GlobalTimer::current()->report_merged_as_json();
    GlobalTimer::current()->clear();
    return json;
}

double Timer::elapsed() const
{
    sync();

    if(m_timer)
        return m_timer->elapsed();
    else
        return -1.0;
}

void Timer::sync() const
{
    if(m_sync)
        m_sync();
}

Timer::~Timer()
{
    if(!GlobalTimer::current())
        return;
    if(!m_global_on && !m_force_on)
        return;
    sync();
    auto& t = GlobalTimer::current()->pop_timer();
    t.tock();
}

GlobalTimer GlobalTimer::default_instance;

GlobalTimer* GlobalTimer::m_current = nullptr;

auto GlobalTimer::push_timer(std::string_view name) -> STimer&
{
    auto& u  = m_timers.emplace_back(new STimer{name});
    u->depth = m_timer_stack.size();
    m_timer_stack.top()->children.push_back(u.get());
    u->parent = m_timer_stack.top();
    u->setup_full_name();

    m_timer_stack.push(u.get());
    return *u;
}

auto GlobalTimer::pop_timer() -> STimer&
{
    auto& top = m_timer_stack.top();
    m_timer_stack.pop();
    return *top;
}

GlobalTimer::GlobalTimer(std::string_view name)
{
    auto& u  = m_timers.emplace_back(new STimer{name});
    u->depth = 0;
    u->setup_full_name();
    m_root = u.get();
    m_timer_stack.push(m_root);
}

GlobalTimer::~GlobalTimer()
{
    if(m_current == this)
        m_current = nullptr;
}

void GlobalTimer::set_as_current()
{
    if(m_current)
    {
        UIPC_ASSERT(m_current->m_timer_stack.size() == 1,
                    "The last GlobalTimer is not finished! Still {} Timer in the Timer Stack",
                    m_current->m_timer_stack.size());
    }
    m_current = this;
}

GlobalTimer* GlobalTimer::current()
{
    if(!m_current)
        m_current = &default_instance;
    return m_current;
}

Json GlobalTimer::report_as_json()
{
    Json json;
    for(auto timer : m_root->children)
    {
        auto& j = json.emplace_back();
        timer->traverse(j);
    }
    return json;
}

Json GlobalTimer::report_merged_as_json()
{
    merge_timers();
    Json json;
    _traverse_merge_timers(json, m_merge_root);
    return json;
}

void GlobalTimer::_print_timings(std::ostream& o, const STimer* timer, int depth)
{
    for(int i = 0; i < depth; ++i)
        o << "    ";
    o << timer->name << ": " << timer->duration.count() * 1000 << "ms" << std::endl;
    for(auto child : timer->children)
    {
        _print_timings(o, child, depth + 1);
    }
}

void GlobalTimer::print_timings(std::ostream& o)
{
    o << R"(
=====================================================================================
                         TIMING BREAKDOWN: ALL TIMERS
=====================================================================================
)";
    for(auto timer : m_root->children)
    {
        _print_timings(o, timer, 0);
    }
}

void GlobalTimer::print_merged_timings(std::ostream& o)
{
    merge_timers();

    auto max_name_length = this->max_full_name_length();
    auto max_depth       = this->max_depth();

    o << R"(
=====================================================================================
                         TIMING BREAKDOWN: MERGED TIMERS
=====================================================================================
)";
    _print_merged_timings(o, m_merge_root, max_name_length, max_depth);
}

void GlobalTimer::clear()
{
    auto timer_names = [&]
    {
        vector<string> names;
        stack<STimer*> stack = m_timer_stack;
        while(stack.size() > 1)
        {
            names.push_back(fmt::format("* {}", stack.top()->name));
            stack.pop();
        }
        return names;
    };

    UIPC_ASSERT(m_timer_stack.size() == 1,
                "Are you calling clear() in the Timer Scope? Current stack:\n{}",
                fmt::join(timer_names(), "\n"));

    string name = m_timers.front()->name;
    m_timers.clear();
    auto& u  = m_timers.emplace_back(new STimer{name});
    u->depth = 0;
    u->setup_full_name();
    m_timer_stack = stack<STimer*>();
    m_root        = u.get();
    m_timer_stack.push(m_root);
}

size_t GlobalTimer::max_full_name_length() const
{
    auto elem =
        std::max_element(m_timers.begin(),
                         m_timers.end(),
                         [](const auto& a, const auto& b)
                         { return a->full_name.size() < b->full_name.size(); });
    return (*elem)->full_name.size();
}

size_t GlobalTimer::max_depth() const
{
    auto elem = std::max_element(m_timers.begin(),
                                 m_timers.end(),
                                 [](const auto& a, const auto& b)
                                 { return a->depth < b->depth; });
    return (*elem)->depth;
}

void GlobalTimer::merge_timers()
{
    m_merge_timers.clear();
    m_merge_root = nullptr;

    // merge timers
    for(auto& timer : m_timers)
    {
        auto iter = m_merge_timers.find(timer->full_name);

        if(iter == m_merge_timers.end())
        {
            auto& new_merged_timer = m_merge_timers[timer->full_name];
            new_merged_timer       = make_unique<MergeResult>();

            new_merged_timer->name = timer->name;
            if(timer->parent)
            {
                new_merged_timer->parent_full_name = timer->parent->full_name;
                new_merged_timer->parent_name      = timer->parent->name;
            }
            else
            {
                new_merged_timer->parent_full_name = "";
                new_merged_timer->parent_name      = "";
                m_merge_root                       = new_merged_timer.get();
            }
            new_merged_timer->duration = timer->duration.count();
            new_merged_timer->count    = 1;
            new_merged_timer->depth    = timer->depth;
        }
        else
        {
            iter->second->duration += timer->duration.count();
            iter->second->count++;
        }
    }

    // do_build hierarchy
    for(auto&& [name, merged_timer] : m_merge_timers)
    {
        if(merged_timer->parent_full_name.empty())
            continue;

        auto iter = m_merge_timers.find(merged_timer->parent_full_name);
        if(iter != m_merge_timers.end())
        {
            iter->second->children.push_back(merged_timer.get());
        }
    }

    // sort children
    for(auto&& [name, merged_timer] : m_merge_timers)
    {
        merged_timer->children.sort([](const MergeResult* a, const MergeResult* b)
                                    { return a->duration > b->duration; });
    }
}
void GlobalTimer::_print_merged_timings(std::ostream&      o,
                                        const MergeResult* timer,
                                        size_t             max_name_length,
                                        size_t             max_depth)
{

    const auto    depth     = timer->depth;
    constexpr int tab_width = 4;
    constexpr int precision = 12;

    size_t parent_full_name_length = timer->parent_full_name.size();

    o << string(parent_full_name_length, ' ') << "*";
    o << std::setw(max_name_length - parent_full_name_length) << std::left
      << timer->name << " | ";

    if(depth == 0)
    {
        o << std::setw(precision + 3) << std::right << "Time Cost"
          << " | " << std::setw(precision) << std::right << "Total Count" << std::endl;
    }
    else
    {
        o << std::setw(precision) << std::right << std::setprecision(precision)
          << timer->duration * 1000 << " ms";
        o << " | " << std::setw(precision) << std::right << timer->count << std::endl;
    }


    for(auto child : timer->children)
    {
        _print_merged_timings(o, child, max_name_length, max_depth);
    }
}

void GlobalTimer::_traverse_merge_timers(Json& j, const MergeResult* timer)
{
    j["name"]     = timer->name;
    j["duration"] = timer->duration;
    j["count"]    = timer->count;
    j["children"] = Json::array();
    j["parent"]   = timer->parent_full_name;
    for(auto& child : timer->children)
    {
        auto& child_json = j["children"].emplace_back();
        _traverse_merge_timers(child_json, child);
    }
}
}  // namespace uipc