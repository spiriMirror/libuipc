#pragma once
#include <chrono>
#include <iomanip>
#include <iostream>
#include <uipc/common/vector.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/stack.h>
#include <uipc/common/string.h>
#include <uipc/common/list.h>
#include <uipc/common/string.h>
#include <uipc/common/json.h>
#include <uipc/common/dllexport.h>
#include <uipc/common/unordered_map.h>
#include <uipc/common/set.h>
#include <uipc/common/string.h>
#include <functional>

namespace uipc
{
class GlobalTimer;
class Timer;
}  // namespace uipc

namespace uipc::details
{
class UIPC_CORE_API ScopedTimer
{
  public:
    using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
    using Duration = std::chrono::duration<double>;

  private:
    friend class uipc::GlobalTimer;
    friend class uipc::Timer;
    string             name;
    string             full_name;
    TimePoint          start;
    TimePoint          end;
    Duration           duration;
    list<ScopedTimer*> children;
    ScopedTimer*       parent = nullptr;
    size_t             depth  = 0;

    ScopedTimer(std::string_view name)
        : name(name)
        , duration(0)
    {
    }

    void   tick();
    void   tock();
    double elapsed() const;
    void   traverse(Json& j);
    void   setup_full_name();

  public:
    ~ScopedTimer() = default;
};
}  // namespace uipc::details

namespace uipc
{
class UIPC_CORE_API Timer
{
  public:
    Timer(std::string_view blockName, bool force_on = false);
    ~Timer();

    double      elapsed() const;
    static void disable_all() { m_global_on = false; }
    static void enable_all() { m_global_on = true; }
    static void set_sync_func(std::function<void()> sync) { m_sync = sync; }

    static void report(std::ostream& o = std::cout);
    static Json report_as_json();

  private:
    void                         sync() const;
    details::ScopedTimer*        m_timer = nullptr;
    bool                         m_force_on;
    static bool                  m_global_on;
    static std::function<void()> m_sync;
};

class UIPC_CORE_API GlobalTimer
{
    using STimer = details::ScopedTimer;

    stack<STimer*> m_timer_stack;
    STimer*        m_root;
    friend class ScopedTimer;

    list<U<STimer>> m_timers;

    friend class Timer;
    STimer& push_timer(std::string_view);
    STimer& pop_timer();

    static GlobalTimer  default_instance;
    static GlobalTimer* m_current;

    void _print_timings(std::ostream& o, const STimer* timer, int depth);

    size_t max_full_name_length() const;
    size_t max_depth() const;

    struct MergeResult
    {

        string name;
        string parent_full_name;
        string parent_name;

        double                  duration = 0.0;
        size_t                  count    = 0;
        std::list<MergeResult*> children;

        size_t depth = 0;
    };

    unordered_map<string, U<MergeResult>> m_merge_timers;
    MergeResult*                          m_merge_root = nullptr;

    void merge_timers();
    void _print_merged_timings(std::ostream&      o,
                               const MergeResult* timer,
                               size_t             max_name_length,
                               size_t             max_depth);
    void _traverse_merge_timers(Json& j, const MergeResult* timer);

  public:
    GlobalTimer(std::string_view name = "GlobalTimer");

    // delete copy_from
    GlobalTimer(const GlobalTimer&)            = delete;
    GlobalTimer& operator=(const GlobalTimer&) = delete;

    ~GlobalTimer();

    void                set_as_current();
    static GlobalTimer* current();
    Json                report_as_json();
    Json                report_merged_as_json();

    void print_timings(std::ostream& o = std::cout);

    void print_merged_timings(std::ostream& o = std::cout);

    void clear();
};
}  // namespace uipc
