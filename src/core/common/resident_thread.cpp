#include <uipc/common/resident_thread.h>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>

namespace uipc
{
class ResidentThread::Impl
{
  public:
    Impl()
        : stop_flag(false)
    {
        worker = std::thread([this] { this->run(); });
    }

    ~Impl()
    {
        {
            std::unique_lock<std::mutex> lock(mtx);
            stop_flag = true;
            cv.notify_all();
        }
        worker.join();
    }

    bool post(std::function<void()> task)
    {
        bool success = false;
        {
            std::unique_lock<std::mutex> lock(mtx);
            if(waiting_task)
            {
                success = false;
            }
            else
            {
                success      = true;
                waiting_task = std::move(task);
                cv.notify_one();
            }
        }

        return success;
    }

    std::size_t hash() const
    {
        return std::hash<std::thread::id>()(worker.get_id());
    }

    bool is_ready() const { return done; }

  private:
    void run()
    {
        while(true)
        {
            std::function<void()> local;
            {
                std::unique_lock<std::mutex> lock(mtx);
                cv.wait(lock,
                        [this] { return waiting_task != nullptr || stop_flag; });
                if(stop_flag && waiting_task == nullptr)
                    break;
                local        = std::move(waiting_task);
                waiting_task = nullptr;
            }

            if(local)
            {
                done = false;
                local();
                done = true;
            }
            else
            {
                done = true;
            }
        }
    }

    std::thread             worker;
    std::function<void()>   waiting_task;
    mutable std::mutex      mtx;
    std::condition_variable cv;
    bool                    stop_flag;
    std::atomic_bool        done{true};
};

ResidentThread::ResidentThread()
    : m_impl(uipc::make_shared<Impl>())
{
}

SizeT ResidentThread::hash() const
{
    return m_impl->hash();
}

bool ResidentThread::post(std::function<void()> task)
{
    return m_impl->post(task);
}

bool ResidentThread::is_ready() const
{
    return m_impl->is_ready();
}

ResidentThread::~ResidentThread() {}
}  // namespace uipc