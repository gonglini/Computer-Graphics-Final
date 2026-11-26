#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

// Minimal persistent thread pool.
// The LiDAR fires a few thousand rays every 10 ms simulation step. Creating and joining fresh
// std::threads for every step (hundreds of times per second) disturbed the GPU driver's own
// threads and made frames stall. Here the workers are created once and wait on a condition
// variable between jobs.
class ThreadPool
{
public:
    explicit ThreadPool(int workers)
    {
        for (int i = 0; i < workers; ++i)
            m_threads.emplace_back([this, i] { loop(i + 1); });
    }

    ~ThreadPool()
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_quit = true;
        }
        m_wake.notify_all();
        for (auto& t : m_threads)
            t.join();
    }

    int size() const { return static_cast<int>(m_threads.size()) + 1; }   // workers + caller

    // Runs job(part, parts) for part = 0 .. parts-1, parts = size(); the caller does part 0.
    // Blocks until every part has finished.
    void run(const std::function<void(int part, int parts)>& job)
    {
        const int parts = size();
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_job = &job;
            m_pending = parts - 1;
            ++m_generation;
        }
        m_wake.notify_all();

        job(0, parts);

        std::unique_lock<std::mutex> lock(m_mutex);
        m_done.wait(lock, [this] { return m_pending == 0; });
        m_job = nullptr;
    }

private:
    void loop(int part)
    {
        unsigned seen = 0;
        for (;;)
        {
            const std::function<void(int, int)>* job = nullptr;
            int parts = 0;
            {
                std::unique_lock<std::mutex> lock(m_mutex);
                m_wake.wait(lock, [&] { return m_quit || m_generation != seen; });
                if (m_quit)
                    return;
                seen = m_generation;
                job = m_job;
                parts = static_cast<int>(m_threads.size()) + 1;
            }
            (*job)(part, parts);
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                --m_pending;
            }
            m_done.notify_one();
        }
    }

    std::vector<std::thread> m_threads;
    std::mutex m_mutex;
    std::condition_variable m_wake, m_done;
    const std::function<void(int, int)>* m_job = nullptr;
    int m_pending = 0;
    unsigned m_generation = 0;
    bool m_quit = false;
};
