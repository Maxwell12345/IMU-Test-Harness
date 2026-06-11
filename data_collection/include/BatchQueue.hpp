#ifndef INU_DISPLAY_BATCHQUEUE_HPP
#define INU_DISPLAY_BATCHQUEUE_HPP

#include "Logger.hpp"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

template <typename T>
class BatchQueue {
public:
    using Sink = std::function<void(std::vector<T>&)>;

    BatchQueue(std::string name, std::size_t batchSize, Sink sink,
               std::chrono::milliseconds flushInterval = std::chrono::milliseconds(1000))
        : m_name(std::move(name)),
          m_batchSize(batchSize),
          m_flushInterval(flushInterval),
          m_sink(std::move(sink)) {
        m_staging.reserve(batchSize);
    }

    ~BatchQueue() { stop(); }

    BatchQueue(const BatchQueue&) = delete;
    BatchQueue& operator=(const BatchQueue&) = delete;

    void start() {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_running) {
            return;
        }
        m_running = true;
        m_stopRequested = false;
        m_worker = std::thread(&BatchQueue::run, this);
        LOG_DEBUG("BatchQueue") << m_name << ": worker started (batchSize=" << m_batchSize
                                << ", flushInterval=" << m_flushInterval.count() << "ms)";
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (!m_running) {
                return;
            }
            m_stopRequested = true;
        }
        m_cv.notify_all();
        if (m_worker.joinable()) {
            m_worker.join();
        }
        std::lock_guard<std::mutex> lock(m_mutex);
        m_running = false;
        LOG_DEBUG("BatchQueue") << m_name << ": worker stopped";
    }

    void push(const T& record) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_staging.push_back(record);
            if (m_staging.size() < m_batchSize) {
                return;
            }
            promoteStagingLocked();
        }
        m_cv.notify_one();
    }

private:
    void run() {
        std::unique_lock<std::mutex> lock(m_mutex);
        for (;;) {
            m_cv.wait_for(lock, m_flushInterval,
                          [this] { return !m_ready.empty() || m_stopRequested; });

            promoteStagingLocked();

            while (!m_ready.empty()) {
                std::vector<T> batch;
                batch.swap(m_ready.front());
                m_ready.pop_front();

                lock.unlock();
                flush(batch);
                lock.lock();
            }

            if (m_stopRequested) {
                return;
            }
        }
    }

    void promoteStagingLocked() {
        if (m_staging.empty()) {
            return;
        }
        std::vector<T> full;
        full.reserve(m_batchSize);
        full.swap(m_staging);
        m_ready.push_back(std::move(full));
    }

    void flush(std::vector<T>& batch) {
        if (batch.empty()) {
            return;
        }
        try {
            m_sink(batch);
        } catch (const std::exception& e) {
            LOG_ERROR("BatchQueue") << m_name << ": flush failed, dropping " << batch.size()
                                    << " records: " << e.what();
        }
    }

    std::string m_name;
    std::size_t m_batchSize;
    std::chrono::milliseconds m_flushInterval;
    Sink m_sink;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::vector<T> m_staging;
    std::deque<std::vector<T>> m_ready;
    std::thread m_worker;
    bool m_running{false};
    bool m_stopRequested{false};
};

#endif
