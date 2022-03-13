#ifndef EVENT_QUEUE_H_
#define EVENT_QUEUE_H_

#include <memory>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include "lang-feature-extension/self_tag.h"
#include "lang-feature-extension/attr_reader.h"
#include "lang-feature-extension/disable_constructor.h"

namespace fsm {
    template<class T>
    class EventQueue : NO_COPY, NO_MOVE {
    public:
        EventQueue();

        ~EventQueue();

        template<class U>
        void Add(U &&msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.emplace_back(std::forward<U>(msg));
            condition_.notify_one();
        }

        T Next();

        T Next(unsigned int timeout);

        void Clear();

        bool MessageAvailable() const;

        [[maybe_unused]] bool Empty() const;

        [[maybe_unused]] unsigned int Size() const;

    private:
        using FifoQueueType = std::deque<T>;
        FifoQueueType queue_;

        mutable std::mutex mutex_;
        std::condition_variable condition_;
    };

    template<class T>
    EventQueue<T>::EventQueue() : queue_() {}

    template<class T>
    EventQueue<T>::~EventQueue() { Clear(); }

    template<class T>
    [[maybe_unused]] bool EventQueue<T>::Empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    template<class T>
    [[maybe_unused]] unsigned int EventQueue<T>::Size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<unsigned int>(queue_.size());
    }

    template<class T>
    void EventQueue<T>::Clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        FifoQueueType().swap(queue_);
        condition_.notify_all();
    }

    template<class T>
    T EventQueue<T>::Next() {
        std::unique_lock<std::mutex> lk(mutex_);
        condition_.wait(lk, [this]() {
            return !queue_.empty();
        });

        T first_message(std::move(queue_.front()));
        queue_.pop_front();

        return first_message;
    }

    template<class T>
    T EventQueue<T>::Next(unsigned int timeout) {
        T first_message;
        auto now = std::chrono::system_clock::now();
        std::unique_lock<std::mutex> lock(mutex_);
        bool signaled = condition_.wait_until(lock,
                                              now + std::chrono::milliseconds(timeout),
                                              [this]() { return !queue_.empty(); }
        );
        if (signaled) {
            first_message = std::move(queue_.front());
            queue_.pop_front();
        }
        return first_message;
    }

    template<class T>
    bool EventQueue<T>::MessageAvailable() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return !queue_.empty();
    }
}

#endif  // EVENT_QUEUE_H_
