#pragma once
#include <condition_variable>
#include <mutex>
#include <queue>
#include <memory>

namespace ols {
    template<typename T>
    class sync_queue {
    public:

        void push(T const &v)
        {
            std::unique_lock<std::mutex> guard(lock_);
            data_.push(v);
            cond_.notify_one();
        }

        T pop()
        {
            std::unique_lock<std::mutex> guard(lock_);
            while(data_.empty()) {
                cond_.wait(guard);
            }
            T res = data_.front();
            data_.pop();
            return res;
        }

    private:
        std::queue<T> data_;
        std::condition_variable cond_;
        std::mutex lock_;
    };

}
