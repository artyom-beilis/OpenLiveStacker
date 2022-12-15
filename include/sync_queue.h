#pragma once
#include <condition_variable>
#include <mutex>
#include <queue>
#include <memory>
#include <thread>

namespace ols {
    template<typename T>
    class sync_queue {
    public:

        typedef std::function<void(T)> callback_type;

        void call_on_push(std::function<void(T)> cb)
        {
            std::unique_lock<std::mutex> guard(lock_);
            cb_ = cb;
            if(cb_) {
                while(!data_.empty()) {
                    auto item = data_.front();
                    cb_(item);
                    data_.pop();
                }
            }
        }

        void push(T const &v)
        {
            std::unique_lock<std::mutex> guard(lock_);
            if(cb_) {
                cb_(v);
                return;
            }
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
        callback_type cb_;
        std::mutex lock_;
    };

}
