#pragma once
#include <condition_variable>
#include <mutex>
#include <queue>
#include <memory>
#include <thread>
#include <functional>
#include <atomic>

namespace ols {
    struct sync_queue_base {
        static std::atomic<long> items;
    };
    template<typename T>
    class sync_queue : public sync_queue_base {
    public:
    

        sync_queue(size_t limit = std::numeric_limits<size_t>::max()) : limit_(limit)
        {
        }

        typedef std::function<void(T)> callback_type;

        void call_on_push(std::function<void(T)> cb)
        {
            std::unique_lock<std::mutex> guard(lock_);
            cb_ = cb;
            if(cb_) {
                while(!data_.empty()) {
                    auto item = data_.front();
                    data_.pop();
                    --items;
                    cb_(item);
                }
            }
        }

        template<typename ItemType>
        void push_or_replace(std::shared_ptr<ItemType> p)
        {
            std::unique_lock<std::mutex> guard(lock_);
            if(cb_) {
                cb_(p);
                return;
            }
            if(!data_.empty()) {
                auto &last = data_.back();
                // replace last item if exactly the same type
                if(std::dynamic_pointer_cast<ItemType>(last)) {
                    last = p;
                    return;
                }
            }
            while(data_.size() >= limit_) {
                cond_has_room_.wait(guard);
            }
            ++items;
            data_.push(p);
            cond_.notify_one();
        }
        size_t size() 
        {
            std::unique_lock<std::mutex> guard(lock_);
            return data_.size();
        }

        void push(T const &v)
        {
            std::unique_lock<std::mutex> guard(lock_);
            if(cb_) {
                cb_(v);
                return;
            }
            while(data_.size() >= limit_) {
                cond_has_room_.wait(guard);
            }
            ++items;
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
            --items;
            cond_has_room_.notify_one();
            return res;
        }

    private:
        size_t limit_;
        std::queue<T> data_;
        std::condition_variable cond_;
        std::condition_variable cond_has_room_;
        callback_type cb_;
        std::mutex lock_;
    };

}
