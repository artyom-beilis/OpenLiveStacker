#pragma once
#include <cstddef>
namespace ols {
    class AllocatorGuard {
    public:
        AllocatorGuard(bool enable);
        ~AllocatorGuard();
        AllocatorGuard(AllocatorGuard const &) = delete;
        void operator=(AllocatorGuard const &) = delete;
        static size_t allocated();    
    };
};
