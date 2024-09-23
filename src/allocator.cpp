#include "allocator.h"
#include <opencv2/core.hpp>
#include <atomic>

namespace ols {

static cv::MatAllocator *stda = nullptr;
static std::atomic<size_t> memory_use;

class MyAlloc : public cv::MatAllocator
{
public:
    virtual cv::UMatData *  allocate (int dims, const int *sizes, int type, void *data, size_t *step, cv::AccessFlag flags, cv::UMatUsageFlags usageFlags) const override
    {
        cv::UMatData *p = stda->allocate(dims,sizes,type,data,step,flags,usageFlags);
        p->currAllocator = this;
        memory_use += p->size;
        return p;
    }
    virtual bool allocate (cv::UMatData *data, cv::AccessFlag accessflags, cv::UMatUsageFlags usageFlags) const override 
    {
        memory_use += data->size;
        return stda->allocate(data,accessflags,usageFlags);
    }
    virtual void deallocate   (   cv::UMatData *  data    )   const override
    {
        memory_use -= data->size;
        stda->deallocate(data);
    }

};

static MyAlloc alloc_inst;

AllocatorGuard::AllocatorGuard(bool enable)
{
    if(enable) {
        stda = cv::Mat::getStdAllocator();
        cv::Mat::setDefaultAllocator(&alloc_inst);
    }
}
AllocatorGuard::~AllocatorGuard()
{
}

size_t AllocatorGuard::allocated()
{
    return memory_use;
}

} // namespace

