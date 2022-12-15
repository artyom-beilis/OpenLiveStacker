#include "data_item.h"
namespace ols {
    class PreProcessor {
    public:
        PreProcessor(queue_pointer_type in,queue_pointer_type out) :
            in_(in),out_(out)
        {
        }
        void run()
        {
            while(true) {
                auto data_ptr = in_->pop();
                auto stop_ptr = std::dynamic_pointer_cast<ShutDownData>(data_ptr);
                if(stop_ptr) {
                    out_->push(data_ptr);
                    break;
                }
            }
        }
    private:
        cv::Mat darks_;
        bool apply_darks_;
    };


}
