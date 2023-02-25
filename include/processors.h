#pragma once
#include "data_items.h"
#include <thread>
namespace ols {
    std::thread start_preprocessor(queue_pointer_type in,queue_pointer_type out);
    std::thread start_stacker(queue_pointer_type in,queue_pointer_type out,std::string data_dir);
    std::thread start_debug_saver(queue_pointer_type in,std::string debug_dir);
}
