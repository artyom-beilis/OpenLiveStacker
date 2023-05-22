#pragma once
#include "data_items.h"
#include <thread>
namespace ols {
    std::thread start_preprocessor(queue_pointer_type in,
                                   queue_pointer_type out,
                                   queue_pointer_type error_queue);
    std::thread start_stacker(queue_pointer_type in,
                              queue_pointer_type out,
                              queue_pointer_type stats_and_error,
                              queue_pointer_type plate_solving_output,
                              std::string data_dir);
    std::thread start_debug_saver(queue_pointer_type in,queue_pointer_type error_queue,std::string debug_dir);
}
