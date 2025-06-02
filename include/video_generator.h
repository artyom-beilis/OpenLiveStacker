#pragma once
#include "data_items.h"
#include <thread>

namespace ols {
    std::thread start_generator(queue_pointer_type input,
                                queue_pointer_type stacking_output,
                                queue_pointer_type live_output,
                                queue_pointer_type debug_save,
                                queue_pointer_type plate_solving_output,
                                queue_pointer_type guiding_output);
}
