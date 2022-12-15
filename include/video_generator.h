#pragma once
#include "data_items.h"
#include <thread>

namespace ols {
    std::thread start_generator(queue_pointer_type input,queue_pointer_type output);
}
