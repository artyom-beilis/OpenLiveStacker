#pragma once
#include "data_items.h"
#include <thread>
namespace ols {
    std::thread start_preprocessor(queue_pointer_type in,
                                   queue_pointer_type out,
                                   queue_pointer_type error_queue);

    std::thread start_stacker(queue_pointer_type in,
                              queue_pointer_type out,
                              queue_pointer_type stats_and_error);
    
    std::thread start_post_processor(queue_pointer_type in,
                                     queue_pointer_type out,
                                     queue_pointer_type plate_solving_output,
                                     queue_pointer_type stats_and_error,
                                     std::string data_dir);

    std::thread start_debug_saver(queue_pointer_type in,queue_pointer_type error_queue,std::string debug_dir);

    inline void send_message(queue_pointer_type q,std::string const &id,std::string const &err)
    {
        if(q) {
            std::shared_ptr<ErrorNotificationData> p(new ErrorNotificationData());
            p->source = id;
            p->message = err;
            q->push(p);
        }
    }

    inline void send_message(queue_pointer_type q,std::string const &id,std::exception const &e)
    {
        send_message(q,id,e.what());
    }
}
