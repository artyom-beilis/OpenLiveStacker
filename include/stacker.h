#pragma once
namespace ols {
    class Stacker;
    class StackerWrapper {
    public:
        StackerWrapper( queue_pointer_type input_queue,
                        queue_pointer_type video_queue,
                        queue_pointer_type save_data_queue);
        ~StackerWrapper();
        StackerWrapper(StackerWrapper const &) = delete;
        StackerWrapper(StackerWrapper &&) = delete;
        void operator=(StackerWrapper const &) =delete;
        void operator=(StackerWrapper &&) =delete;
        void run();

    private:
        queue_pointer_type input_;
        queue_pointer_type video_;
        queue_pointer_type save_;
        std::unique_ptr<Stacker> stacker_;
    };

}
