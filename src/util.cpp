#include "util.h"
#include "sync_queue.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <system_error>
#include <dirent.h>
#include <sys/inotify.h>
#include <poll.h>
#include <string.h>


namespace ols {
    std::atomic<long> sync_queue_base::items;

    void make_dir(std::string const &path)
    {
        int r = mkdir(path.c_str(),0777);
        if(r < 0) {
            int e=errno;
            if(e == EEXIST)
                return;
            throw std::system_error(e,std::generic_category(),"mkdir failed");
        }
    }
    bool exists(std::string const &path)
    {
        return access(path.c_str(),F_OK) == 0;
    }
    std::string ftime(std::string const &pattern,time_t ts)
    {
        char buf[256];
        strftime(buf,sizeof(buf),pattern.c_str(),localtime(&ts));
        return buf;
    }

#ifdef __linux__
    struct DirWatch::data {
        int data_size = 0;
        union INotifyUnion {
            char buf[sizeof(struct inotify_event) + NAME_MAX + 1];
            struct inotify_event event;
        } ev;
    };

    DirWatch::DirWatch(std::string const &dir_name) : 
        dir_(dir_name),
        d(new DirWatch::data())
    {
        fd_ = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
        if(fd_ < 0)
            throw std::system_error(errno,std::generic_category(),"inotify failed");
        int res = inotify_add_watch(fd_,dir_.c_str(),IN_CLOSE_WRITE | IN_MOVED_TO);
        if(res < 0) {
            close(fd_);
            throw std::system_error(errno,std::generic_category(),"inotify add_watch failed on " + dir_ + " directory");
        }
    }
    DirWatch::~DirWatch()
    {
        close(fd_);
    }
    bool DirWatch::has_enough()
    {
        return d->data_size >= int(sizeof(struct inotify_event)) 
               && d->data_size >= (d->data_size >= int(d->ev.event.len + sizeof(struct inotify_event)));
    }
    std::string DirWatch::wait_for_new_file(int time_ms)
    {
        if(!has_enough()) {
            struct pollfd pfd;
            pfd.fd = fd_;
            pfd.events = POLLIN;
            int r = poll(&pfd,1,time_ms);
            if(r < 0) {
                int e = errno;
                if(e == EINTR)
                    return "";
                throw std::system_error(errno,std::generic_category(),"watch failed failed on " + dir_ + " directory: poll");
            }
            if(r == 0) {
                return "";
            }
            if(pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                throw std::runtime_error("Failed to read inotify on " + dir_);
            }
            if(!(r & POLLIN))
                return "";
            size_t reminder = sizeof(d->ev) - d->data_size;
            char *buf = d->ev.buf + d->data_size;
            int got_bytes = read(fd_,buf,reminder);
            if(got_bytes < 0) {
                int e = errno;
                if(e == EAGAIN || e == EINTR)
                    return "";
                throw std::system_error(errno,std::generic_category(),"read watch failed failed on " + dir_ + " directory: poll");
            }
            if(got_bytes == 0) {
                throw std::runtime_error("Unexpected EOF " + dir_);
            }
            d->data_size += got_bytes;
        }
        if(!has_enough())
            return "";
        std::string name = dir_ + "/" + d->ev.event.name;
        int consumed = d->ev.event.len + sizeof(struct inotify_event);
        if(consumed == d->data_size) {
            d->data_size = 0;
        }
        else {
            memmove(d->ev.buf,d->ev.buf + consumed,d->data_size - consumed);
            d->data_size = d->data_size = consumed;
        }
        return name;
    }
#endif    
}
