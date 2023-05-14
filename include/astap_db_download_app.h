#pragma once
#include <cppcms/application.h>
#include <cppcms/http_context.h>
#include <cppcms/http_response.h>
#include <cppcms/url_dispatcher.h>
#include <cppcms/service.h>
#include <booster/log.h>
#include <booster/aio/io_service.h>

#include "ctl_app.h"
#include "server_sent_events.h"
#include "util.h"
#include "downloader.h"

namespace ols {
    class AstapDBDownloadApp : public ControlAppBase {
    public:
        AstapDBDownloadApp(cppcms::service &srv,
                          std::string db_dir) :
            ControlAppBase(srv),
            data_dir_(db_dir)
        {
            dispatcher().map("POST","/download/?",&AstapDBDownloadApp::download,this);
            dispatcher().map("POST","/cancel/?",&AstapDBDownloadApp::cancel,this);
            dispatcher().map("GET","/?",&AstapDBDownloadApp::status,this);
            stream_ = sse::state_stream::create(srv.get_io_service());
            update_status(false);
        }
        virtual ~AstapDBDownloadApp()
        {
            try {
                cancel();
            }
            catch(...) {}
        }

        void cancel()
        {
            BOOSTER_INFO("stacker") << "Aborting download";
            if(thread_) {
                cancel_ = 1;
                thread_->join();
                thread_.reset();
            }
        }
        void download()
        {
            if(thread_) {
                response_["status"]="fail";
                response_["error"] = "Download in progress";
                return;
            }
            std::string id = content_.get<std::string>("db");
            if(check(id)) {
                response_["status"] = "fail";
                response_["error"] = "Already downloaded";
                return;
            }
            files_in_db_ = files_in_db(id);
            std::string url = content_.get<std::string>("url");
            cancel_ = 0;
            downloaded_files_ = 0;
            last_file_ = "";
            update_status(false);
            booster::intrusive_ptr<AstapDBDownloadApp> self(this);
            thread_.reset(new std::thread([=]() {
                std::string error;
                BOOSTER_INFO("stacker")<< "Downloading " << url << " to " << self->data_dir_;
                bool status = zip_download(url,data_dir_,error,[=](char const *file) ->bool {
                    if(self->cancel_ > 0) {
                        return false;
                    }
                    self->update(file);
                    return true;
                }); 
                BOOSTER_INFO("stacker") << "DONE " << status <<" :" << error << std::endl;
                self->set_completed(status,error);
            }));
        }
        void update(char const *file)
        {
            std::string name = file;
            booster::intrusive_ptr<AstapDBDownloadApp> self(this);
            service().get_io_service().post([=]() {
                self->notify_download(name);
            });

        }
        void notify_download(std::string const &name)
        {
            downloaded_files_ ++;
            last_file_ = name;
            update_status(false);
        }

        void update_status(bool finished,bool is_ok=true,std::string const &error=std::string())
        {
            std::ostringstream ss;
            cppcms::json::value v;
            v["completed"] = finished;
            if(finished) {
                v["success"] = is_ok;
                if(!is_ok)
                    v["error"] = error;
                else
                    v["error"] = "ok";
            }
            else {
                v["downloaded"]=downloaded_files_;
                v["expected"] = files_in_db_;
                v["last_file"] = last_file_;
            }
            ss << v;
            stream_->update(ss.str());
            if(finished) {
                if(thread_) {
                    thread_->join();
                    thread_.reset();
                }
            }
        }

        void set_completed(bool status,std::string error_message = std::string())
        {
            booster::intrusive_ptr<AstapDBDownloadApp> self(this);
            service().get_io_service().post([=]() {
                self->update_status(true,status,error_message);
            });
        }

        void main(std::string url)
        {
            if(url == "/progress") {
                stream_->accept(release_context());
                return;
            }
            ControlAppBase::main(url);
        }

        void status()
        {
            response_["downloading"] = bool(thread_);
            char const *dbs[] = {"d05","d20","d50","g05","w08"};
            int N = 0;
            response_["db"] = cppcms::json::array();
            for(char const *db:dbs) {
                if(check(db)) {
                    response_["db"][N] = db;
                    N++;
                }
            }
            response_["has_db"] = N > 0;
        }

        int files_in_db(std::string const &prefix)
        {
            char code = prefix.c_str()[0];
            int N = -1;
            switch(code) {
            case 'd':
                N = 1476;
                break;
            case 'g':
                N = 290;
                break;
            case 'w':
                N = 1;
                break;
            }
            return N;
        }

        bool check(std::string const &prefix)
        {
            char const *d_names[] = {
                "_2713.1476", "_1509.1476", "_1964.1476","_0802.1476","_3601.1476","_3407.1476"
            };
            char const *g_names[] = {
                "_0810.290","_0813.290", "_0924.290", "_1602.290", "_1801.290"
            };
            char const *w_names[] = { "_0101.001"};
            
            int N;
            char const **names;
            char code = prefix.c_str()[0];
            switch(code) {
            case 'd':
                names = d_names;
                N = sizeof(d_names)/sizeof(d_names[0]);
                break;
            case 'g':
                names = g_names;
                N = sizeof(g_names)/sizeof(g_names[0]);
                break;
            case 'w':
                names = w_names;
                N = sizeof(w_names)/sizeof(w_names[0]);
                break;
            default:
                return false;
            }

            for(int i=0;i<N;i++) {
                char const *name = names[i];
                if(!exists(data_dir_ +"/" + prefix + name)) {
                    return false;
                }
            }
            return true;
        }


    private:
        std::string data_dir_;
        std::unique_ptr<std::thread> thread_;
        int downloaded_files_;
        int files_in_db_;
        std::string last_file_;
        std::atomic<int> cancel_;
        std::shared_ptr<sse::state_stream> stream_;
    };

};

