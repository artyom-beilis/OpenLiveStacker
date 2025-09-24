#include <iostream>
#include <sstream>
#include <string>
#include <mutex>
#include <thread>

#include "camera.h"
#include "focuser_ctl.h"
#include "focuser.h"


namespace ols {

FocuserControlApp::~FocuserControlApp()
{
}

FocuserControlApp::guard_type FocuserControlApp::guard()
{
    return FocuserInterface::instance().guard();
}

void FocuserControlApp::drivers()
{
    auto drivers = FocuserInterface::instance().drivers_list();
    for(size_t i=0;i<drivers.size();i++) {
        response_["drivers"][i]["name"] = drivers[i].name;
        response_["drivers"][i]["opt"] = drivers[i].opt;
    }
}

void FocuserControlApp::load()
{
    std::string driver = content_.get<std::string>("driver");
    std::string option = content_.get<std::string>("opt");
    FocuserInfo info = {driver,option};
    FocuserErrorCode e;
    FocuserInterface::instance().load_driver(info,e);
    e.check();
}

void FocuserControlApp::move_to()
{
    guard_type g(guard());
    Focuser &f = focuser();
    FocuserErrorCode e;
    f.move_to(content_.get<int>("value"),e);
    e.check();
}

void FocuserControlApp::move_by()
{
    guard_type g(guard());
    Focuser &f = focuser();
    FocuserErrorCode e;
    f.move_by(content_.get<int>("value"),e);
    e.check();
}
Focuser &FocuserControlApp::focuser()
{
    Focuser *f=FocuserInterface::instance().focuser();
    if(!f)
        throw FocuserError("Focuser driver is not loaded");
    return *f;
}

void FocuserControlApp::abort()
{
    guard_type g(guard());
    Focuser &f = focuser();
    FocuserErrorCode e;
    f.abort(e);
    e.check();
}


void FocuserControlApp::get_status()
{
    guard_type g(guard());
    Focuser *f = FocuserInterface::focuser();
    if(!f) {
        response_["connected"] = false;
        return;
    }
    response_["connected"] = true;
    FocuserErrorCode e;
    response_["position"] = f->position(e);
    e.check();
    response_["is_moving"] = f->is_moving(e);
    e.check();
    response_["is_absolute"] = f->is_absolute(e);
    e.check();
    response_["max_range"] = f->max_range(e);
    e.check();
}


} // namespace

