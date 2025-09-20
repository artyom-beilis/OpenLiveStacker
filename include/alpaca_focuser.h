#pragma once
#include "focuser.h"
namespace ols {
    std::unique_ptr<Focuser> load_alpaca_focuser(std::string const &opt,FocuserErrorCode &e);
}
