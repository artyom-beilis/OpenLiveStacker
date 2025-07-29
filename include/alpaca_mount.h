#pragma once
#include "mount.h"
namespace ols {
    std::unique_ptr<MountDriver> alpaca_start_driver(std::string const &opt,MountErrorCode &e);
}
