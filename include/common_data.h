#pragma once
namespace ols {
    enum StackMethod {
        stack_calibration,
        stack_dso,
        stack_planetary,
        stack_dynamic
    };
    struct StretchInfo {
        double cut = 0;
        double gain = 1.0;
        double gamma = 2.2;
        bool auto_stretch = true;
    };
}
