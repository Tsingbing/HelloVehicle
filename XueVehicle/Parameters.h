#pragma once

#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Param/AP_Param.h>

class Xue {
public:
    AP_Float gain;
    AP_Float speed;
    AP_Int8 enable;
    AP_Int32 test;

    static const AP_Param::GroupInfo var_info[];
};

class Parameters {
public:
    enum : uint16_t {
        k_param_format_version = 0,
        k_param_log_bitmask = 1,
        k_param_g2 = 2,
        k_param_logger = 3,
        k_param_BoardConfig = 4,
    };

    AP_Int16 format_version;
    AP_Int32 log_bitmask;
};

class ParametersG2 {
public:
    ParametersG2();

    Xue xue;
#if AP_AIRSPEED_ENABLED
    AP_Airspeed airspeed;
#endif

    static const AP_Param::GroupInfo var_info[];
};
