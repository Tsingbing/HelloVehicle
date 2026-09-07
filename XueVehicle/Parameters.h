#pragma once

#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Param/AP_Param.h>

class Xue {
public:
    AP_Float gain;
    AP_Float speed;
    AP_Int8 enable;
    AP_Int32 test;
    AP_Int8 mode;
    AP_Float manual;

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
        k_param_ins = 5,
        k_param_ahrs = 6,
        k_param_compass = 7,
        k_param_scheduler = 8,
        k_param_barometer = 9,
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
