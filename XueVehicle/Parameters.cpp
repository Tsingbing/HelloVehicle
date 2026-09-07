#include "Parameters.h"

const AP_Param::GroupInfo Xue::var_info[] = {
    // @Param: GAIN
    // @DisplayName: Hello gain
    // @Description: Proportional gain for the software speed controller
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("GAIN", 1, Xue, gain, 1.0f),

    // @Param: SPEED
    // @DisplayName: Hello speed
    // @Description: Target speed for the software speed controller
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("SPEED", 2, Xue, speed, 2.5f),

    // @Param: ENABLE
    // @DisplayName: Enable Hello
    // @Description: Enables drive output in the software speed controller
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 3, Xue, enable, 1),

    // @Param: TEST
    // @DisplayName: Hello test value
    // @Description: Integer value used by the Hello parameter test
    // @Range: -2147483648 2147483647
    // @User: Standard
    AP_GROUPINFO("TEST", 4, Xue, test, 42),

    // @Param: MODE
    // @DisplayName: Requested learning mode
    // @Description: Requested software controller mode; invalid requests retain the current mode
    // @Values: 0:STOP,1:MANUAL,2:AUTO
    // @User: Standard
    AP_GROUPINFO("MODE", 5, Xue, mode, 0),

    // @Param: MANUAL
    // @DisplayName: Manual model drive
    // @Description: Normalized drive for the software model in MANUAL mode
    // @Range: -1 1
    // @User: Standard
    AP_GROUPINFO("MANUAL", 6, Xue, manual, 0.0f),

    AP_GROUPEND
};

const AP_Param::GroupInfo ParametersG2::var_info[] = {
    // @Group: XUE_
    // @Path: Parameters.cpp
    AP_SUBGROUPINFO(xue, "XUE_", 1, ParametersG2, Xue),

#if AP_AIRSPEED_ENABLED
    // @Group: ARSPD
    // @Path: ../../libraries/AP_Airspeed/AP_Airspeed.cpp
    AP_SUBGROUPINFO(airspeed, "ARSPD", 10, ParametersG2, AP_Airspeed),
#endif

    AP_GROUPEND
};

ParametersG2::ParametersG2()
{
    AP_Param::setup_object_defaults(this, var_info);
}
