#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "GCS_Hello.h"
#include "Parameters.h"

class Hello_Vehicle : public AP_HAL::HAL::Callbacks {
public:
    Hello_Vehicle();

    void setup() override;
    void loop() override;

    Parameters g;
    ParametersG2 g2;
#if HAL_LOGGING_ENABLED
    AP_Logger logger;
#endif
    AP_BoardConfig BoardConfig;

private:
    void load_parameters();
#if HAL_LOGGING_ENABLED
    void write_log();
    bool log_start_reported;
#endif

    AP_Param param_loader;
    AP_SerialManager serial_manager;
#if HAL_GCS_ENABLED
    GCS_Hello gcs;
#endif
};

extern Hello_Vehicle hello_vehicle;
