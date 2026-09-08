#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>

#include "GCS_Hello.h"
#include "Parameters.h"

class Hello_Vehicle : public AP_HAL::HAL::Callbacks {
public:
    Hello_Vehicle();

    void setup() override;
    void loop() override;
    enum class Mode : uint8_t { STOP = 0, MANUAL = 1, AUTO = 2 };
    Mode get_mode() const { return current_mode; }

    Parameters g; //xueqingbing
    ParametersG2 g2;
#if HAL_LOGGING_ENABLED
    AP_Logger logger;
#endif
    AP_BoardConfig BoardConfig;
    AP_InertialSensor ins;
    AP_GPS gps;
    AP_Baro barometer;
    Compass compass;
    AP_AHRS ahrs;
    AP_Scheduler scheduler;
#if HAL_EXTERNAL_AHRS_ENABLED
    AP_ExternalAHRS external_ahrs;
#endif

private:
    void load_parameters();
    void init_imu();
    void init_baro();
    void update_baro();
    void update_imu();
    void update_compass();
    void control_task();
    static const AP_Scheduler::Task scheduler_tasks[];
    void update_control(float dt);
    void update_mode();
    bool set_mode(int8_t requested);
    bool mode_inputs_valid(Mode mode) const;
    Mode current_mode = Mode::STOP;
    int16_t last_mode_request = -129;
    uint64_t last_control_us = 0;
    uint32_t control_interval_us = 0;
    uint32_t control_elapsed_us = 0;
    float target_speed = 0;
    float simulated_speed = 0;
    float control_error = 0;
    float control_output = 0;
    float control_dt = 0;
    bool control_valid = false;
#if HAL_LOGGING_ENABLED
    void write_log();
    void write_control_log();
    void write_imu_log();
    void logger_task();
    void sensor_log_task();
    bool log_start_reported;
#endif

    AP_Param param_loader;
    AP_SerialManager serial_manager;
#if HAL_GCS_ENABLED
    GCS_Hello gcs;
#endif
};

extern Hello_Vehicle hello_vehicle;
