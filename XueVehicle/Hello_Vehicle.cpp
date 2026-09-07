#include "Hello_Vehicle.h"

#include <AP_HAL/AP_HAL_Main.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

Hello_Vehicle hello_vehicle;

// Lower priorities run first. INS/AHRS runs every loop before other tasks.
const AP_Scheduler::Task Hello_Vehicle::scheduler_tasks[] = {
    FAST_TASK_CLASS(Hello_Vehicle, &hello_vehicle, update_imu),
    SCHED_TASK_CLASS(Hello_Vehicle, &hello_vehicle, update_compass, 10, 1000, 6),
    SCHED_TASK_CLASS(Hello_Vehicle, &hello_vehicle, update_baro, 10, 1000, 7),
    SCHED_TASK_CLASS(Hello_Vehicle, &hello_vehicle, control_task, 50, 1000, 9),
#if HAL_GCS_ENABLED
    SCHED_TASK_CLASS(Hello_Vehicle, &hello_vehicle, gcs_task, 100, 1500, 12),
#endif
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(Hello_Vehicle, &hello_vehicle, logger_task, 100, 1000, 15),
    SCHED_TASK_CLASS(Hello_Vehicle, &hello_vehicle, sensor_log_task, 50, 1500, 18),
    SCHED_TASK_CLASS(Hello_Vehicle, &hello_vehicle, write_log, 1, 500, 21),
    SCHED_TASK_CLASS(AP_Scheduler, &hello_vehicle.scheduler, update_logging, 0.2, 1000, 24),
#endif
};

#if HAL_LOGGING_ENABLED
static const LogStructure log_structure[] = {
    LOG_COMMON_STRUCTURES,
};
#endif

#define AP_PARAM_VEHICLE_NAME hello_vehicle

const AP_Param::Info var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: EEPROM format version
    // @Description: EEPROM format version for Hello_Vehicle
    // @User: Advanced
    GSCALAR(format_version, "FORMAT_VERSION", 1),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: Enables Hello_Vehicle logging when nonzero
    // @User: Advanced
    GSCALAR(log_bitmask, "LOG_BITMASK", 1),

    GOBJECT(g2, "", ParametersG2),

#if HAL_LOGGING_ENABLED
    // @Group: LOG
    // @Path: ../../libraries/AP_Logger/AP_Logger.cpp
    GOBJECT(logger, "LOG", AP_Logger),
#endif

    // @Group: BRD_
    // @Path: ../../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig, "BRD_", AP_BoardConfig),

    // @Group: INS
    // @Path: ../../../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins, "INS", AP_InertialSensor),
    // @Group: AHRS_
    // @Path: ../../../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs, "AHRS_", AP_AHRS),
    // @Group: COMPASS_
    // @Path: ../../../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass, "COMPASS_", Compass),
    // @Group: SCHED_
    // @Path: ../../../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),
    // @Group: BARO
    // @Path: ../../../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "BARO", AP_Baro),

    AP_VAREND
};

#undef AP_PARAM_VEHICLE_NAME

#if HAL_LOGGING_ENABLED
Hello_Vehicle::Hello_Vehicle() :
    logger(g.log_bitmask),
    log_start_reported(false),
    param_loader(var_info)
#else
Hello_Vehicle::Hello_Vehicle() :
    param_loader(var_info)
#endif
{
}

void Hello_Vehicle::load_parameters()
{
    AP_Param::setup_sketch_defaults();
    if (!AP_Param::setup()) {
        AP_HAL::panic("Hello AP_Param setup failed");
    }
    AP_Param::check_var_info();
    AP_Param::set_default_by_name("SCHED_LOOP_RATE", 100);
    AP_Param::load_all();
}

void Hello_Vehicle::setup()
{
    load_parameters();
    BoardConfig.init();

#if HAL_LOGGING_ENABLED
    // Hello_Vehicle is a dedicated filesystem logging test.  An EEPROM left
    // over from an earlier build may contain LOG_BACKEND_TYPE=0, in which
    // case AP_Logger::Init() creates no backend and all writes are discarded.
    enum ap_var_type backend_type;
    AP_Param *backend_param = AP_Param::find("LOG_BACKEND_TYPE", &backend_type);
    if (backend_param != nullptr && backend_type == AP_PARAM_INT8) {
        static_cast<AP_Int8 *>(backend_param)->set_and_save_ifchanged(1);
    }

    // This must be set before the first write.  The file backend then opens a
    // new log asynchronously from logger.periodic_tasks(), so GCS startup is
    // not blocked waiting for the SD card.
    logger.set_force_log_disarmed(true);
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
#endif

    serial_manager.init();
#if HAL_GCS_ENABLED
    gcs.init();
    gcs.setup_console();
#endif
    scheduler.init(scheduler_tasks, ARRAY_SIZE(scheduler_tasks), uint32_t(-1));
    init_imu();
    init_baro();
    last_control_us = AP_HAL::micros64();
}

#if HAL_LOGGING_ENABLED
void Hello_Vehicle::write_log()
{
    if (!logger.logging_started()) {
        return;
    }
    logger.Write(
        "XUE",
        "TimeUS,Gain,Speed,Enable,Test",
        "s---#",
        "F0000",
        "QffBi",
        AP_HAL::micros64(),
        float(g2.xue.gain),
        float(g2.xue.speed),
        uint8_t(g2.xue.enable),
        int32_t(g2.xue.test));
}
#endif

void Hello_Vehicle::loop()
{
    scheduler.loop();
}

void Hello_Vehicle::control_task()
{
    const uint64_t control_start_us = AP_HAL::micros64();
    const uint64_t interval_us = control_start_us - last_control_us;
    last_control_us = control_start_us;
    control_interval_us = uint32_t(MIN(interval_us, uint64_t(UINT32_MAX)));
    update_control(float(interval_us) * 1.0e-6f);
    control_elapsed_us = uint32_t(AP_HAL::micros64() - control_start_us);
#if HAL_LOGGING_ENABLED
    if (logger.logging_started()) {
        write_control_log();
    }
#endif
}

void Hello_Vehicle::gcs_task()
{
#if HAL_GCS_ENABLED
    gcs.update();
#endif
}

#if HAL_LOGGING_ENABLED
void Hello_Vehicle::logger_task()
{
    logger.periodic_tasks();

    if (logger.logging_started() && !log_start_reported) {
        log_start_reported = true;
        logger.Write_Message("Hello_Vehicle started");
    }

}

void Hello_Vehicle::sensor_log_task()
{
    if (logger.logging_started()) {
        write_imu_log();
    }
}
#endif

AP_HAL_MAIN_CALLBACKS(&hello_vehicle);
