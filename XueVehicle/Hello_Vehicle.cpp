#include "Hello_Vehicle.h"

#include <AP_HAL/AP_HAL_Main.h>

#define THISFIRMWARE "XueVehicle V0.1.0-dev"
#define FW_MAJOR 0
#define FW_MINOR 1
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV
// Custom application uses the generic firmware-version vehicle identifier.
#define APM_BUILD_XueVehicle APM_BUILD_UNKNOWN
#define FORCE_VERSION_H_INCLUDE
#include <AP_Common/AP_FWVersionDefine.h>
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

Hello_Vehicle hello_vehicle;

// Vehicle methods use the same shorthand as Rover; library methods retain
// SCHED_TASK_CLASS so the owning object is visible in the task table.
#define SCHED_TASK(func, rate_hz, max_time_us, priority) \
    SCHED_TASK_CLASS(Hello_Vehicle, &hello_vehicle, func, rate_hz, max_time_us, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(Hello_Vehicle, &hello_vehicle, func)

/*
 * 按优先级从小到大排列。普通任务列为：函数、频率 Hz、预计耗时 us、优先级。
 * FAST_TASK 每个主循环执行；普通任务在到期且预算允许时执行。
 * 本应用未继承 AP_Vehicle，因此需要自行列出通信、日志和校准任务。
 */
const AP_Scheduler::Task Hello_Vehicle::scheduler_tasks[] = {
    FAST_TASK(update_imu),
    SCHED_TASK(read_radio,             50,    200,  3),
    //         Function name,         Hz,     us, priority
    SCHED_TASK(update_compass,         10,   1000,  6),
    SCHED_TASK(update_baro,            10,   1000,  7),
#if COMPASS_CAL_ENABLED
    SCHED_TASK_CLASS(Compass,      &hello_vehicle.compass,   cal_update,     100,  200,  8),
#endif
    SCHED_TASK(control_task,           50,   1000,  9),
#if HAL_GCS_ENABLED
    SCHED_TASK_CLASS(GCS, static_cast<GCS*>(&hello_vehicle.gcs), update_receive, 100, 1000, 12),
    SCHED_TASK_CLASS(GCS, static_cast<GCS*>(&hello_vehicle.gcs), update_send,    100, 1000, 14),
#endif
#if HAL_LOGGING_ENABLED
    SCHED_TASK(logger_task,           100,   1000, 15),
    SCHED_TASK(sensor_log_task,        50,   1500, 18),
    SCHED_TASK(log_radio,              10,   1000, 20),
    SCHED_TASK(write_log,               1,    500, 21),
    SCHED_TASK_CLASS(AP_Scheduler, &hello_vehicle.scheduler, update_logging, 0.2, 1000, 24),
#endif
};

#undef FAST_TASK
#undef SCHED_TASK

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

    // @Group: RC
    // @Path: ../../../libraries/RC_Channel/RC_Channels_VarInfo.h
    GOBJECT(rc_channels, "RC", RC_Channels_Hello),

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
    init_radio();

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
