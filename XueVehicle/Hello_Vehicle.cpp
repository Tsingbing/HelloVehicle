#include "Hello_Vehicle.h"

#include <AP_HAL/AP_HAL_Main.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

Hello_Vehicle hello_vehicle;

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
}

#if HAL_LOGGING_ENABLED
void Hello_Vehicle::write_log()
{
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
#if HAL_GCS_ENABLED
    gcs.update();
#endif
#if HAL_LOGGING_ENABLED
    logger.periodic_tasks();
    static uint32_t last_log_ms;
    const uint32_t now = AP_HAL::millis();

    if (logger.logging_started() && !log_start_reported) {
        log_start_reported = true;
        logger.Write_Message("Hello_Vehicle started");
    }

    if (logger.logging_started() && now - last_log_ms >= 1000) {
        last_log_ms = now;
        write_log();
    }
#endif
    hal.scheduler->delay(10);
}

AP_HAL_MAIN_CALLBACKS(&hello_vehicle);
