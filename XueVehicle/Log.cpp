#include "Hello_Vehicle.h"

#if HAL_LOGGING_ENABLED
void Hello_Vehicle::write_control_log()
{
    logger.Write("TIME", "TimeUS,Interval,Elapsed", "QII",
                 AP_HAL::micros64(), control_interval_us, control_elapsed_us);
    // Actual and Error describe the same pre-integration sample used by control.
    logger.Write("CTRL", "TimeUS,Target,Actual,Error,Output,Dt,Valid,Mode",
                 "QfffffBB", AP_HAL::micros64(),
                 double(target_speed), double(target_speed - control_error),
                 double(control_error), double(control_output),
                 double(control_dt), uint8_t(control_valid), uint8_t(current_mode));
}
#endif
