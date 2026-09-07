#include "Hello_Vehicle.h"
#include <AP_Math/AP_Math.h>

bool Hello_Vehicle::mode_inputs_valid(Mode mode) const
{
    if (mode == Mode::STOP) {
        return true;
    }
    if (g2.xue.enable.get() != 0 && g2.xue.enable.get() != 1) {
        return false;
    }
    if (mode == Mode::MANUAL) {
        const float manual = g2.xue.manual.get();
        return std::isfinite(manual) && manual >= -1 && manual <= 1;
    }
    const float speed = g2.xue.speed.get();
    const float gain = g2.xue.gain.get();
    return std::isfinite(speed) && speed >= 0 && speed <= 100 &&
           std::isfinite(gain) && gain >= 0 && gain <= 10;
}

bool Hello_Vehicle::set_mode(int8_t requested)
{
    const Mode previous = current_mode;
    const bool accepted = requested >= int8_t(Mode::STOP) &&
                          requested <= int8_t(Mode::AUTO) &&
                          mode_inputs_valid(Mode(requested));
    if (accepted) {
        current_mode = Mode(requested);
        // Reset controller state, but preserve the physical model's velocity.
        control_output = 0;
        control_error = 0;
        target_speed = 0;
    }
#if HAL_LOGGING_ENABLED
    if (logger.logging_started()) {
        // Reason 0: parameter request. Accepted distinguishes rejected requests.
        logger.Write("XMOD", "TimeUS,Old,Requested,Mode,Reason,Accepted", "QBbBBB",
                     AP_HAL::micros64(), uint8_t(previous), int(requested),
                     uint8_t(current_mode), uint8_t(0), uint8_t(accepted));
        logger.Write_Message(accepted ? "Xue mode request accepted" :
                                       "Xue mode request rejected");
    }
#endif
    return accepted;
}

void Hello_Vehicle::update_mode()
{
    const int8_t requested = g2.xue.mode.get();
    if (requested != last_mode_request) {
        last_mode_request = requested;
        set_mode(requested);
    }
}
