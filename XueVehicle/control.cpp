#include "Hello_Vehicle.h"

#include <AP_Math/AP_Math.h>

// Software-only plant: no actuator outputs are connected to this controller.
void Hello_Vehicle::update_control(float dt)
{
    update_mode();
    const float requested_speed = g2.xue.speed.get();
    const float gain = g2.xue.gain.get();
    control_dt = dt;
    control_valid = mode_inputs_valid(current_mode) &&
                    dt > 0 && dt <= 0.1f;
    target_speed = control_valid && current_mode == Mode::AUTO ? requested_speed : 0;
    control_error = target_speed - simulated_speed;
    control_output = 0;
    if (control_valid && g2.xue.enable.get() == 1) {
        switch (current_mode) {
        case Mode::STOP:
            break;
        case Mode::MANUAL:
            control_output = g2.xue.manual.get();
            break;
        case Mode::AUTO:
            control_output = constrain_float(gain * control_error, -1.0f, 1.0f);
            break;
        }
    }

    // Skip integration after a scheduling stall, rather than taking a large step.
    if (dt > 0 && dt <= 0.1f) {
        constexpr float acceleration_gain = 5.0f; // m/s^2 at full output
        constexpr float drag = 0.5f;             // 1/s
        simulated_speed += (control_output * acceleration_gain -
                            drag * simulated_speed) * dt;
    }
}
