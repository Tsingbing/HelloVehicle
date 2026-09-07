#include "GCS_Hello.h"
#include "Hello_Vehicle.h"

#if HAL_GCS_ENABLED

uint32_t GCS_Hello::custom_mode() const
{
    return uint32_t(hello_vehicle.get_mode());
}

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

void GCS_MAVLINK_Hello::update_hello()
{
    update_receive();
    queued_param_send();

    const uint32_t now = AP_HAL::millis();
    if (now - last_pressure_ms >= 100 && hello_vehicle.barometer.healthy(0) &&
        HAVE_PAYLOAD_SPACE(chan, SCALED_PRESSURE)) {
        send_scaled_pressure();
        last_pressure_ms = now;
    }
    if (now - last_attitude_ms >= 100 && HAVE_PAYLOAD_SPACE(chan, ATTITUDE)) {
        send_attitude();
        last_attitude_ms = now;
    }
    if (now - last_imu_ms >= 100 && HAVE_PAYLOAD_SPACE(chan, RAW_IMU)) {
        send_raw_imu();
        last_imu_ms = now;
    }
    if (now - last_heartbeat_time >= 1000) {
        send_heartbeat();
        last_heartbeat_time = now;
    }
}
//xueqingbing

void GCS_Hello::update()
{
    GCS_MAVLINK_Hello *link = chan(0);
    if (link != nullptr) {
        link->update_hello();
    }

    // Flush queued STATUSTEXT and other deferred MAVLink messages.  The
    // heartbeat is sent directly by update_hello(), but send_text() relies on
    // this standard GCS transmit update.
    update_send();
}

#endif // HAL_GCS_ENABLED
