#include "GCS_Hello.h"

#if HAL_GCS_ENABLED

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

void GCS_MAVLINK_Hello::update_hello()
{
    update_receive();
    queued_param_send();

    const uint32_t now = AP_HAL::millis();
    if (now - last_heartbeat_time >= 1000) {
        send_heartbeat();
        last_heartbeat_time = now;
    }
}

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
