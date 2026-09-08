#include "Hello_Vehicle.h"

#define RC_CHANNELS_SUBCLASS RC_Channels_Hello
#define RC_CHANNEL_SUBCLASS RC_Channel_Hello
#include <RC_Channel/RC_Channels_VarInfo.h>

void Hello_Vehicle::init_radio()
{
    rc_channels.init();
    for (uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
        rc_channels.channel(i)->set_angle(1000);
        rc_channels.channel(i)->set_default_dead_zone(30);
    }
}

void Hello_Vehicle::read_radio()
{
    if (rc_channels.read_input()) {
        rc_seen = true;
        last_rc_ms = AP_HAL::millis();
    }
}

#if HAL_LOGGING_ENABLED
void Hello_Vehicle::log_radio()
{
    if (!logger.logging_started()) {
        return;
    }
    const uint32_t age_ms = rc_seen ? AP_HAL::millis() - last_rc_ms : UINT32_MAX;
    const uint8_t count = rc_channels.get_valid_channel_count();
    // Fresh is a sample-age observation, not a receiver failsafe verdict.
    logger.Write("XRCS", "TimeUS,Count,AgeMS,Fresh", "QBIB",
                 AP_HAL::micros64(), count, age_ms, uint8_t(rc_seen && age_ms <= 500));
    for (uint8_t i = 0; i < count; i++) {
        const RC_Channel *channel = rc_channels.channel(i);
        logger.Write("XRC", "TimeUS,Ch,PWM,Control", "QBHh",
                     AP_HAL::micros64(), uint8_t(i + 1),
                     uint16_t(channel->get_radio_in()), int16_t(channel->get_control_in()));
    }
}
#endif
