#pragma once

#include <RC_Channel/RC_Channel.h>

class RC_Channel_Hello : public RC_Channel {};

class RC_Channels_Hello : public RC_Channels {
public:
    RC_Channel_Hello obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Hello *channel(uint8_t index) override {
        return index < NUM_RC_CHANNELS ? &obj_channels[index] : nullptr;
    }

protected:
    // No RC flight-mode or auxiliary actions in this observation-only stage.
    int8_t flight_mode_channel_number() const override { return 0; }
};
