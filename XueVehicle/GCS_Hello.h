#pragma once

#include <GCS_MAVLink/GCS.h>
#include <AP_Compass/AP_Compass_config.h>

#if HAL_GCS_ENABLED

class GCS_MAVLINK_Hello : public GCS_MAVLINK {
public:
    using GCS_MAVLINK::GCS_MAVLINK;

private:
    uint8_t sysid_my_gcs() const override { return 255; }
    uint32_t telem_delay() const override { return 0; }
    bool handle_guided_request(AP_Mission::Mission_Command &) override { return false; }

protected:
    MAV_MODE base_mode() const override { return MAV_MODE(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED); }
    MAV_STATE vehicle_system_status() const override { return MAV_STATE_STANDBY; }
    bool set_home_to_current_location(bool) override { return false; }
    bool set_home(const Location &, bool) override { return false; }
    void send_nav_controller_output() const override {}
    void send_pid_tuning() override {}
};

class GCS_Hello : public GCS {
public:
    uint8_t sysid_this_mav() const override { return 1; }
    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override { return MAV_TYPE_GROUND_ROVER; }

protected:
    GCS_MAVLINK_Hello *new_gcs_mavlink_backend(
        GCS_MAVLINK_Parameters &parameters,
        AP_HAL::UARTDriver &uart) override
    {
        return new GCS_MAVLINK_Hello(parameters, uart);
    }

private:
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Hello);
};

#endif // HAL_GCS_ENABLED
