#include "GCS_Hello.h"
#include "Hello_Vehicle.h"

#if HAL_GCS_ENABLED

uint32_t GCS_Hello::custom_mode() const
{
    return uint32_t(hello_vehicle.get_mode());
}

// Match Rover's stream layout; include only this application's telemetry.
const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK_Parameters, streamRates[0], 10),
    AP_GROUPINFO("RC_CHAN", 2, GCS_MAVLINK_Parameters, streamRates[2], 10),
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK_Parameters, streamRates[5], 10),
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK_Parameters, streamRates[7], 5),
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK_Parameters, streamRates[8], 10),
    AP_GROUPEND
};

static const ap_message STREAM_RAW_SENSORS_msgs[] = {
    MSG_RAW_IMU,
    MSG_SCALED_PRESSURE,
};
static const ap_message STREAM_RC_CHANNELS_msgs[] = {
    MSG_RC_CHANNELS,
    MSG_RC_CHANNELS_RAW,
};
static const ap_message STREAM_EXTRA1_msgs[] = {
    MSG_ATTITUDE,
};
#if COMPASS_CAL_ENABLED
static const ap_message STREAM_EXTRA3_msgs[] = {
    MSG_MAG_CAL_REPORT,
    MSG_MAG_CAL_PROGRESS,
};
#endif
static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM,
};

const GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_RAW_SENSORS),
    MAV_STREAM_ENTRY(STREAM_RC_CHANNELS),
    MAV_STREAM_ENTRY(STREAM_EXTRA1),
#if COMPASS_CAL_ENABLED
    MAV_STREAM_ENTRY(STREAM_EXTRA3),
#endif
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_TERMINATOR
};

#endif // HAL_GCS_ENABLED
