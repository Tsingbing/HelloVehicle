#include "Hello_Vehicle.h"

void Hello_Vehicle::init_baro()
{
    barometer.init();
    // Includes sensor settling and ground-pressure calibration. Keep the board
    // at a fixed height until startup completes; never run this in a task.
    barometer.calibrate();
}

void Hello_Vehicle::update_baro()
{
    barometer.update();
#if HAL_LOGGING_ENABLED
    if (!logger.logging_started()) {
        return;
    }
    for (uint8_t i = 0; i < barometer.num_instances(); i++) {
        const bool healthy = barometer.healthy(i);
        // Invalid numeric fields are zero; always inspect Healthy as well.
        logger.Write("XBAR", "TimeUS,I,Press,Temp,Alt,Healthy", "QBfffB",
                     AP_HAL::micros64(), i,
                     double(healthy ? barometer.get_pressure(i) : 0),
                     double(healthy ? barometer.get_temperature(i) : 0),
                     double(healthy ? barometer.get_altitude(i) : 0),
                     uint8_t(healthy));
    }
#endif
}

void Hello_Vehicle::init_imu()
{
    // BoardConfig and parameter loading must precede driver discovery.
    // Keep the board still during the default startup gyro calibration.
    ins.init(scheduler.get_loop_rate_hz());
    // This learning vehicle uses DCM only. EKF navigation requires additional
    // vehicle/DAL infrastructure, so do not start an EKF in this application.
#if HAL_NAVEKF2_AVAILABLE
    ahrs.EKF2.set_enable(false);
#endif
#if HAL_NAVEKF3_AVAILABLE
    ahrs.EKF3.set_enable(false);
#endif
    ahrs.set_ekf_type(AP_AHRS::EKFType::DCM);
    ahrs.init();
    compass.init();
    compass.read();
}

void Hello_Vehicle::update_imu()
{
    // AP_Scheduler::loop() already waited for this sample.
    ins.update();
    // INS was already updated above; do not consume a second sample.
    ahrs.update(true);
}

void Hello_Vehicle::update_compass()
{
    compass.read();
#if HAL_LOGGING_ENABLED
    if (!logger.logging_started()) {
        return;
    }
    // Log even when no device was detected: absence must be observable.
    const uint8_t count = compass.get_count();
    bool calibrating = false;
#if COMPASS_CAL_ENABLED
    calibrating = compass.is_calibrating();
#endif
    logger.Write("XCST", "TimeUS,Count,Healthy,UseYaw,Cal", "QBBBB",
                 AP_HAL::micros64(), count, uint8_t(compass.healthy()),
                 uint8_t(compass.use_for_yaw()), uint8_t(calibrating));
    for (uint8_t i = 0; i < count; i++) {
        const Vector3f &field = compass.get_field(i);
        // Keep the last field even if unhealthy; AgeMS identifies stale data.
        logger.Write("XCMP", "TimeUS,I,MX,MY,MZ,Norm,AgeMS,Healthy,UseYaw,Config",
                     "QBffffIBBB", AP_HAL::micros64(), i,
                     double(field.x), double(field.y), double(field.z),
                     double(field.length()),
                     uint32_t(AP_HAL::millis() - compass.last_update_ms(i)),
                     uint8_t(compass.healthy(i)), uint8_t(compass.use_for_yaw(i)),
                     uint8_t(compass.configured(i)));
    }
#endif
}

#if HAL_LOGGING_ENABLED
void Hello_Vehicle::write_imu_log()
{
    logger.Write("XATT", "TimeUS,Roll,Pitch,Yaw,Healthy", "QfffB",
                 AP_HAL::micros64(), double(degrees(ahrs.get_roll())),
                 double(degrees(ahrs.get_pitch())), double(degrees(ahrs.get_yaw())),
                 uint8_t(ahrs.healthy()));
    const uint8_t accel_count = ins.get_accel_count();
    const uint8_t gyro_count = ins.get_gyro_count();
    const uint8_t count = MAX(accel_count, gyro_count);
    for (uint8_t i = 0; i < count; i++) {
        // A missing or unhealthy sensor is explicitly marked; its numeric
        // fields must not be interpreted as valid measurements.
        const bool accel_ok = ins.get_accel_health(i);
        const bool gyro_ok = ins.get_gyro_health(i);
        const Vector3f accel = accel_ok ? ins.get_accel(i) : Vector3f{};
        const Vector3f gyro = gyro_ok ? ins.get_gyro(i) : Vector3f{};
        logger.Write("XIMU", "TimeUS,I,AX,AY,AZ,GX,GY,GZ,AH,GH", "QBffffffBB",
                     AP_HAL::micros64(), i,
                     double(accel.x), double(accel.y), double(accel.z),
                     double(gyro.x), double(gyro.y), double(gyro.z),
                     uint8_t(accel_ok), uint8_t(gyro_ok));
    }
}
#endif
