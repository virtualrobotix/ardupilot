/*
  IMU fed by the AP_JointBus link: the MCU that drives the servo bus samples the IMU in the same
  tick and ships both in one state packet. The link thread hands every new sample to
  handle_sample(); this backend owns no device. While the link is silent a level, motionless
  stand-in sample is published at the same rate, so the vehicle boots and keeps running.

  Author: Roberto Navoni, member of the ArduPilot Dev Team
  Contact: r.navoni74@gmail.com
  Developed by Roberto Navoni — DelphyAI LAB
  For information: r.navoni74@gmail.com
*/
#pragma once

#include "AP_InertialSensor_config.h"

#if AP_INERTIALSENSOR_JOINTBUS_ENABLED

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#ifndef AP_INERTIALSENSOR_JOINTBUS_RATE_HZ
#define AP_INERTIALSENSOR_JOINTBUS_RATE_HZ 100
#endif

class AP_InertialSensor_JointBus : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_JointBus(AP_InertialSensor &imu);

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu);

    bool update() override;
    void start() override;
    bool get_output_banner(char *banner, uint8_t banner_len) override;

    // one sample in the autopilot body frame (forward, right, down): specific force in m/s/s,
    // angular rate in rad/s, source_us the sampling time on the sender's clock. Called from the
    // link thread; ignored until start() has run.
    static void handle_sample(const Vector3f &accel, const Vector3f &gyro, uint32_t source_us);

    // true while samples come from the link; false while the stand-in is feeding the main loop
    static bool link_alive();

    // the link samples after rotation and calibration, oldest first, each one once. For a
    // consumer that filters at the sensor rate whatever the main loop does. Stand-in samples
    // are not included.
    struct Sample {
        Vector3f accel;
        Vector3f gyro;
        uint32_t source_us;
    };
    static uint8_t pop_samples(Sample *out, uint8_t max);

private:
    void publish(const Vector3f &accel_in, const Vector3f &gyro_in, uint64_t now_us, Vector3f &accel, Vector3f &gyro);
    void timer();

    static AP_InertialSensor_JointBus *_singleton;
    HAL_Semaphore sem;
    Sample fifo[16];
    uint8_t fifo_tail;
    uint8_t fifo_count;
    uint64_t last_sample_us;
    uint64_t last_standin_us;
    bool started;
};

#endif  // AP_INERTIALSENSOR_JOINTBUS_ENABLED
