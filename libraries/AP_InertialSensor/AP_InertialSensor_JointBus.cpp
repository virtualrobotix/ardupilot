/*
  Author: Roberto Navoni, member of the ArduPilot Dev Team
  Contact: r.navoni74@gmail.com
  Developed by Roberto Navoni — DelphyAI LAB
  For information: r.navoni74@gmail.com
*/
#include "AP_InertialSensor_JointBus.h"

#if AP_INERTIALSENSOR_JOINTBUS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

AP_InertialSensor_JointBus *AP_InertialSensor_JointBus::_singleton;

AP_InertialSensor_JointBus::AP_InertialSensor_JointBus(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_JointBus::probe(AP_InertialSensor &imu)
{
    if (_singleton != nullptr) {
        return nullptr;
    }
    _singleton = NEW_NOTHROW AP_InertialSensor_JointBus(imu);
    return _singleton;
}

void AP_InertialSensor_JointBus::start()
{
    // the sensors are registered whether or not the link is up, so the vehicle boots with the
    // MCU silent
    const uint16_t rate = AP_INERTIALSENSOR_JOINTBUS_RATE_HZ;
    if (_imu.register_gyro(gyro_instance, rate,
                           AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, 0, 1, DEVTYPE_SERIAL)) &&
        _imu.register_accel(accel_instance, rate,
                            AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, 0, 2, DEVTYPE_SERIAL))) {
        started = true;
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_JointBus::timer, void));
    }
}

void AP_InertialSensor_JointBus::publish(const Vector3f &accel_in, const Vector3f &gyro_in, uint64_t now_us,
                                         Vector3f &accel, Vector3f &gyro)
{
    accel = accel_in;
    gyro = gyro_in;

    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel, now_us);

    _notify_new_gyro_sensor_rate_sample(gyro_instance, gyro);
    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro, now_us);
}

void AP_InertialSensor_JointBus::handle_sample(const Vector3f &accel, const Vector3f &gyro, uint32_t source_us)
{
    AP_InertialSensor_JointBus *self = _singleton;
    if (self == nullptr || !self->started) {
        return;
    }
    const uint64_t now = AP_HAL::micros64();
    WITH_SEMAPHORE(self->sem);
    self->last_sample_us = now;
    Sample sample;
    self->publish(accel, gyro, now, sample.accel, sample.gyro);
    sample.source_us = source_us;
    // every sample, in order, for a consumer that filters at the sensor rate
    if (self->fifo_count == ARRAY_SIZE(self->fifo)) {
        self->fifo_tail = (self->fifo_tail + 1) % ARRAY_SIZE(self->fifo);
        self->fifo_count--;
    }
    self->fifo[(self->fifo_tail + self->fifo_count) % ARRAY_SIZE(self->fifo)] = sample;
    self->fifo_count++;
}

uint8_t AP_InertialSensor_JointBus::pop_samples(Sample *out, uint8_t max)
{
    AP_InertialSensor_JointBus *self = _singleton;
    if (self == nullptr || out == nullptr) {
        return 0;
    }
    WITH_SEMAPHORE(self->sem);
    uint8_t n = 0;
    while (n < max && self->fifo_count > 0) {
        out[n++] = self->fifo[self->fifo_tail];
        self->fifo_tail = (self->fifo_tail + 1) % ARRAY_SIZE(self->fifo);
        self->fifo_count--;
    }
    return n;
}

/*
  The main loop waits for an IMU sample: with the link down (boot, MCU not flashed, relay
  stopped) a level, motionless stand-in keeps it running. link_alive() tells the two apart.
 */
void AP_InertialSensor_JointBus::timer()
{
    const uint64_t now = AP_HAL::micros64();
    const uint32_t period_us = 1000000UL / AP_INERTIALSENSOR_JOINTBUS_RATE_HZ;
    WITH_SEMAPHORE(sem);
    if (now - last_sample_us < 5U * period_us || now - last_standin_us < period_us) {
        return;
    }
    last_standin_us = now;
    Vector3f accel, gyro;
    publish(Vector3f(0.0f, 0.0f, -GRAVITY_MSS), Vector3f(), now, accel, gyro);
}

bool AP_InertialSensor_JointBus::link_alive()
{
    AP_InertialSensor_JointBus *self = _singleton;
    if (self == nullptr || !self->started) {
        return false;
    }
    WITH_SEMAPHORE(self->sem);
    return self->last_sample_us != 0 &&
           AP_HAL::micros64() - self->last_sample_us < 5U * (1000000UL / AP_INERTIALSENSOR_JOINTBUS_RATE_HZ);
}

bool AP_InertialSensor_JointBus::update()
{
    if (started) {
        update_accel(accel_instance);
        update_gyro(gyro_instance);
    }
    return started;
}

bool AP_InertialSensor_JointBus::get_output_banner(char *banner, uint8_t banner_len)
{
    snprintf(banner, banner_len, "IMU%u: JointBus %uHz", gyro_instance, unsigned(AP_INERTIALSENSOR_JOINTBUS_RATE_HZ));
    return true;
}

#endif  // AP_INERTIALSENSOR_JOINTBUS_ENABLED
