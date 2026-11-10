/*
  Author: Roberto Navoni, member of the ArduPilot Dev Team
  Contact: r.navoni74@gmail.com
  Developed by Roberto Navoni — DelphyAI LAB
  For information: r.navoni74@gmail.com
*/
#include "AP_JointBus.h"

#if AP_JOINTBUS_ENABLED

#include "AP_JointBus_Backend.h"
#include "AP_JointBus_XDuck.h"

#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <string.h>

AP_JointBus *AP_JointBus::_singleton;

AP_JointBus::AP_JointBus()
{
    _singleton = this;
}

bool AP_JointBus::init(uint8_t n_joints)
{
    if (_backend != nullptr) {
        return true;
    }
#if AP_JOINTBUS_XDUCK_ENABLED
    AP_HAL::UARTDriver *uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_JointBus, 0);
    if (uart != nullptr) {
        const uint32_t baud = AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_JointBus, 0);
        AP_JointBus_Backend *backend = NEW_NOTHROW AP_JointBus_XDuck(uart, baud, n_joints);
        if (backend != nullptr && backend->init()) {
            _backend = backend;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "JointBus: XDuck link, %u joints", unsigned(n_joints));
            return true;
        }
        delete backend;
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "JointBus: XDuck link failed to start");
    }
#endif
    return false;
}

bool AP_JointBus::get_feedback(Feedback &fb)
{
    return _backend != nullptr && _backend->get_feedback(fb);
}

bool AP_JointBus::torque_enabled()
{
    return _backend != nullptr && _backend->torque_enabled();
}

void AP_JointBus::set_velocity_alpha(float alpha)
{
    if (_backend != nullptr) {
        _backend->set_velocity_alpha(alpha);
    }
}

void AP_JointBus::write(const float *q_rad, uint8_t n, bool enable)
{
    if (_backend != nullptr) {
        _backend->write(q_rad, n, enable);
    }
}

void AP_JointBus::set_aux_deg(float deg)
{
    if (_backend != nullptr) {
        _backend->set_aux_deg(deg);
    }
}

void AP_JointBus::request(const Request &req)
{
    if (_backend != nullptr) {
        _backend->request(req);
    }
}

bool AP_JointBus::pop_reply(Reply &reply)
{
    return _backend != nullptr && _backend->pop_reply(reply);
}

bool AP_JointBus::exclusive()
{
    return _backend != nullptr && _backend->exclusive();
}

void AP_JointBus::end_exclusive()
{
    if (_backend != nullptr) {
        _backend->end_exclusive();
    }
}

void AP_JointBus::get_telemetry(Telemetry &t)
{
    memset(&t, 0, sizeof(t));
    if (_backend != nullptr) {
        _backend->get_telemetry(t);
    }
}

namespace AP {
    AP_JointBus *jointbus()
    {
        return AP_JointBus::get_singleton();
    }
};

#endif  // AP_JOINTBUS_ENABLED
