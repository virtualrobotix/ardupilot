/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_RCMAPPER_H
#define AP_RCMAPPER_H

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>

class RCMapper
{
public:
    /// Constructor
    ///
    RCMapper();

    /// roll - return input channel number for roll / aileron input
    uint8_t roll() const { return _ch_roll; }

    /// pitch - return input channel number for pitch / elevator input
    uint8_t pitch() const { return _ch_pitch; }

    /// throttle - return input channel number for throttle input
    uint8_t throttle() const { return _ch_throttle; }

    /// yaw - return input channel number for yaw / rudder input
    uint8_t yaw() const { return _ch_yaw; }

    /// fltmode - return input channel number for flight mode change
    uint8_t fltmode() const { return _ch_fltmode; }

    /// tune - return input channel number for tune input
    uint8_t tune() const { return _ch_tune; }

    /// CH7 - return input channel number for CH7 input
    uint8_t aux1() const { return _ch_aux1; }

    /// CH8 - return input channel number for CH8 input
    uint8_t aux2() const { return _ch_aux2; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    // channel mappings
    AP_Int8 _ch_roll;
    AP_Int8 _ch_pitch;
    AP_Int8 _ch_yaw;
    AP_Int8 _ch_throttle;
    AP_Int8 _ch_fltmode;
    AP_Int8 _ch_tune;
    AP_Int8 _ch_aux1;
    AP_Int8 _ch_aux2;
};
#endif
