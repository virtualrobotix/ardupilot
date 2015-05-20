// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __AP_Frsky_VRBRAIN_H__
#define __AP_Frsky_VRBRAIN_H__

#include <AP_HAL.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Baro.h>
#include <AP_BattMonitor.h>
#include <AP_SerialManager.h>

//#define CONFIG_HAL_BOARD HAL_BOARD_VRBRAIN
//#define APM_BUILD_TYPE APM_BUILD_ArduCopter

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN && APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#define FRSKY_POLL_REQUEST      0x7E
#define FRSKY_SENSOR_ID         0xA1        // Sensor ID, must be something that is polled by FrSky RX
#define FRSKY_DATA_FRAME        0x10


/* FrSky sensor hub data IDs */
#define FRSKY_ID_GPS_ALT_BP     0x01
#define FRSKY_ID_TEMP1          0x02
#define FRSKY_ID_RPM            0x03
#define FRSKY_ID_FUEL           0x04
#define FRSKY_ID_TEMP2          0x05
#define FRSKY_ID_VOLTS          0x06
#define FRSKY_ID_GPS_ALT_AP     0x09
#define FRSKY_ID_BARO_ALT_BP    0x10
#define FRSKY_ID_GPS_SPEED_BP   0x11
#define FRSKY_ID_GPS_LONG_BP    0x12
#define FRSKY_ID_GPS_LAT_BP     0x13
#define FRSKY_ID_GPS_COURS_BP   0x14
#define FRSKY_ID_GPS_DAY_MONTH  0x15
#define FRSKY_ID_GPS_YEAR       0x16
#define FRSKY_ID_GPS_HOUR_MIN   0x17
#define FRSKY_ID_GPS_SEC        0x18
#define FRSKY_ID_GPS_SPEED_AP   0x19
#define FRSKY_ID_GPS_LONG_AP    0x1A
#define FRSKY_ID_GPS_LAT_AP     0x1B
#define FRSKY_ID_GPS_COURS_AP   0x1C
#define FRSKY_ID_BARO_ALT_AP    0x21
#define FRSKY_ID_GPS_LONG_EW    0x22
#define FRSKY_ID_GPS_LAT_NS     0x23
#define FRSKY_ID_ACCEL_X        0x24
#define FRSKY_ID_ACCEL_Y        0x25
#define FRSKY_ID_ACCEL_Z        0x26
#define FRSKY_ID_CURRENT        0x28
#define FRSKY_ID_VARIO          0x30
#define FRSKY_ID_VFAS           0x39
#define FRSKY_ID_VOLTS_BP       0x3A
#define FRSKY_ID_VOLTS_AP       0x3B

// FrSky S-Port DATA IDs (2 bytes)
#define FRSKY_ID2_ALTITUDE      0x0100
#define FRSKY_ID2_VARIO         0x0110
#define FRSKY_ID2_CURRENT       0x0200
#define FRSKY_ID2_VOLTAGE       0x0210
#define FRSKY_ID2_CELLS         0x0300
#define FRSKY_ID2_T1            0x0400
#define FRSKY_ID2_T2            0x0410
#define FRSKY_ID2_RPM           0x0500
#define FRSKY_ID2_FUEL          0x0600
#define FRSKY_ID2_ACCX          0x0700
#define FRSKY_ID2_ACCY          0x0710
#define FRSKY_ID2_ACCZ          0x0720
#define FRSKY_ID2_GPS_LON_LAT   0x0800
#define FRSKY_ID2_GPS_ALT       0x0820
#define FRSKY_ID2_SPEED         0x0830
#define FRSKY_ID2_HEADING       0x0840
#define FRSKY_ID2_A3            0x0900
#define FRSKY_ID2_A4            0x0910
#define FRSKY_ID2_BARO_ALT      0x8010
#define FRSKY_ID2_BETA_VARIO    0x8030
#define FRSKY_ID2_GPS_TIME_DATE 0x0850
#define FRSKY_ID2_RSSI          0xF101
#define FRSKY_ID2_ADC1          0xF102      // used by X4R receiver
#define FRSKY_ID2_ADC2          0xF103      // used by X4R receiver
#define FRSKY_ID2_BATT          0xF104
#define FRSKY_ID2_SWR           0xF105


class AP_Frsky_VRBRAIN
{
public:
    //constructor
    AP_Frsky_VRBRAIN(AP_AHRS &ahrs, AP_BattMonitor &battery);

    // supported protocols
    enum FrSkyProtocol {
        FrSkyUnknown = 0,
        FrSkyDPORT = 2,
        FrSkySPORT = 3
    };

    // init - perform require initialisation including detecting which protocol to use
    void init(const AP_SerialManager& serial_manager);

    // send_frames - sends updates down telemetry link for both DPORT and SPORT protocols
    //  should be called by main program at 50hz to allow poll for serial bytes
    //  coming from the receiver for the SPort protocol
    void send_frames(uint8_t control_mode, uint8_t simple_mode, int32_t altitude, int8_t alt_mode, float climbrate, int16_t throttle);


private:

    // init_uart_for_sport - initialise uart for use by sport
    void init_uart_for_sport();

    // send_hub_frame - main transmission function when protocol is FrSkyDPORT
    void send_hub_frame();

    // sport_tick - main call to send updates to transmitter when protocol is FrSkySPORT
    //  called by scheduler at a high rate
    void sport_tick();

    // methods related to the nuts-and-bolts of sending data
    void calc_crc(uint8_t byte);
    void send_crc();
    void frsky_send_byte(uint8_t value);
    void frsky_send_hub_startstop();
    void frsky_send_sport_prim();
    void frsky_send_data(uint8_t id, int16_t data);

    // methods to convert flight controller data to frsky telemetry format
    void calc_baro_alt();
    float frsky_format_gps(float dec);
    void calc_gps_position();
    void calc_battery();
    void calc_gps_sats();

    // methods to send individual pieces of data down telemetry link
    void send_gps_sats(void);
    void send_mode(void);
    void send_baro_alt_m(void);
    void send_baro_alt_cm(void);
    void send_batt_remain(void);
    void send_batt_volts(void);
    void send_current(void);
    void send_prearm_error(void);
    void send_heading(void);
    void send_gps_lat_dd(void);
    void send_gps_lat_mm(void);
    void send_gps_lat_ns(void);
    void send_gps_lon_dd(void);
    void send_gps_lon_mm(void);
    void send_gps_lon_ew(void);
    void send_gps_speed_meter(void);
    void send_gps_speed_cm(void);
    void send_gps_alt_meter(void);
    void send_gps_alt_cm(void);

    void frsky_send_frame1(uint8_t mode);
    void frsky_send_frame2(void);
    void frsky_send_char(uint8_t value);
    void frsky_send_value(uint16_t valueid, uint32_t value);
    void check_sport_input(uint8_t control_mode, uint8_t simple_mode, int32_t altitude, int8_t alt_mode, float climbrate, int16_t throttle);


    AP_AHRS &_ahrs;                         // reference to attitude estimate
    AP_BattMonitor &_battery;               // reference to battery monitor object
    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver
    bool _initialised_uart;                 // true when we have detected the protocol and UART has been initialised
    enum FrSkyProtocol _protocol;           // protocol used - detected using SerialManager's SERIALX_PROTOCOL parameter

    uint16_t _crc;
    uint32_t _last_frame1_ms;
    uint32_t _last_frame2_ms;

    bool _battery_data_ready;
    uint16_t _batt_remaining;
    uint16_t _batt_volts;
    uint16_t _batt_amps;

    bool _sats_data_ready;
    uint16_t gps_sats;

    bool _gps_data_ready;
    bool _pos_gps_ok;
    uint16_t _course_in_degrees;
    char _lat_ns, _lon_ew;
    uint16_t _latdddmm;
    uint16_t _latmmmm;
    uint16_t _londddmm;
    uint16_t _lonmmmm;	
    uint16_t _alt_gps_meters;
    uint16_t _alt_gps_cm;
    int16_t _speed_in_meter;
    uint16_t _speed_in_centimeter;

    bool _baro_data_ready;
    int16_t _baro_alt_meters;
    uint16_t _baro_alt_cm;

    bool _mode_data_ready;
    uint8_t _mode; 

    uint8_t _fas_call;
    uint8_t _gps_call;
    uint8_t _vario_call;
    uint8_t _various_call;

    uint8_t _sport_status;

    uint8_t _ignore_rx_chars;

    enum _states_serial {
        FRSKY_STATE_WAIT_POLL_REQUEST,
        FRSKY_STATE_WAIT_SENSOR_ID,
        FRSKY_STATE_WAIT_TX_DONE,
    };

};

#endif
#endif
