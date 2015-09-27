// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*

   Inspired by work done here https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>

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

/* 
   FRSKY Telemetry library
*/
#include "AP_Frsky_VRBRAIN.h"

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN && APM_BUILD_TYPE(APM_BUILD_ArduCopter)
extern uint8_t textId, g_severity;


//constructor
AP_Frsky_VRBRAIN::AP_Frsky_VRBRAIN(AP_AHRS &ahrs, AP_BattMonitor &battery) :
    _ahrs(ahrs),
    _battery(battery),
    _port(NULL),
    _initialised_uart(false),
    _protocol(FrSkyUnknown),
    _crc(0),
    _last_frame1_ms(0),
    _last_frame2_ms(0),
    _battery_data_ready(false),
    _batt_remaining(0),
    _batt_volts(0),
    _batt_amps(0),
    _sats_data_ready(false),
    gps_sats(0),
    _gps_data_ready(false),
    _pos_gps_ok(false),
    _course_in_degrees(0),
    _lat_ns(0),
    _lon_ew(0),
    _latdddmm(0),
    _latmmmm(0),
    _londddmm(0),
    _lonmmmm(0),
    _alt_gps_meters(0),
    _alt_gps_cm(0),
    _speed_in_meter(0),
    _speed_in_centimeter(0),
    _baro_data_ready(false),
    _baro_alt_meters(0),
    _baro_alt_cm(0),
    _mode_data_ready(false),
    _mode(0),
    _fas_call(0),
    _gps_call(0),
    _vario_call(0),
    _various_call(0),
    _sport_status(0)
    {}

// init - perform require initialisation including detecting which protocol to use
void AP_Frsky_VRBRAIN::init(const AP_SerialManager& serial_manager)
{
    // check for FRSky_DPort
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FRSky_DPort, 0))) {
        _protocol = FrSkyDPORT;
        _port->begin(AP_SERIALMANAGER_FRSKY_DPORT_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
        _initialised_uart = true;   // SerialManager initialises uart for us
    } else if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FRSky_SPort, 0))) {
        // check for FRSky_SPort
        _protocol = FrSkySPORT;
        _gps_call = 0;
        _fas_call = 0;
        _vario_call = 0 ;
        _various_call = 0 ;
        _gps_data_ready = false;
        _battery_data_ready = false;
        _baro_data_ready = false;
        _mode_data_ready = false;
        _sats_data_ready = false;
        _sport_status = 0;
        _ignore_rx_chars = 0;
        _port->begin(AP_SERIALMANAGER_FRSKY_SPORT_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
        _initialised_uart = true;   // SerialManager initialises uart for us
    }

    if (_port != NULL) {
	//noop;
    }
}

/*
  send_frames - sends updates down telemetry link for both DPORT and SPORT protocols
  should be called by main program at 50hz to allow poll for serial bytes
  coming from the receiver for the SPort protocol
*/
void AP_Frsky_VRBRAIN::send_frames (uint8_t control_mode, uint8_t simple_mode, int32_t altitude, int8_t alt_mode, float climbrate, int16_t throttle)
{
    // return immediately if not initialised
    if (!_initialised_uart) {
        return;
    }

    if (_protocol == FrSkySPORT) {
        // check for sport bytes
        check_sport_input (control_mode, simple_mode, altitude, alt_mode, climbrate, throttle);
    } else {
        uint32_t now = hal.scheduler->millis();

        // send frame1 every 200ms
        if (now - _last_frame1_ms > 200) {
            _last_frame1_ms = now;
            frsky_send_frame1(control_mode);
        }

        // send frame2 every second
        if (now - _last_frame2_ms > 1000) {
            _last_frame2_ms = now;
            frsky_send_frame2();
        }
    }
}


/*
  init_uart_for_sport - initialise uart for use by sport
  this must be called from sport_tick which is called from the 1khz scheduler
  because the UART begin must be called from the same thread as it is used from
 */
void AP_Frsky_VRBRAIN::init_uart_for_sport()
{
//NOOP;
}

/*
  send_hub_frame - send frame1 and frame2 when protocol is FrSkyDPORT
  frame 1 is sent every 200ms with baro alt, nb sats, batt volts and amp, control_mode
  frame 2 is sent every second with gps position data
*/
void AP_Frsky_VRBRAIN::send_hub_frame()
{
//NOOP;

}

/*
  sport_tick - main call to send updates to transmitter when protocol is FrSkySPORT
  called by scheduler at a high rate
*/
void AP_Frsky_VRBRAIN::sport_tick(void)
{
//NOOP

}


/* 
   simple crc implementation for FRSKY telem S-PORT
*/
void AP_Frsky_VRBRAIN::calc_crc(uint8_t byte)
{
//NOOP

}

/*
 * send the crc at the end of the S-PORT frame
 */
void AP_Frsky_VRBRAIN::send_crc() 
{
//NOOP

}


/*
  send 1 byte and do the byte stuffing Frsky stuff 
  This can send more than 1 byte eventually
*/
void AP_Frsky_VRBRAIN::frsky_send_byte(uint8_t value)
{
    switch (value) {
        case 0x5D:
        case 0x5E:
            _port->write(0x5D);
            _port->write(value ^ 0x60);
            break;
        default:
            _port->write(value);
            break;
    }

}

/**
 * Sends a 0x5E start/stop byte.
 */
void AP_Frsky_VRBRAIN::frsky_send_hub_startstop()
{
//NOOP;

}

/*
  add sport protocol for frsky tx module 
*/
void AP_Frsky_VRBRAIN::frsky_send_sport_prim()
{
//NOOP;

}


/**
 * Sends one data id/value pair.
 */
void  AP_Frsky_VRBRAIN::frsky_send_data(uint8_t id, int16_t data)
{

    // Cast data to unsigned, because signed shift might behave incorrectly
    uint16_t udata = data;

    _port->write (0x5E);            // Send a 0x5E start/stop byte
    frsky_send_byte (id);
    frsky_send_byte (udata);        // LSB
    frsky_send_byte (udata >> 8);   // MSB

}

/*
 * calc_baro_alt : send altitude in Meters based on ahrs estimate
 */
void AP_Frsky_VRBRAIN::calc_baro_alt()
{
//NOOP;

}

/**
 * Formats the decimal latitude/longitude to the required degrees/minutes.
 */
float  AP_Frsky_VRBRAIN::frsky_format_gps(float dec)
{
    uint8_t dm_deg = (uint8_t) dec;
    return (dm_deg * 100.0f) + (dec - dm_deg) * 60;
}

/*
 * prepare latitude and longitude information stored in member variables
 */

void AP_Frsky_VRBRAIN::calc_gps_position()
{
//NOOP;

}

/*
 * prepare battery information stored in member variables
 */
void AP_Frsky_VRBRAIN::calc_battery()
{
//NOOP

}

/*
 * prepare sats information stored in member variables
 */
void AP_Frsky_VRBRAIN::calc_gps_sats()
{
//NOOP;

}

/*
 * send number of gps satellite and gps status eg: 73 means 7 satellite and 3d lock
 */
void AP_Frsky_VRBRAIN::send_gps_sats()
{
//NOOP

}

/*
 * send control_mode as Temperature 1 (TEMP1)
 */
void AP_Frsky_VRBRAIN::send_mode(void)
{
//NOOP;

}

/*
 * send barometer altitude integer part . Initialize baro altitude
 */
void AP_Frsky_VRBRAIN::send_baro_alt_m(void)
{
//NOOP;

}

/*
 * send barometer altitude decimal part
 */
void AP_Frsky_VRBRAIN::send_baro_alt_cm(void)
{
//NOOP;

}

/*
 * send battery remaining
 */
void AP_Frsky_VRBRAIN::send_batt_remain(void)
{
//NOOP;

}

/*
 * send battery voltage 
 */
void AP_Frsky_VRBRAIN::send_batt_volts(void)
{
//NOOP;

}

/*
 * send current consumptiom 
 */
void AP_Frsky_VRBRAIN::send_current(void)
{
//NOOP;

}

/*
 * send heading in degree based on AHRS and not GPS 
 */
void AP_Frsky_VRBRAIN::send_heading(void)
{
//NOOP;

}

/*
 * send gps lattitude degree and minute integer part; Initialize gps info
 */
void AP_Frsky_VRBRAIN::send_gps_lat_dd(void)
{
//NOOP;

}

/*
 * send gps lattitude minutes decimal part 
 */
void AP_Frsky_VRBRAIN::send_gps_lat_mm(void)
{
//NOOP

}

/*
 * send gps North / South information 
 */
void AP_Frsky_VRBRAIN::send_gps_lat_ns(void)
{
//NOOP;

}

/*
 * send gps longitude degree and minute integer part 
 */
void AP_Frsky_VRBRAIN::send_gps_lon_dd(void)
{
//NOOP;

}

/*
 * send gps longitude minutes decimal part 
 */
void AP_Frsky_VRBRAIN::send_gps_lon_mm(void)
{
//NOOP;

}

/*
 * send gps East / West information 
 */
void AP_Frsky_VRBRAIN::send_gps_lon_ew(void)
{
//NOOP;

}

/*
 * send gps speed integer part
 */
void AP_Frsky_VRBRAIN::send_gps_speed_meter(void)
{
//NOOP;

}

/*
 * send gps speed decimal part
 */
void AP_Frsky_VRBRAIN::send_gps_speed_cm(void)
{
//NOOP;

}

/*
 * send gps altitude integer part
 */
void AP_Frsky_VRBRAIN::send_gps_alt_meter(void)
{
//NOOP;

}

/*
 * send gps altitude decimals
 */
void AP_Frsky_VRBRAIN::send_gps_alt_cm(void)
{
//NOOP;

}

// Send frame 2 (every 1000ms): course(heading), latitude, longitude, ground speed, GPS altitude (D-Port receivers only)
void AP_Frsky_VRBRAIN::frsky_send_frame2 (void)
{
    // we send the heading based on the ahrs instead of GPS course which is not very usefull
    uint16_t course_in_degrees = (_ahrs.yaw_sensor / 100) % 360;
    frsky_send_data (FRSKY_ID_GPS_COURS_BP, course_in_degrees);

    const AP_GPS &gps = _ahrs.get_gps ();
    bool posok = (gps.status() >= 3);
    if  (posok) {
        // send formatted frame
        float lat = 0, lon = 0,  alt = 0, speed= 0;
        char lat_ns = 0, lon_ew = 0;
        Location loc = gps.location();//get gps instance 0

        lat = frsky_format_gps (fabsf(loc.lat/10000000.0));
        uint16_t latdddmm = lat;
        uint16_t latmmmm = (lat - latdddmm) * 10000;
        lat_ns = (loc.lat < 0) ? 'S' : 'N';

        lon = frsky_format_gps (fabsf(loc.lng/10000000.0));
        uint16_t londddmm = lon;
        uint16_t lonmmmm = (lon - londddmm) * 10000;
        lon_ew = (loc.lng < 0) ? 'W' : 'E';

        alt = loc.alt / 100;
        uint16_t alt_gps_meters = alt;
        uint16_t alt_gps_cm = (alt - alt_gps_meters) * 100;

        speed = gps.ground_speed ();
        uint16_t speed_in_meter = speed;
        uint16_t speed_in_centimeter = (speed - speed_in_meter) * 100;

        frsky_send_data (FRSKY_ID_GPS_LAT_BP, latdddmm);
        frsky_send_data (FRSKY_ID_GPS_LAT_AP, latmmmm);
        frsky_send_data (FRSKY_ID_GPS_LAT_NS, lat_ns);

        frsky_send_data (FRSKY_ID_GPS_LONG_BP, londddmm);
        frsky_send_data (FRSKY_ID_GPS_LONG_AP, lonmmmm);
        frsky_send_data (FRSKY_ID_GPS_LONG_EW, lon_ew);

        frsky_send_data (FRSKY_ID_GPS_SPEED_BP, speed_in_meter);
        frsky_send_data (FRSKY_ID_GPS_SPEED_AP, speed_in_centimeter);

        frsky_send_data (FRSKY_ID_GPS_ALT_BP, alt_gps_meters);
        frsky_send_data (FRSKY_ID_GPS_ALT_AP, alt_gps_cm);
    }
}

// Check for input bytes from S.Port and answer accordingly
void AP_Frsky_VRBRAIN::check_sport_input(uint8_t control_mode, uint8_t simple_mode, int32_t altitude, int8_t alt_mode, float climbrate, int16_t throttle)
{
    static uint8_t state_serial = FRSKY_STATE_WAIT_POLL_REQUEST;
    static uint8_t state_sensor = 0;
    uint16_t hdop;

    while (_port->available()) {
        uint8_t b = _port->read ();
        switch (state_serial) {
            case FRSKY_STATE_WAIT_TX_DONE:             // because RX and TX are connected, we need to ignore bytes transmitted by us
                if (--_ignore_rx_chars == 0)
                    state_serial = FRSKY_STATE_WAIT_POLL_REQUEST;
                break;
            case FRSKY_STATE_WAIT_POLL_REQUEST:        // we get polled ~ every 12 ms (83 Hz)
                if (b == FRSKY_POLL_REQUEST)
                    state_serial = FRSKY_STATE_WAIT_SENSOR_ID;
                break;
            case FRSKY_STATE_WAIT_SENSOR_ID:
                if (b == FRSKY_SENSOR_ID) {
                    state_serial = FRSKY_STATE_WAIT_TX_DONE;
                    switch (state_sensor++) {          // send one value every second poll (24 ms)
                        case 0:     // Vfas (Battery voltage [V])
                            frsky_send_value (FRSKY_ID2_VOLTAGE, _battery.voltage()*100);
                            break;
                        case 1:     // Curr (Battery current [A])
                            frsky_send_value (FRSKY_ID2_CURRENT, _battery.current_amps()*10);
                            break;
                        case 2:     // Fuel (Flightmode as defined in defines.h + Simple mode + Armed/Disarmed)
                            frsky_send_value (FRSKY_ID2_FUEL, (control_mode<<3) | (simple_mode<<1) | hal.util->get_soft_armed());
                            break;
                        case 3:     // Hdg (Heading, [°])
                            frsky_send_value (FRSKY_ID2_HEADING, _ahrs.yaw_sensor);
                            break;
                        case 4:     // AccX (HDOP)
                            hdop = _ahrs.get_gps().get_hdop() / 4;
							if (hdop > 255) {
								hdop = 255;
							}
                            frsky_send_value (FRSKY_ID2_ACCX, hdop*100);
                            break;
                        case 5:     // AccY (Roll angle [°])
							frsky_send_value (FRSKY_ID2_ACCY, _ahrs.roll_sensor);
                            break;
                        case 6:     // AccZ (Pitch angle [°])
							frsky_send_value (FRSKY_ID2_ACCZ, _ahrs.pitch_sensor);
                            break;
                        case 7:     // RPM (Throttle [%] + Altitude mode, = ^v)
                            frsky_send_value (FRSKY_ID2_RPM, (throttle<<2) | alt_mode);
                            break;
                        case 8:     // VSpd (Vertical speed [m/s])
                            frsky_send_value (FRSKY_ID2_VARIO, (int32_t) climbrate);
                            break;
                        case 9:     // Alt (Baro height above ground [m])
                            if (hal.util->get_soft_armed()==true) {
                                frsky_send_value (FRSKY_ID2_ALTITUDE, altitude);
                            } else {
                                frsky_send_value (FRSKY_ID2_ALTITUDE, 0);
                            }
                            break;
                        case 10:    // GAlt (GPS height above ground [m])
                            if (hal.util->get_soft_armed()==true && _ahrs.get_gps().status()>=3) {
                                frsky_send_value (FRSKY_ID2_GPS_ALT, _ahrs.get_gps().location().alt);
                            } else {
                                frsky_send_value (FRSKY_ID2_GPS_ALT, 0);
                            }
                            break;
                        case 11:    // GPS Latitude
                            if (hal.util->get_soft_armed()==true && _ahrs.get_gps().status()>=3) {
                                uint32_t pos;
                                if (_ahrs.get_gps().location().lat > 0) {
                                    pos = 0;                // north latitude: id 00
                                } else {
                                    pos = 1 << 30;          // south latitude: id 01
                                }
                                frsky_send_value (FRSKY_ID2_GPS_LON_LAT, pos | abs(_ahrs.get_gps().location().lat)/100*6);
                            } else {
                                frsky_send_value (FRSKY_ID2_GPS_LON_LAT, 0);
                            }
                            break;
                        case 12:    // GPS Longitude
                            if (hal.util->get_soft_armed()==true && _ahrs.get_gps().status()>=3) {
                                uint32_t pos;
                                if (_ahrs.get_gps().location().lng > 0) {
                                    pos = 2 << 30;          // east longitude: id 10
                                } else {
                                    pos = 3 << 30;          // west longitude: id 11
                                }
                                frsky_send_value (FRSKY_ID2_GPS_LON_LAT, pos | abs(_ahrs.get_gps().location().lng)/100*6);
                            } else {
                                frsky_send_value (FRSKY_ID2_GPS_LON_LAT, 0);
                            }
                            break;
                        case 13:    // Spd (Ground speed [km/h])
                            if (hal.util->get_soft_armed()==true && _ahrs.get_gps().status()>=3) {
                                frsky_send_value (FRSKY_ID2_SPEED, _ahrs.get_gps().ground_speed()*543*3.6);
                            } else {
                                frsky_send_value (FRSKY_ID2_SPEED, 0);
                            }
                            break;
                        case 14:    // T1 (Number of satellites used for navigation + Lock Mode as defined in AP_GPS.h)
                            frsky_send_value (FRSKY_ID2_T1, ((_ahrs.get_gps().num_sats()&0x1F)<<3) | (_ahrs.get_gps().status()&0x07));
                            break;
                        case 15:    // T2 (Message ID + severity as defined in GCS_MAVLink.h)
							// text messages are sent multiple times to make sure they really arrive (even on radio glitches)
							#define TX_REPEATS 4
                            static uint8_t  tx_cnt=TX_REPEATS;
                            static uint32_t val=0;
                            if (tx_cnt > 0 && textId > 0) {
								if (tx_cnt == TX_REPEATS) {
                                	val = (textId<<3) | (g_severity&0x07);
                                }
                                tx_cnt--;
                            }
                            frsky_send_value (FRSKY_ID2_T2, val);
                            if (tx_cnt == 0) {
                               	textId     = 0;
                               	g_severity = 0;
                               	val        = 0;
                               	tx_cnt     = TX_REPEATS;
                            }
                            state_sensor = 0;			// Reset state counter on last state
                            break;
                    }
                } else {
                    state_serial = FRSKY_STATE_WAIT_POLL_REQUEST;
                }
                break;
            }
        }
}

// Send one byte, do byte stuffing and crc calculation (S-Port receivers only)
void AP_Frsky_VRBRAIN::frsky_send_char (uint8_t b)
{
    switch (b) {
        case 0x7D:
        case 0x7E:
            _port->write (    0x7D);
            _port->write (b ^ 0x20);
            _ignore_rx_chars++;
            break;
        default:
            _port->write (b);
            break;
    }

    _ignore_rx_chars++;

    _crc += b;                    // checksum calculation is based on data before byte-stuffing
    _crc += (_crc >> 8);
    _crc &= 0xFF;
}

// Send one data id/value pair (S-Port receivers only)
void AP_Frsky_VRBRAIN::frsky_send_value (uint16_t valueid, uint32_t value)
{
	_crc = 0;
    frsky_send_char (FRSKY_DATA_FRAME);
    frsky_send_char ((valueid >>  0) & 0xFF);
    frsky_send_char ((valueid >>  8) & 0xFF);
    frsky_send_char ((value   >>  0) & 0xFF);
    frsky_send_char ((value   >>  8) & 0xFF);
    frsky_send_char ((value   >> 16) & 0xFF);
    frsky_send_char ((value   >> 24) & 0xFF);
    frsky_send_char (0xFF - _crc);
}

// Send frame 1 (every 200ms): barometer altitude, battery voltage + current (D-Port receivers only)
void AP_Frsky_VRBRAIN::frsky_send_frame1 (uint8_t mode)
{
    struct Location loc;
    float battery_amps = _battery.current_amps();
    float baro_alt = 0; // in meters
    bool posok = _ahrs.get_position(loc);
    if (posok) {
        baro_alt = loc.alt * 0.01f; // convert to meters
        if (!loc.flags.relative_alt) {
            baro_alt -= _ahrs.get_home().alt * 0.01f; // subtract home if set
        }
    }
    const AP_GPS &gps = _ahrs.get_gps ();

    // GPS status is sent as num_sats*10 + status, to fit into a uint8_t
    uint8_t T2 = gps.num_sats() * 10 + gps.status();

    frsky_send_data (FRSKY_ID_TEMP1, mode);
    frsky_send_data (FRSKY_ID_TEMP2, T2);

    // Note that this isn't actually barometric altitdue, it is the AHRS estimate of altitdue above home
    uint16_t baro_alt_meters = (uint16_t)baro_alt;
    uint16_t baro_alt_cm = (baro_alt - baro_alt_meters) * 100;
    frsky_send_data (FRSKY_ID_BARO_ALT_BP, baro_alt_meters);
    frsky_send_data (FRSKY_ID_BARO_ALT_AP, baro_alt_cm);

    frsky_send_data (FRSKY_ID_FUEL, roundf(_battery.capacity_remaining_pct()));

    frsky_send_data (FRSKY_ID_VFAS, roundf(_battery.voltage() * 10.0f));
    frsky_send_data (FRSKY_ID_CURRENT, (battery_amps < 0) ? 0 : roundf(battery_amps * 10.0f));
}


#endif
