/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_circle.pde - init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool Copter::circle_init(bool ignore_checks)
{
	uint32_t dist = 0;

    if (position_ok() || ignore_checks) {
        circle_pilot_yaw_override = false;

        // initialize speeds and accelerations
        pos_control.set_speed_xy(wp_nav.get_speed_xy());
        pos_control.set_accel_xy(wp_nav.get_wp_acceleration());
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // Determine which switch is used to control circle behaviour
        if(g.ch7_option == AUXSW_CIRCLE_SET_CNTR) {
            circle_aux_chan = 7;
        } else if (g.ch8_option == AUXSW_CIRCLE_SET_CNTR) {
            circle_aux_chan = 8;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        } else if (g.ch9_option == AUXSW_CIRCLE_SET_CNTR) {
            circle_aux_chan = 9;
#endif
        } else if (g.ch10_option == AUXSW_CIRCLE_SET_CNTR) {
            circle_aux_chan = 10;
        } else if (g.ch11_option == AUXSW_CIRCLE_SET_CNTR) {
            circle_aux_chan = 11;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        } else if (g.ch12_option == AUXSW_CIRCLE_SET_CNTR) {
            circle_aux_chan = 12;
#endif
        } else {
            circle_aux_chan = 0;
        }
       
        // Calculate distance from saved center and set it as the radius.
        Vector3f cur_loc = pv_location_to_vector(current_loc);
        if(circle_aux_chan != 0) {
            dist = labs(pythagorous2(cur_loc.x - circle_nav.get_center().x , cur_loc.y - circle_nav.get_center().y ));
        }
        if (read_circle_chan() == AUX_SWITCH_MIDDLE) {
            circle_nav.set_radius(dist);
       } else if (read_circle_chan() == AUX_SWITCH_HIGH) {
           if(dist > 150.0f) {
               circle_nav.set_radius(dist);
        	} else {
                circle_nav.set_radius(0);
            }
        }
        // initialise circle controller including setting the circle center based on vehicle speed
        if(ap.circle_center_set) {
            circle_nav.init(circle_nav.get_center());
            set_circle_center(false);
        } else {
            circle_nav.init();
        }

        return true;
    }else{
        return false;
    }
}

uint8_t Copter::read_circle_chan() {

    switch(circle_aux_chan) {
        case 7:
        	return read_3pos_switch(g.rc_7.radio_in);
        	break;
        case 8:
        	return read_3pos_switch(g.rc_8.radio_in);
        	break;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        case 9:
        	return read_3pos_switch(g.rc_9.radio_in);
        	break;
#endif
        case 10:
        	return read_3pos_switch(g.rc_10.radio_in);
        	break;
        case 11:
        	return read_3pos_switch(g.rc_11.radio_in);
        	break;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        case 12:
        	return read_3pos_switch(g.rc_12.radio_in);
        	break;
#endif
        default:
        	return 0;
	}
}
// circle_run - runs the circle flight mode
// should be called at 100hz or more
void Copter::circle_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete || !motors.get_interlock()) {
        // To-Do: add some initialisation of position controllers
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        pos_control.set_alt_target_to_current_alt();
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
        if (!is_zero(target_yaw_rate)) {
            circle_pilot_yaw_override = true;
        }

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->control_in);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
        //read pitch channel and use it to update circle radius
        // To-Do: make variable increase based on stick position
        if(channel_pitch->control_in != 0) {
            float circ_radius;
            if(channel_pitch->control_in > 0) {
                circ_radius = (float)circle_nav.get_radius() - 0.5f; // at 100Hz makes it 50 cm every second
            } else if (channel_pitch->control_in < 0) {
                circ_radius = (float)circle_nav.get_radius() + 0.5f; // at 100Hz makes it 50 cm every second
            }
            circ_radius = constrain_float(circ_radius,0.0f,10000.0f);
            circle_nav.set_radius(circ_radius);
        }

        //read roll channel and use it to update circle rate
        // To-Do: make variable increase based on stick position
        if(channel_roll->control_in != 0) {
            float circ_rate;
            if(channel_roll->control_in > 0) {
                circ_rate = (float)circle_nav.get_rate() + 0.05f; // at 100Hz makes it 5 deg/s every second
            } else if (channel_roll->control_in < 0) {
                circ_rate = (float)circle_nav.get_rate() - 0.05f;// at 100Hz makes it 5 deg/s every second
            }
            circ_rate = constrain_float(circ_rate, 0.0f, 90.0f);
            circle_nav.set_rate(circ_rate);
        }
    }

    // run circle controller
    circle_nav.update();

    // call attitude controller
    if (circle_pilot_yaw_override) {
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), target_yaw_rate);
    }else{
        attitude_control.angle_ef_roll_pitch_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), circle_nav.get_yaw(),true);
    }

    // run altitude controller
    if (sonar_enabled && (sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        // if sonar is ok, use surface tracking
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
    }
    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
    pos_control.update_z_controller();
}
