#include "Copter.h"
#include <AP_Proximity/AP_Proximity.h>

#if MODE_ZIGZAGVERT_ENABLED == ENABLED

/*
* Init and run calls for zigzag flight mode
*/

#define ZIGZAG_WP_RADIUS_CM 300
#define ZIGZAG_LINE_INFINITY -1

const AP_Param::GroupInfo ModeZigZagVert::var_info[] = {
    // @Param: AUTO_ENABLE_VERT
    // @DisplayName: ZigZagVert auto enable/disable
    // @Description: Allows you to enable (1) or disable (0) ZigZag auto feature
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE_VERT", 1, ModeZigZagVert, _auto_enabled, 0, AP_PARAM_FLAG_ENABLE),

#if HAL_SPRAYER_ENABLED
    // @Param: SPRAYER_VERT
    // @DisplayName: Auto sprayer in ZigZag
    // @Description: Enable the auto sprayer in ZigZag mode. SPRAY_ENABLE = 1 and SERVOx_FUNCTION = 22(SprayerPump) / 23(SprayerSpinner) also must be set. This makes the sprayer on while moving to destination A or B. The sprayer will stop if the vehicle reaches destination or the flight mode is changed from ZigZag to other.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("SPRAY_VERT", 2, ModeZigZagVert, _spray_enabled, 0),
#endif // HAL_SPRAYER_ENABLED

    // @Param: WP_DELAY_VERT
    // @DisplayName: The delay for zigzag waypoint
    // @Description: Waiting time after reached the destination
    // @Units: s
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("VWP_DEL", 3, ModeZigZagVert, _wp_delay, 0),

    // @Param: SIDE_DIST_VERT
    // @DisplayName: Sideways distance in ZigZag auto
    // @Description: The distance to move sideways in ZigZag mode
    // @Units: m
    // @Range: 0.1 100
    // @User: Advanced
    AP_GROUPINFO("VERT_DIST", 4, ModeZigZagVert, _side_dist, 4),

    // @Param: DIRECTION_VERT
    // @DisplayName: Sideways direction in ZigZag auto
    // @Description: The direction to move sideways in ZigZag mode
    // @Values: 0:forward, 1:right, 2:backward, 3:left
    // @User: Advanced
    AP_GROUPINFO("VERT_DIR", 5, ModeZigZagVert, _direction, 0),

    // @Param: LINE_NUM_VERT
    // @DisplayName: Total number of lines
    // @Description: Total number of lines for ZigZag auto if 1 or more. -1: Infinity, 0: Just moving to sideways
    // @Range: -1 32767
    // @User: Advanced
    AP_GROUPINFO("VERT_LINE", 6, ModeZigZagVert, _line_num, 0),

    AP_GROUPINFO("WALL_DIST", 7, ModeZigZagVert, _wall_dist, 0),

   // AP_GROUPINFO("RGN_QDRT", 8, ModeZigZagVert, _rgn_quad, 0),

    AP_GROUPINFO("MAX_WLDT", 8, ModeZigZagVert, _max_wall_detect, 0),




    AP_GROUPEND
};

ModeZigZagVert::ModeZigZagVert(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise zigzag controller
bool ModeZigZagVert::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        // apply simple mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        float target_roll, target_pitch;
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);


    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise waypoint state
    stage = STORING_POINTS;
    dest_A.zero();
    dest_B.zero();

    // initialize zigzag auto
    init_auto();

    return true;
}

// perform cleanup required when leaving zigzag mode
void ModeZigZagVert::exit()
{
    // The sprayer will stop if the flight mode is changed from ZigZag to other
    spray(false);
}

// run the zigzag controller
// should be called at 100hz or more
void ModeZigZagVert::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // set the direction and the total number of lines
    zigzag_vertdirection = (VertDirection)constrain_int16(_direction, 0, 3);
    line_num = constrain_int16(_line_num, ZIGZAG_LINE_INFINITY, 32767);

    yaw_deg = copter.ahrs.get_yaw()*57.3;
//    gcs().send_text(MAV_SEVERITY_INFO, "YAW in rads %f", copter.ahrs.get_yaw());

    // if (copter.gps.have_gps_yaw()) {
    //     float yaw_acc_deg;
    //     uint32_t yaw_time_ms;
    //     if (copter.gps.gps_yaw_deg(yaw_deg, yaw_acc_deg, yaw_time_ms) && yaw_time_ms != last_gps_yaw_ms) {
    //         last_gps_yaw_ms = yaw_time_ms;
    //     }
    // }
//gcs().send_text(MAV_SEVERITY_INFO, "YAW in sin %f, yaw deg %f", sin(yaw_deg), yaw_deg);
    // auto control
    if (stage == AUTO) {
        if (is_disarmed_or_landed() || !motors->get_interlock()) {
            // vehicle should be under manual control when disarmed or landed
            return_to_manual_control_vert(false);
        } else if (reached_destination()) {
            // if vehicle has reached destination switch to manual control or moving to A or B
            AP_Notify::events.waypoint_complete = 1;
            if (is_auto) {
                if (line_num == ZIGZAG_LINE_INFINITY || line_count < line_num) {
                    if(auto_stage == AutoState::CorrectWall){
                         do_wall_correct();
                    }
                    else if (auto_stage == AutoState::SIDEWAYS) {
                        save_or_move_to_destination_vert((ab_dest_stored == VertDestination::vertA) ? VertDestination::vertB : VertDestination::vertA);
                    } else {
                        // spray off
                        spray(false);
                        move_to_side();
                    }
                } else {
                    init_auto();
                    return_to_manual_control_vert(true);
                }
            } else {
                return_to_manual_control_vert(true);
            }
        } else {
            auto_control();
        }
    }

    // manual control
    if (stage == STORING_POINTS || stage == MANUAL_REGAIN) {
        // receive pilot's inputs, do position and attitude control
        manual_control();
    }
}

// save current position as A or B.  If both A and B have been saved move to the one specified
void ModeZigZagVert::save_or_move_to_destination_vert(VertDestination ab_destvert)
{
    // get current position as an offset from EKF origin
    const Vector2f curr_pos {inertial_nav.get_position_xy_cm()};

    // handle state machine changes
    switch (stage) {

        case STORING_POINTS:
            if (ab_destvert == VertDestination::vertA) {
                // store point A
                dest_A = curr_pos;
                gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: point A stored");
                AP::logger().Write_Event(LogEvent::ZIGZAG_STORE_A);
                walldist_A = copter.rangefinder_state.alt_cm;

            if(yaw_deg > 0 && yaw_deg < 90){
                    _rgn_quad = 1;
                }
                else if (yaw_deg > 90 && yaw_deg < 180)
                {
                    _rgn_quad = 2;
                }
                else if (yaw_deg > 180 && yaw_deg < 270)
                {
                    _rgn_quad = 3;
                }
                else{
                    _rgn_quad = 4;
                }

                //float current_rgn1 = copter.rangefinder_state.alt_cm;
                // float current_rgn2 = copter.rangefinder_up_state.alt_cm;

                // float average_dist = (current_rgn1 + current_rgn2)/2;
                // const Vector2f curr_pos2d {inertial_nav.get_position_xy_cm()};
                // float yaw_radian = yaw_deg*0.0175;
                // float rgnd_dist_x = curr_pos2d.x + average_dist*cos(yaw_radian);
                // float rgnd_dist_y = curr_pos2d.y + average_dist*sin(yaw_radian);
                // gcs().send_text(MAV_SEVERITY_INFO, "obj dist x: %f, obj dist y: %f", rgnd_dist_x, rgnd_dist_y);
               // gcs().send_text(MAV_SEVERITY_INFO, "obj dist y: %f", rgn_dist_y);
                //gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: quadrant selected: %d", _rgn_quad);
            } else {
                // store point B
                dest_B = curr_pos;
                gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: point B stored");
                AP::logger().Write_Event(LogEvent::ZIGZAG_STORE_B);
                walldist_B = copter.rangefinder_state.alt_cm;
               // const Vector2f curr_pos2d {inertial_nav.get_position_xy_cm()};
                // gcs().send_text(MAV_SEVERITY_INFO, "current pos x: %f, current pos y: %f", curr_pos2d.x, curr_pos2d.y);
               // gcs().send_text(MAV_SEVERITY_INFO, "sin yaw: %f, yaw deg: %f", sin(yaw_deg*0.0175), yaw_deg);

            }
            // if both A and B have been stored advance state
            if (!dest_A.is_zero() && !dest_B.is_zero() && !is_zero((dest_B - dest_A).length_squared())) {
                stage = MANUAL_REGAIN;
                spray(false);
            } else if (!dest_A.is_zero() || !dest_B.is_zero()) {
                // if only A or B have been stored, spray on
                spray(true);
            }
            break;

        case AUTO:
        case MANUAL_REGAIN:
            // A and B have been defined, move vehicle to destination A or B
            Vector3f next_dest;
            bool terr_alt;
            if (calculate_next_dest(ab_destvert, stage == AUTO, next_dest, terr_alt)) {
                wp_nav->wp_and_spline_init();
                if (wp_nav->set_wp_destination(next_dest, terr_alt)) {
                    stage = AUTO;
                    auto_stage = AutoState::AB_MOVING;
                    ab_dest_stored = ab_destvert;
                    // spray on while moving to A or B
                    spray(true);
                    reach_wp_time_ms = 0;
                    if (is_auto == false || line_num == ZIGZAG_LINE_INFINITY) {
                        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: moving to %s", (ab_destvert == VertDestination::vertA) ? "A" : "B");
                    } else {
                        line_count++;
                        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: moving to %s (line %d/%d)", (ab_destvert == VertDestination::vertA) ? "A" : "B", line_count, line_num);
                    }
                }
            }
            break;
    }
}

void ModeZigZagVert::move_to_side()
{
    if (!dest_A.is_zero() && !dest_B.is_zero() && !is_zero((dest_B - dest_A).length_squared())) {
        Vector3f next_dest;
        bool terr_alt;
        if (calculate_side_dest(next_dest, terr_alt)) {
            wp_nav->wp_and_spline_init();
            if (wp_nav->set_wp_destination(next_dest, terr_alt)) {
                stage = AUTO;
                auto_stage = AutoState::CorrectWall;
                current_dest = next_dest;
                current_terr_alt = terr_alt;
                reach_wp_time_ms = 0;
                gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: moving vertically");
            }
        }
    }
}

void ModeZigZagVert::do_wall_correct()
{
    if (!dest_A.is_zero() && !dest_B.is_zero() && !is_zero((dest_B - dest_A).length_squared())) {
        Vector3f next_dest;
        bool terr_alt;
        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: Doing Wall Correct");
        if (calculate_wall_dist(next_dest, terr_alt)) {
                stage = AUTO;
                auto_stage = AutoState::SIDEWAYS;
                current_dest = next_dest;
                current_terr_alt = terr_alt;
                reach_wp_time_ms = 0; 
        }
    }
}

// return manual control to the pilot
void ModeZigZagVert::return_to_manual_control_vert(bool maintain_target)
{
    if (stage == AUTO) {
        stage = MANUAL_REGAIN;
        pos_control->relax_z_controller(0.0f);
        spray(false);
        loiter_nav->clear_pilot_desired_acceleration();
        if (maintain_target) {
            const Vector3f& wp_dest = wp_nav->get_wp_destination();
            loiter_nav->init_target(wp_dest.xy());
            if (wp_nav->origin_and_destination_are_terrain_alt()) {
                copter.surface_tracking.set_target_alt_cm(wp_dest.z);
            }
        } else {
            loiter_nav->init_target();
        }
        is_auto = false;
        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: manual control");
    }
}

// fly the vehicle to closest point on line perpendicular to dest_A or dest_B
void ModeZigZagVert::auto_control()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;

    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        //gcs().send_text(MAV_SEVERITY_INFO, "target_roll: %f, target_pitch: %f", target_roll, target_pitch);
    }
   // float yaw_rgn = -12000;



    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    // run waypoint controller
    const bool wpnav_ok = wp_nav->update_wpnav();

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
    //pos_control->update_xy_controller(); 
      // current_dist_rgn1 = copter.rangefinder_state.alt_cm;
   // current_dist_rgn2 = copter.rangefinder_up_state.alt_cm;
//     float rgn_error;

// if(current_dist_rgn2 < current_dist_rgn1){
//         rgn_error = current_dist_rgn2 - current_dist_rgn1;
// }
// else if(current_dist_rgn2 > current_dist_rgn1){
//         rgn_error = current_dist_rgn2 - current_dist_rgn1;
// }
// else{
//     rgn_error = 0;
// }
    //  gcs().send_text(MAV_SEVERITY_INFO, "rgn_error: %f", rgn_error);

    // if(rgn_error < 0){
    // yaw_val_scale = 0.002*rgn_error;
    // }
    // else{
    // yaw_val_scale = -0.002*rgn_error;
    // }
    //float prop_yaw = 5*rgn_error;
    //float kd_yaw = (rgn_error - der_yaw_preverror)*0.0001;
    //ki_yaw += rgn_error;
    //ki_yaw = 0.00001*ki_yaw;

  //  der_yaw_preverror = rgn_error;
    

    
    // if(rgn_error < -5){
    //     yaw_val_scale = -0.35;
    // }
    // else if(rgn_error > 5){
    //     yaw_val_scale = 0.35;
    // }
    // else{
    //     yaw_val_scale = 0.0;    
    // }
   // float output = prop_yaw + kd_yaw + ki_yaw;
    //output = constrain_float(-output, -0.1, 0.1);
   // output = constrain_float(-output, -0.1, 0.1);

  //  gcs().send_text(MAV_SEVERITY_INFO, "yaw_scale_val: %f", output);

   // float target_rgn_yawrate = get_pilot_desired_yaw_rate(output);

    // call attitude controller
    // roll & pitch from waypoint controller, yaw rate from pilot
    if(auto_stage == AutoState::CorrectWall){
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
         float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        gcs().send_text(MAV_SEVERITY_INFO, "tar_pitch: %f, tar_roll: %f", target_pitch, target_roll);
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // loiter_nav->set_pilot_desired_acceleration(0.0, output);
        // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);
        // gcs().send_text(MAV_SEVERITY_INFO, "pitch_val: %f", output);
    }
    else{
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    }
    
    
    //  AP_Proximity *proximity = AP_Proximity::get_singleton();

    // Proximity_Distance_Array dist_array;
    // if (proximity->get_horizontal_distances(dist_array)) {
    // current_rgn1 = dist_array.distance[4];
    // }



   // float rgn_out = 0.5*(_wall_dist - average_dist);

   // Vector3f vel_input;
   // float yaw_rad = yaw_deg*0.0175;
   // vel_input.x = rgn_out*cos(yaw_rad);
   // vel_input.y = rgn_out*sin(yaw_rad);
   // vel_input.z = 0;
  //  copter.set_target_velocity_NED(vel_input);    // if wpnav failed (because of lack of terrain data) switch back to pilot control for next iteration
    if (!wpnav_ok) {
        return_to_manual_control_vert(false);
    }
}

// manual_control - process manual control
void ModeZigZagVert::manual_control()
{
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        //gcs().send_text(MAV_SEVERITY_INFO, "tar_pitch: %f, tar_roll: %f", target_pitch, target_roll);
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        // make sure the climb rate is in the given range, prevent floating point errors
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we
        // do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // althold state machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        // gcs().send_text(MAV_SEVERITY_INFO, "loiter pitch: %f", loiter_nav->get_pitch());
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

// return true if vehicle is within a small area around the destination
bool ModeZigZagVert::reached_destination()
{
    // check if wp_nav believes it has reached the destination
    if (!wp_nav->reached_wp_destination()) {
        return false;
    }

    // check distance to destination
    if (wp_nav->get_wp_distance_to_destination() > ZIGZAG_WP_RADIUS_CM) {
        return false;
    }

    // wait at time which is set in zigzag_wp_delay
    uint32_t now = AP_HAL::millis();
    if (reach_wp_time_ms == 0) {
        reach_wp_time_ms = now;
    }
    return ((now - reach_wp_time_ms) >= (uint16_t)constrain_int16(_wp_delay, 0, 127) * 1000);
}

// calculate next destination according to vector A-B and current position
// use_wpnav_alt should be true if waypoint controller's altitude target should be used, false for position control or current altitude target
// terrain_alt is returned as true if the next_dest should be considered a terrain alt
bool ModeZigZagVert::calculate_next_dest(VertDestination ab_destvert, bool use_wpnav_alt, Vector3f& next_dest, bool& terrain_alt)
{
    // define start_pos as either destination A or B
    Vector2f start_pos = (ab_destvert == VertDestination::vertA) ? dest_A : dest_B;


    // calculate vector from A to B
    Vector2f AB_diff = dest_B - dest_A;

    // check distance between A and B
    if (is_zero(AB_diff.length_squared())) {
        return false;
    }

//     current_rgn1 = copter.rangefinder_state.alt_cm;
//     current_rgn2 = copter.rangefinder_up_state.alt_cm;
//     average_dist = (current_rgn1 + current_rgn2)/2;
   // gcs().send_text(MAV_SEVERITY_INFO, "destAx:%f,DestAy:%f,DestBx:%f,DestBy:%f", dest_A.x, dest_A.y,dest_B.x,dest_B.y);
//     float yaw_radian = yaw_deg*0.0175;
    // get distance from vehicle to start_pos
    const Vector2f curr_pos2d {inertial_nav.get_position_xy_cm()};
    Vector2f veh_to_start_pos = curr_pos2d - start_pos;
    gcs().send_text(MAV_SEVERITY_INFO, "cur_posx:%f,cur_posy:%f", curr_pos2d.x, curr_pos2d.y);

    // lengthen AB_diff so that it is at least as long as vehicle is from start point
    // we need to ensure that the lines perpendicular to AB are long enough to reach the vehicle
    float scalar = 1.0f;
    if (veh_to_start_pos.length_squared() > AB_diff.length_squared()) {
        scalar = veh_to_start_pos.length() / AB_diff.length();
        gcs().send_text(MAV_SEVERITY_INFO, "scalar:%f", scalar);
    }


//    float current_rgn1 = copter.rangefinder_state.alt_cm;
//    float current_rgn2 = copter.rangefinder_up_state.alt_cm;

//     float average_dist = (current_rgn1 + current_rgn2)/2;
    
//     rgn_dist_x = curr_pos2d.x + average_dist*cos(yaw_radian);
//     rgn_dist_y = curr_pos2d.y + average_dist*sin(yaw_radian);

    //gcs().send_text(MAV_SEVERITY_INFO, "rgndistx:%f,rgndisty:%f", rgn_dist_x, rgn_dist_y);
    
//     float wall_diff = abs(walldist_A - walldist_B);
//     float glob_wall_distx;
//     float glob_wall_disty;
//     if(wall_diff > 10 && mv_once == false){
//         float dest_distB = sqrt((dest_B.x - curr_pos2d.x)*(dest_B.x - curr_pos2d.x) + (dest_B.y - curr_pos2d.y)*(dest_B.y - curr_pos2d.y));
//         float dest_distA = sqrt((dest_A.x - curr_pos2d.x)*(dest_A.x - curr_pos2d.x) + (dest_A.y - curr_pos2d.y)*(dest_A.y - curr_pos2d.y));
//         gcs().send_text(MAV_SEVERITY_INFO, "dest_distA:%f, dest_distB:%f", dest_distA, dest_distB);
//         if(dest_distB < dest_distA){
//             glob_wall_distx = rgn_dist_x - walldist_A*cos(yaw_radian);
//             glob_wall_disty = rgn_dist_y - walldist_A*sin(yaw_radian);
//             gcs().send_text(MAV_SEVERITY_INFO, "DestA walldist");
//             dest_B.x = glob_wall_distx;
//             dest_B.y = glob_wall_disty;
//         }
//         else{
//             glob_wall_distx = rgn_dist_x - walldist_B*cos(yaw_radian);
//             glob_wall_disty = rgn_dist_y - walldist_B*sin(yaw_radian);
//             gcs().send_text(MAV_SEVERITY_INFO, "DestB walldist");
//             dest_A.x = glob_wall_distx;
//             dest_A.y = glob_wall_disty;
//         }
//         gcs().send_text(MAV_SEVERITY_INFO, "walldistx:%f,walldisty:%f", glob_wall_distx,glob_wall_disty);
//         next_dest.x = glob_wall_distx;
//         next_dest.y = glob_wall_disty;
//         next_dest.z = inertial_nav.get_position_z_up_cm();
//         mv_once = true;
//         terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
//         return true;

//     } else{
//         glob_wall_distx = rgn_dist_x - _wall_dist*cos(yaw_radian);
//         glob_wall_disty = rgn_dist_y - _wall_dist*sin(yaw_radian);
//     }

//     float dist_mag = sqrt((glob_wall_distx - curr_pos2d.x)*(glob_wall_distx - curr_pos2d.x) + (glob_wall_disty - curr_pos2d.y)*(glob_wall_disty - curr_pos2d.y));
    //float rgn_dist_error = fabs(average_dist - _wall_dist);

//     Vector2f horizontal_perpendicular;

      
//         if (_rgn_quad == 2 || _rgn_quad == 4) {
//         float yaw_ab_sign = (-sin(yaw_radian) * AB_diff[1]) + (cos(yaw_radian) * -AB_diff[0]);
//         if (is_positive(yaw_ab_sign)) {
//             horizontal_perpendicular = Vector2f(AB_diff[1], -AB_diff[0]);
//         } else {
//             horizontal_perpendicular = Vector2f(-AB_diff[1], AB_diff[0]);
//         }
//     } else {
//         float yaw_ab_sign = (cos(yaw_radian) * AB_diff[1]) + (sin(yaw_radian) * -AB_diff[0]);
//         if (is_positive(yaw_ab_sign)) {
//             horizontal_perpendicular = Vector2f(AB_diff[1], -AB_diff[0]);
//         } else {
//             horizontal_perpendicular = Vector2f(-AB_diff[1], AB_diff[0]);
//         }
//     }

    //horizontal_perpendicular = horizontal_perpendicular.normalized()*dist_mag;
    
     Vector2f perp1 = start_pos + Vector2f(-AB_diff[1] * scalar, AB_diff[0] * scalar);
     Vector2f perp2 = start_pos + Vector2f(AB_diff[1] * scalar, -AB_diff[0] * scalar);
    //     // find the closest point on the perpendicular line
     const Vector2f closest2d = Vector2f::closest_point(curr_pos2d, perp1, perp2);
     next_dest.x = closest2d.x;
     next_dest.y = closest2d.y;
    // gcs().send_text(MAV_SEVERITY_INFO, "horizontal perp x: %f", horizontal_perpendicular.x);
    // gcs().send_text(MAV_SEVERITY_INFO, "horizontal perp y: %f", horizontal_perpendicular.y);
//  if(average_dist < _max_wall_detect){


//     if(_wall_dist > average_dist){
//         horizontal_perpendicular = curr_pos2d - horizontal_perpendicular.normalized()*dist_mag;
//         // find the closest point on the perpendicular line
//         //float wall_des_x = rgn_dist_x - _wall_dist*horizontal_perpendicular.x;
//         //float wall_des_y = rgn_dist_y - _wall_dist*horizontal_perpendicular.y;
//                 // create a line perpendicular to AB but originating at start_pos

//         const Vector2f closest2d = Vector2f::closest_point(horizontal_perpendicular, perp1, perp2);
//          next_dest.x = closest2d.x;
//          next_dest.y = closest2d.y;

//     }
//     else{
//         horizontal_perpendicular = curr_pos2d + horizontal_perpendicular.normalized()*dist_mag;
//         // find the closest point on the perpendicular line
//         //float wall_des_x = rgn_dist_x + _wall_dist*horizontal_perpendicular.x;
//         //float wall_des_y = rgn_dist_y + _wall_dist*horizontal_perpendicular.y;
//         const Vector2f closest2d = Vector2f::closest_point(horizontal_perpendicular, perp1, perp2);
//         next_dest.x = closest2d.x;
//         next_dest.y = closest2d.y;
//     }
//  }
//     else{
//     // find the closest point on the perpendicular line
//         const Vector2f closest2d = Vector2f::closest_point(curr_pos2d, perp1, perp2);
//          next_dest.x = closest2d.x;
//          next_dest.y = closest2d.y;
//     }



        //gcs().send_text(MAV_SEVERITY_INFO, "next dest x: %f, rgn_error x: %f", next_dest.x, glob_wall_dist.x);
         //gcs().send_text(MAV_SEVERITY_INFO, "next dest y: %f, rgn_error y: %f", next_dest.y, glob_wall_dist.y);


         // Calculate the desired ascent altitude based on the vertical component of AB_diff
        //float ascent_altitude = fabs(AB_diff.dot(veh_to_start_pos) / AB_diff.length());
        //float ascent_altitude = 0;
        // Use terrain altitude if available, otherwise use inertial_nav altitude
        terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
        // if (terrain_alt) {
        //     next_dest.z = copter.rangefinder_state.alt_cm_filt.get() + ascent_altitude;
        // } else {
        //     next_dest.z = inertial_nav.get_position_z_up_cm() + ascent_altitude;
        // }
        next_dest.z = inertial_nav.get_position_z_up_cm();

    return true;
}


bool ModeZigZagVert::calculate_wall_dist(Vector3f& next_dest, bool& terrain_alt)
{

    float current_dist_rgn1 = copter.rangefinder_state.alt_cm;
    float current_dist_rgn2 = copter.rangefinder_up_state.alt_cm;
    average_dist = (current_dist_rgn1+current_dist_rgn2)/2;

    float rgn_error = average_dist - _wall_dist;
    gcs().send_text(MAV_SEVERITY_INFO, "rgn_error: %f", rgn_error);

    // if(rgn_error < 0){
    // yaw_val_scale = 0.002*rgn_error;
    // }
    // else{
    // yaw_val_scale = -0.002*rgn_error;
    // }
    float prop_yaw = 0.08*rgn_error;
    float kd_yaw = (rgn_error - der_yaw_preverror)*0.0001;
    ki_yaw += rgn_error;
    ki_yaw = 0.00001*ki_yaw;

    der_yaw_preverror = rgn_error;

    output = prop_yaw + kd_yaw + ki_yaw;
        //float ascent_alt = fabs(AB_diff.dot(curr_pos2d) / AB_diff.length());
    // Calculate the vertical position based on the scalar
        terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();

    if(fabs(rgn_error) > 30){
        return false;
    }
    else{
        return true;
    }

    // return true;
}


// calculate side destination according to vertical vector A-B and current position
// terrain_alt is returned as true if the next_dest should be considered a terrain alt
bool ModeZigZagVert::calculate_side_dest(Vector3f& next_dest, bool& terrain_alt)
{
// Calculate vector from A to B in the xy plane
    Vector2f AB_diff = dest_B - dest_A;

    // Check distance between A and B
    if (is_zero(AB_diff.length_squared())) {
        return false;
    }
 
    // Set the horizontal position to the current position
    const Vector2f curr_pos2d {inertial_nav.get_position_xy_cm()};
    next_dest.x = curr_pos2d.x;
    next_dest.y = curr_pos2d.y;
        float ascent_alt = 0;
    if(zigzag_vertdirection == VertDirection::UPWARD){
         ascent_alt = _side_dist*100;

    }
    else if(zigzag_vertdirection == VertDirection::DOWNWARD){
        ascent_alt = (-_side_dist)*100;
    }
    else{
        ascent_alt = _side_dist*100;

    }
    next_dest.z = inertial_nav.get_position_z_up_cm() + ascent_alt;
    

        //float ascent_alt = fabs(AB_diff.dot(curr_pos2d) / AB_diff.length());
    // Calculate the vertical position based on the scalar
    terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
    // if (terrain_alt) {
    //         next_dest.z = copter.rangefinder_state.alt_cm_filt.get() + ascent_alt;
    //     } else {
    //         next_dest.z = inertial_nav.get_position_z_up_cm() + ascent_alt;
    //     }


        

    // Calculate the direction of the vertical ascent
   // Vector2f ascent_direction = AB_diff.normalized();  // Direction from A to B

    // // Calculate the side destination point in the xy plane
    // Vector2f current_position_xy = inertial_nav.get_position_xy_cm();

    //     // Set the x and y components of the next_dest to the calculated side destination in the xy plane
    // next_dest.x = current_position_xy.x;
    // next_dest.y = current_position_xy.y;
    // next_dest.z = inertial_nav.get_position_z_up_cm() + (_side_dist * ascent_direction.y);

    // // Get terrain altitude if available
    // terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
    // if (terrain_alt) {
    //     if (!copter.surface_tracking.get_target_alt_cm(next_dest.z)) {
    //         next_dest.z = copter.rangefinder_state.alt_cm_filt.get();
    //     }
    // }

    return true;
}

// run zigzag auto feature which is automate both AB and sideways
void ModeZigZagVert::run_auto()
{
    // exit immediately if we are disabled
    if (!_auto_enabled) {
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: In run_auto");
    // make sure both A and B point are registered and not when moving to A or B
    if (stage != MANUAL_REGAIN) {
        return;
    }

    is_auto = true;
    // resume if zigzag auto is suspended
    if (is_suspended && line_count <= line_num) {
        // resume the stage when it was suspended
        if (auto_stage == AutoState::AB_MOVING) {
            line_count--;
            save_or_move_to_destination_vert(ab_dest_stored);
        } else if (auto_stage == AutoState::SIDEWAYS) {
            wp_nav->wp_and_spline_init();
            if (wp_nav->set_wp_destination(current_dest, current_terr_alt)) {
                stage = AUTO;
                reach_wp_time_ms = 0;
                char const *dir[] = {"forward", "right", "backward", "left"};
                gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: moving to %s", dir[(uint8_t)zigzag_vertdirection]);
            }
        }
    } else {
        move_to_side();
    }
}

// suspend zigzag auto
void ModeZigZagVert::suspend_auto()
{
    // exit immediately if we are disabled
    if (!_auto_enabled) {
        return;
    }

    if (auto_stage != AutoState::MANUAL) {
        is_suspended = true;
        return_to_manual_control_vert(true);
    }
}

// initialize zigzag auto
void ModeZigZagVert::init_auto()
{
    is_auto = false;
    auto_stage = AutoState::MANUAL;
    line_count = 0;
    is_suspended = false;
}

// spray on / off
void ModeZigZagVert::spray(bool b)
{
#if HAL_SPRAYER_ENABLED
    if (_spray_enabled) {
        copter.sprayer.run(b);
    }
#endif
}

uint32_t ModeZigZagVert::wp_distance() const
{
    if (is_auto) {
        return wp_nav->get_wp_distance_to_destination();
    } else {
        return 0;
    }
}
int32_t ModeZigZagVert::wp_bearing() const
{
    if (is_auto) {
        return wp_nav->get_wp_bearing_to_destination();
    } else {
        return 0;
    }
}
float ModeZigZagVert::crosstrack_error() const
{
    if (is_auto) {
        return wp_nav->crosstrack_error();
    } else {
        return 0;
    }
}

#endif // MODE_ZIGZAGVERT_ENABLED == ENABLED
