/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file rtl.cpp
 *
 * Helper class to access RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <float.h>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>

#include <uORB/uORB.h>
#include <navigator/navigation.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

#include "navigator.h"
#include "rtl.h"

#define DELAY_SIGMA	0.01f

RTL::RTL(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_rtl_state(RTL_STATE_NONE),
	_param_return_alt(this, "RTL_RETURN_ALT", false),
	_param_min_loiter_alt(this, "MIS_LTRMIN_ALT", false),
	_param_descend_alt(this, "RTL_DESCEND_ALT", false),
	_param_land_delay(this, "RTL_LAND_DELAY", false),
	_param_rtl_min_dist(this, "RTL_MIN_DIST", false)
{
	/* load initial params */
	updateParams();

	// reset RTL state
	_rtl_state = RTL_STATE_NONE;
}

RTL::~RTL()
{
}

void
RTL::on_inactive()
{
	// reset RTL state
	_rtl_state = RTL_STATE_NONE;
}

void
RTL::on_activation()
{
	set_current_position_item(&_mission_item);
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	mission_apply_limitation(&_mission_item);
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->next.valid = false;

	// for safety reasons don't go into RTL if landed
	if (_navigator->get_land_detected()->landed) {
		_rtl_state = RTL_STATE_LANDED;
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Already landed, not executing RTL");

		// otherwise start RTL by braking first

	} else {
		_rtl_state = RTL_STATE_BRAKE;

	}

	set_rtl_item();
}

void
RTL::on_active()
{
	if (_rtl_state != RTL_STATE_LANDED && is_mission_item_reached()) {
		advance_rtl();
		set_rtl_item();
	}
}

void
RTL::set_rtl_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	_navigator->set_can_loiter_at_sp(false);

	// use home yaw if close to home
	float home_dist = get_distance_to_next_waypoint(_navigator->get_home_position()->lat,
			  _navigator->get_home_position()->lon,
			  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

	bool home_close = (home_dist < _param_rtl_min_dist.get());
	bool home_altitude_close = (fabsf(_navigator->get_home_position()->alt - _navigator->get_global_position()->alt) <
				    _param_rtl_min_dist.get());

	switch (_rtl_state) {
	case RTL_STATE_BRAKE: {

			/* slow down before climbing */
			set_brake_item(&_mission_item);

			break;

		}

	case RTL_STATE_CLIMB: {

			// climb to at least a 45 degree cone
			float climb_alt = _navigator->get_home_position()->alt + get_distance_to_next_waypoint(
						  _navigator->get_home_position()->lat,
						  _navigator->get_home_position()->lon,
						  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

			/* limit altitude to rtl max */
			climb_alt = math::min(_navigator->get_home_position()->alt + _param_return_alt.get(), climb_alt);

			// do also not reduce altitude if already higher
			climb_alt = math::max(climb_alt, _navigator->get_global_position()->alt);

			// and also make sure that an absolute minimum altitude is obeyed so the landing gear does not catch.
			climb_alt = math::max(climb_alt, _navigator->get_home_position()->alt + _param_min_loiter_alt.get());

			_mission_item.lat = _navigator->get_global_position()->lat;
			_mission_item.lon = _navigator->get_global_position()->lon;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = climb_alt;
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.deploy_gear = home_close && home_altitude_close;
			_mission_item.force_velocity = false;

			break;
		}

	case RTL_STATE_PRE_RETURN: {

			_mission_item.lat = _navigator->get_global_position()->lat;
			_mission_item.lon = _navigator->get_global_position()->lon;

			if (home_close) {
				_mission_item.yaw = _navigator->get_home_position()->yaw;

			} else {
				// use current heading to home
				_mission_item.yaw = get_bearing_to_next_waypoint(
							    _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
							    _navigator->get_home_position()->lat, _navigator->get_home_position()->lon);

			}

			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.deploy_gear = home_close && home_altitude_close;

			break;
		}

	case RTL_STATE_RETURN: {
			_mission_item.lat = _navigator->get_home_position()->lat;
			_mission_item.lon = _navigator->get_home_position()->lon;

			// don't change altitude

			if (home_close) {
				_mission_item.yaw = _navigator->get_home_position()->yaw;

			} else {
				// use current heading to home
				_mission_item.yaw = get_bearing_to_next_waypoint(
							    _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
							    _navigator->get_home_position()->lat, _navigator->get_home_position()->lon);

			}

			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.deploy_gear = home_close && home_altitude_close;

			break;
		}

	case RTL_STATE_AFTER_RETURN: {
			_mission_item.lat = _navigator->get_home_position()->lat;
			_mission_item.lon = _navigator->get_home_position()->lon;
			// don't change altitude
			_mission_item.yaw = _navigator->get_home_position()->yaw;

			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.deploy_gear = home_close && home_altitude_close;

			break;
		}

	case RTL_STATE_TRANSITION_TO_MC: {
			_mission_item.nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
			_mission_item.params[0] = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
			break;
		}

	case RTL_STATE_DESCEND: {
			_mission_item.lat = _navigator->get_home_position()->lat;
			_mission_item.lon = _navigator->get_home_position()->lon;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = _navigator->get_home_position()->alt + _param_descend_alt.get();

			// check if we are already lower - then we will just stay there
			if (_mission_item.altitude > _navigator->get_global_position()->alt) {
				_mission_item.altitude = _navigator->get_global_position()->alt;
			}

			// except for vtol which might be still off here and should point towards this location
			float d_current = get_distance_to_next_waypoint(
						  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
						  _mission_item.lat, _mission_item.lon);

			if (_navigator->get_vstatus()->is_vtol && d_current > _navigator->get_acceptance_radius()) {
				_mission_item.yaw = get_bearing_to_next_waypoint(
							    _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
							    _mission_item.lat, _mission_item.lon);
			}

			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.deploy_gear = home_close && home_altitude_close;

			/* disable previous setpoint to prevent drift */
			pos_sp_triplet->previous.valid = false;

			break;
		}

	case RTL_STATE_LOITER: {
			bool autoland = _param_land_delay.get() > -DELAY_SIGMA;

			_mission_item.lat = _navigator->get_home_position()->lat;
			_mission_item.lon = _navigator->get_home_position()->lon;
			// don't change altitude
			_mission_item.yaw = _navigator->get_home_position()->yaw;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = autoland ? NAV_CMD_LOITER_TIME_LIMIT : NAV_CMD_LOITER_UNLIMITED;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();

			if ((int)_navigator->get_attitude_setpoint()->landing_gear == (int)vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN) {
				_mission_item.time_inside = 0.0f;

			} else {
				_mission_item.time_inside = _param_land_delay.get() < 0.0f ? 0.0f : _param_land_delay.get();
			}

			_mission_item.autocontinue = autoland;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.deploy_gear = true;

			vehicle_command_s cmd{};
			// Set gimbal to default orientation
			cmd.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL;
			cmd.param1 = 0.0f; // pitch
			cmd.param2 = 0.0f; // roll
			cmd.param3 = 0.0f; // yaw
			cmd.param7 = 2.0f; // VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;
			cmd.timestamp = hrt_absolute_time();
			orb_advert_t pub = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
			(void)orb_unadvertise(pub);

			_navigator->set_can_loiter_at_sp(true);

			break;
		}

	case RTL_STATE_LAND: {
			set_land_item(&_mission_item, false);
			_mission_item.yaw = _navigator->get_home_position()->yaw;
			_mission_item.deploy_gear = true;
			_navigator->set_can_loiter_at_sp(false);
			break;
		}

	case RTL_STATE_LANDED: {
			set_idle_item(&_mission_item);
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	/* convert mission item to current position setpoint and make it valid */
	mission_apply_limitation(&_mission_item);
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

void
RTL::advance_rtl()
{
	switch (_rtl_state) {

	case RTL_STATE_BRAKE:
		_rtl_state = RTL_STATE_CLIMB;

		break;

	case RTL_STATE_CLIMB:
		_rtl_state = RTL_STATE_PRE_RETURN;

		break;

	case RTL_STATE_PRE_RETURN:
		_rtl_state = RTL_STATE_RETURN;

		break;

	case RTL_STATE_RETURN:
		_rtl_state = RTL_STATE_AFTER_RETURN;

		if (_navigator->get_vstatus()->is_vtol && !_navigator->get_vstatus()->is_rotary_wing) {
			_rtl_state = RTL_STATE_TRANSITION_TO_MC;
		}

		break;

	case RTL_STATE_AFTER_RETURN:
		_rtl_state = RTL_STATE_DESCEND;

		break;

	case RTL_STATE_TRANSITION_TO_MC:
		_rtl_state = RTL_STATE_RETURN;

		break;

	case RTL_STATE_DESCEND:

		/* only go to land if autoland is enabled */
		if (_param_land_delay.get() < -DELAY_SIGMA || _param_land_delay.get() > DELAY_SIGMA) {
			_rtl_state = RTL_STATE_LOITER;

		} else {
			_rtl_state = RTL_STATE_LAND;
		}

		break;

	case RTL_STATE_LOITER:
		_rtl_state = RTL_STATE_LAND;

		break;

	case RTL_STATE_LAND:
		_rtl_state = RTL_STATE_LANDED;

		break;

	default:
		break;
	}
}
