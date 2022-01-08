/*
 * level2.cpp
 *
 *  Created on: 17 Aug 2019
 *      Author: xasin
 */

#include "level2.h"
#include "level1.h"

#include "esp_timer.h"

#include <cmath>

namespace HVR {
namespace LVL2 {

std::array<float, 2> axis_targets = std::array<float, 2>();
std::array<float, 2> axis_is_estimates = std::array<float, 2>();

float r_position_is = 0;

auto speed_vector = std::array<float, 2>();

auto axis_derivatives = std::array<float, 2>();
auto axis_integrals   = std::array<float, 2>();

int64_t last_calc_time = 0;
int64_t last_pid_time  = 0;

void run_position_calc() {
	//////////////
	/// SETUP
	//////////////
	int64_t cTime = esp_timer_get_time();
	float calc_secs = (cTime - last_calc_time)/1000000.0;
	last_calc_time = cTime;

	float rNewts = LVL1::right_thruster.get_newtons();
	float lNewts = LVL1::left_thruster.get_newtons();

	float in_phase =  cos(r_position_is);
	float out_phase = sin(r_position_is);

	//////////////
	/// ROTATION
	//////////////
	float rot_torque = (rNewts -lNewts) * PROPELLER_DISTANCE;
	rot_torque -= CRAFT_ROT_DECELERATION * axis_is_estimates[1];

	axis_derivatives[1] = rot_torque / CRAFT_ROT_INERTIA;
	axis_is_estimates[1] += axis_derivatives[1] * calc_secs;

	r_position_is += axis_is_estimates[1] * calc_secs;

	//////////////
	/// VELOCITY
	//////////////
	float fwd_aceleration = calc_secs*(lNewts + rNewts)/CRAFT_WEIGHT;

	// Increment global speed
	speed_vector[0] += in_phase * fwd_aceleration;
	speed_vector[1] += out_phase * fwd_aceleration;

	// Constant deceleration term of global speed
	float log_decelleration = (1 - calc_secs * CRAFT_DECELERATION/CRAFT_WEIGHT);
	speed_vector[0] *= log_decelleration;
	speed_vector[1] *= log_decelleration;

	// Re-transform global to local speed
	float fwd_speed_was = axis_is_estimates[0];
	axis_is_estimates[0] = speed_vector[0] * in_phase + speed_vector[1] * out_phase;
	axis_derivatives[0]  = (axis_is_estimates[0] - fwd_speed_was) / calc_secs;
}

void run_position_pid() {
	int64_t cTime = esp_timer_get_time();
	float pid_secs = (cTime - last_pid_time)/1000000.0;
	last_pid_time = cTime;

	float fwd_target = FWD_P * (axis_targets[0] - axis_is_estimates[0]);
	axis_integrals[0] += pid_secs * FWD_I * (axis_targets[0] - axis_is_estimates[0]);
	fwd_target -= FWD_D * axis_derivatives[0];
	fwd_target += axis_integrals[0];

	fwd_target = fmin(fmax(fwd_target, -FWD_MAX), FWD_MAX);

	float rot_target = ROT_P * (axis_targets[1] - axis_is_estimates[1]);
	axis_integrals[1] += pid_secs * ROT_I * (axis_targets[1] - axis_is_estimates[1]);
	rot_target -= ROT_D * axis_derivatives[1];
	rot_target += axis_integrals[1];

	rot_target /= PROPELLER_DISTANCE;
	rot_target = fmin(fmax(rot_target, -ROT_MAX), ROT_MAX);

	LVL1::axes_thrust_targets[0] = (-rot_target + fwd_target)/2;
	LVL1::axes_thrust_targets[1] = (rot_target + fwd_target)/2;
}

void init() {
	last_pid_time = esp_timer_get_time();
	last_calc_time = esp_timer_get_time();
}

}
}
