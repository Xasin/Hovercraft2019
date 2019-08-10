/*
 * level1.cpp
 *
 *  Created on: 10 Aug 2019
 *      Author: xasin
 */

#include "level0.h"
#include "level1.h"

#include "telemetry.h"

#include <cmath>

namespace HVR {
namespace LVL1 {

using namespace Xasin::Drone;

quad_fact_t thrust_factors = {
	0.00501, 0.065, 1.32
};

std::array<float, 3> axes_thrust_targets = {};
std::array<float, 3> axes_thrust_scaled_targets = {};
std::array<float, 3> axes_thrust_is = {};

QuadrPropeller left_thruster = QuadrPropeller(thrust_factors, LVL0::rawMotors, 0);
QuadrPropeller right_thruster = QuadrPropeller(thrust_factors, LVL0::rawMotors, 1);


void push_motors() {
	float ctrl_scaling = 1;
	ctrl_scaling = fmin(1, left_thruster.get_scaling_factor(axes_thrust_target[0]));
	ctrl_scaling = fmin(ctrl_scaling, right_thruster.get_scaling_factor(axes_thrust_target[1]));

	for(int i=0; i<2; i++)
		axes_thrust_scaled_targets[i] = ctrl_scaling * axes_thrust_target[i];

	for(int i=0; i<3; i++) {
		float &axVal = axes_thrust_is[i];
		float axTGT  = axes_thrust_scaled_targets[i];

		if(fabs(axTGT - axVal) < 0.01)
			axTGT = axVal;
		else if(axTGT < axVal)
			axVal -= 0.001;
		else if(axTGT > axVal)
			axVal += 0.001;
	}

	left_thruster.set_newtons(axes_thrust_is[0]);
	right_thruster.set_newtons(axes_thrust_is[1]);
	LVL0::rawMotors.set_motor_power(2, axes_thrust_target[2]);

	Telemetry::update_motors();
}

}
}
