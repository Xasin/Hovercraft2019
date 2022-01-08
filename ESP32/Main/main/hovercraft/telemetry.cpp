/*
 * telemetry.cpp
 *
 *  Created on: 9 Aug 2019
 *      Author: xasin
 */


#include "level0.h"
#include "level1.h"

#include "xasin/propertypoint/PropActuator.h"
#include "xasin/propertypoint/FloatArrayProp.h"

namespace HVR {
namespace Telemetry {

std::array<float, 3> control_inputs = {};
const char * control_input_names[] = {
		"F", "T", "Lift"
};

auto control_input_prop = Xasin::PropP::FloatArrayProp(LVL0::propHandler, "Axis", control_inputs.data(), 3, control_input_names);

auto thrust_left   = Xasin::PropP::PropActuator(LVL0::propHandler, "ML", "m_thrust", -1.385, 1.385);
auto thrust_right  = Xasin::PropP::PropActuator(LVL0::propHandler, "MR", "m_thrust", -1.385, 1.385);

auto thrust_center = Xasin::PropP::PropActuator(LVL0::propHandler, "MLift", "m_thrust", 0, 1);

void update_motors() {
	std::vector<Xasin::PropP::PropActuator*> props = {
			&thrust_left, &thrust_right, &thrust_center
	};

	for(int i=0; i<3; i++) {
		props[i]->set_value(thrust_left.IS, LVL1::axes_thrust_is[i]);
		props[i]->set_value(thrust_left.TARGET, LVL1::axes_thrust_targets[i]);
	}
}

void init() {
	control_input_prop.on_write = [](const std::vector<float> &written) {
		std::copy(written.begin(), written.end(), control_inputs.begin());
	};
}

}
}
