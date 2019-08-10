/*
 * telemetry.cpp
 *
 *  Created on: 9 Aug 2019
 *      Author: xasin
 */


#include "level0.h"
#include "level1.h"

#include "xasin/propertypoint/PropActuator.h"

namespace HVR {
namespace Telemetry {

auto thrust_left   = Xasin::PropP::PropActuator(LVL0::propHandler, "ML", "m_thrust", -1.2, 1.2);
auto thrust_right  = Xasin::PropP::PropActuator(LVL0::propHandler, "MR", "m_thrust", -1.2, 1.2);

auto thrust_center = Xasin::PropP::PropActuator(LVL0::propHandler, "MLift", "m_thrust", 0, 1);

void update_motors() {
	std::vector<Xasin::PropP::PropActuator*> props = {
			&thrust_left, &thrust_right, &thrust_center
	};

	for(int i=0; i<3; i++) {
		props[i]->set_value(thrust_left.IS, LVL1::axes_thrust_is[i]);
		props[i]->set_value(thrust_left.TARGET, LVL1::axes_thrust_target[i]);
	}
}

void init() {

}

}
}
