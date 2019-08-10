/*
 * level0.cpp
 *
 *  Created on: 21 Jul 2019
 *      Author: xasin
 */


#include "level0.h"
#include "../pins.h"


namespace HVR {
namespace LVL0 {

Xasin::Drone::DShot rawMotors = Xasin::Drone::DShot(DSHOT_TIMER, 3, DSHOT_START_GPIO);

void init() {
	rawMotors.init();
}

}
}
