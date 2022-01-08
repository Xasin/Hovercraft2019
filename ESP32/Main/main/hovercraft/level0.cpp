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

using namespace Xasin;

Drone::DShot rawMotors = Drone::DShot(DSHOT_TIMER, 3, DSHOT_START_GPIO);

Xasin::PropP::PropertyHandler propHandler = Xasin::PropP::PropertyHandler();
Xasin::HTTP::Server telemetryServer = Xasin::HTTP::Server();

Xasin::HTTP::HTTPPropChannel telemetryChannel = Xasin::HTTP::HTTPPropChannel(telemetryServer, "/", propHandler);

void init() {
	rawMotors.init();

	telemetryServer.start();
}

}
}
