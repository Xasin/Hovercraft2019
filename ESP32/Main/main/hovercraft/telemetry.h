/*
 * telemetry.h
 *
 *  Created on: 10 Aug 2019
 *      Author: xasin
 */

#ifndef MAIN_HOVERCRAFT_TELEMETRY_H_
#define MAIN_HOVERCRAFT_TELEMETRY_H_

#include <array>

namespace HVR {
namespace Telemetry {

extern std::array<float, 3> control_inputs;

void update_motors();
void init();

}
}



#endif /* MAIN_HOVERCRAFT_TELEMETRY_H_ */
