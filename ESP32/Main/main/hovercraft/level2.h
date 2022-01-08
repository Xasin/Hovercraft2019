/*
 * level2.h
 *
 *  Created on: 17 Aug 2019
 *      Author: xasin
 */

#ifndef MAIN_HOVERCRAFT_LEVEL2_H_
#define MAIN_HOVERCRAFT_LEVEL2_H_

#include <array>

// Hovercraft weight, in kg
#define CRAFT_WEIGHT 0.9
// Hovercraft rotation inertia, in kg m^2
#define CRAFT_ROT_INERTIA 0.6

// Hovercraft decellerative force, in (Ns/m)
#define CRAFT_DECELERATION 0.3
// Hovercraft rotation decelleration torque, in (Nm/s)
#define CRAFT_ROT_DECELERATION 0.4

// Propeller distance from center line, meters
#define PROPELLER_DISTANCE	(86 * 0.001)


#define FWD_P	5
#define FWD_I	0.001
#define FWD_D	0.3
#define FWD_MAX 3

#define ROT_P	3
#define ROT_I	0.001
#define ROT_D	0.01
#define ROT_MAX 5

namespace HVR {
namespace LVL2 {

extern std::array<float, 2> axis_targets;
extern std::array<float, 2> axis_is_estimates;

void run_position_calc();
void run_position_pid();

void init();

}
}



#endif /* MAIN_HOVERCRAFT_LEVEL2_H_ */
