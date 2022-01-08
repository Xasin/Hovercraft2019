/*
 * level1.h
 *
 *  Created on: 10 Aug 2019
 *      Author: xasin
 */

#ifndef MAIN_HOVERCRAFT_LEVEL1_H_
#define MAIN_HOVERCRAFT_LEVEL1_H_

#include "level0.h"

#include "xasin/QuadrPropeller.h"

#include <array>

namespace HVR {
namespace LVL1 {

extern std::array<float, 3> axes_thrust_targets;
extern std::array<float, 3> axes_thrust_is;

extern Xasin::Drone::QuadrPropeller left_thruster;
extern Xasin::Drone::QuadrPropeller right_thruster;

void init();

void push_motors();

}
}


#endif /* MAIN_HOVERCRAFT_LEVEL1_H_ */
