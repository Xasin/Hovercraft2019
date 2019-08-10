/*
 * level0.h
 *
 *  Created on: 20 Jul 2019
 *      Author: xasin
 */

#ifndef MAIN_HOVERCRAFT_LEVEL0_H_
#define MAIN_HOVERCRAFT_LEVEL0_H_

#include "xasin/BatteryManager.h"
#include "NeoController.h"

#include "SSD1306.h"

#include "xasin/DShot.h"

#include "xasin/propertypoint/PropertyHandler.h"

#include "xasin/smolhttp/Server.h"
#include "xasin/smolhttp/HTTPPropChannel.h"

namespace HVR {
namespace LVL0 {

extern Housekeeping::BatteryManager battery;

extern Xasin::Drone::DShot rawMotors;

extern Peripheral::NeoController leds;
extern Peripheral::OLED::SSD1306 screen;

extern Xasin::PropP::PropertyHandler propHandler;

extern Xasin::HTTP::Server telemetryServer;
extern Xasin::HTTP::HTTPPropChannel telemetryChannel;

void init();

}
}


#endif /* MAIN_HOVERCRAFT_LEVEL0_H_ */
