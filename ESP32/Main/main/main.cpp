
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "esp32/pm.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"

#include "hovercraft/level0.h"

#include "esp_log.h"

#include "xasin/mqtt/Handler.h"

#include "xasin/smolhttp/Server.h"
#include "xasin/smolhttp/Endpoint.h"
#include "xasin/smolhttp/HTTPPropChannel.h"

#include "xasin/propertypoint/PropertyHandler.h"
#include "xasin/propertypoint/PropActuator.h"

using namespace Xasin;

esp_err_t event_handler(void *ctx, system_event_t *event)
{
	Xasin::MQTT::Handler::try_wifi_reconnect(event);
	return ESP_OK;
}

#include "xasin/QuadrPropeller.h"

Xasin::Drone::quad_fact_t factors = {
	0.00501, 0.065, 1.32
};
auto testProp = Xasin::Drone::QuadrPropeller(factors, HVR::LVL0::rawMotors, 0);

float oldMotor = 0;
void smooth_set(float newMotor) {
	for(float j=0; j<1; j+=0.01) {
		vTaskDelay(4);

		auto n_value = (oldMotor*(1-j) + newMotor*j);
		testProp.set_newtons(n_value);
	}

	oldMotor = newMotor;
}

extern "C" void app_main(void)
{
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );

    esp_timer_init();

    esp_pm_config_esp32_t power_config = {};
    power_config.max_freq_mhz = 240;
	power_config.min_freq_mhz = 240;
	power_config.light_sleep_enable = false;
    esp_pm_configure(&power_config);

    Xasin::MQTT::Handler::start_wifi("TP-LINK_84CDC2\0", "f36eebda48\0");

    HVR::LVL0::init();

//    motor_prop.on_motor_set = [&targetI](int id, float val) {
//    	if(id == 0)
//    		targetI = val;
//
//    	printf("New motor power: %f\n", targetI);
//    };

    vTaskDelay(100);
    smooth_set(0.03);
    smooth_set(0);

    float i=0;

    while (true) {
    	vTaskDelay(4);

    	i = (1-0.002)*i + 0.002*sin(xTaskGetTickCount()/3000.0);

		testProp.set_newtons(i);
    }
}

