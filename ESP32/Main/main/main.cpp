
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
#include "hovercraft/level1.h"
#include "hovercraft/level2.h"
#include "hovercraft/telemetry.h"

#include "esp_log.h"

#include "xasin/mqtt/Handler.h"

#include "xasin/smolhttp/Server.h"
#include "xasin/smolhttp/Endpoint.h"
#include "xasin/smolhttp/HTTPPropChannel.h"

#include "xasin/propertypoint/PropertyHandler.h"
#include "xasin/propertypoint/PropActuator.h"

#include "xasin/socks/UDP.h"

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

void test_server_task(void *args) {
	auto test_sock = Xasin::Socks::UDP("", 456);

	vTaskDelay(10000);

	test_sock.open_local();

	char *buffer = new char[2048];
	int written_len = 0;

	ESP_LOGI("UDP", "Test port now open!");

	while(true) {
		written_len = test_sock.wait_on_receive(buffer, 2047);

		if(written_len) {
			buffer[written_len] = '\0';
			auto parsed = cJSON_Parse(buffer);

			HVR::LVL0::propHandler.feed_cJSON(parsed);
			cJSON_Delete(parsed);
		}
	}
}

void motor_reinit(int count = 2000) {
    for(int j = count; j>=0; j--) {
		for(int i=0; i<3; i++)
			HVR::LVL0::rawMotors.set_motor_power(i, 0.06 + 0.02 * sin(xTaskGetTickCount()/50.0));
		vTaskDelay(2);
    }
}
void spam_cmd(Xasin::Drone::DShot::dshot_cmd_t cmd) {
    vTaskDelay(10);
	for(int j = 300; j >= 0; j--) {
		for(int i=0; i<3; i++) {
			HVR::LVL0::rawMotors.send_cmd(i, cmd);
		}
		vTaskDelay(1);
	}
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

    Xasin::MQTT::Handler::start_wifi("TP-LINK_84CDC2\0", "f36eebda48\0", -1);

    HVR::LVL0::init();
    HVR::LVL1::init();
    HVR::LVL2::init();
	HVR::Telemetry::init();

	xTaskCreate(test_server_task, "TestUDP", 2048, nullptr, 4, nullptr);

    enum mode_t {
    	ARMING,
		NORMAL,
    } mode = ARMING;
    TickType_t init_ticks = xTaskGetTickCount() + 10000;


    motor_reinit(5000);
    spam_cmd(HVR::LVL0::rawMotors.STOP);
    spam_cmd(HVR::LVL0::rawMotors.SPIN_NORMAL);
    motor_reinit();
    spam_cmd(HVR::LVL0::rawMotors.STOP);
    spam_cmd(HVR::LVL0::rawMotors.SPIN_3D);
	motor_reinit();
	spam_cmd(HVR::LVL0::rawMotors.STOP);
	spam_cmd(HVR::LVL0::rawMotors.SAVE_SETTING);
	motor_reinit();
    spam_cmd(HVR::LVL0::rawMotors.BEACON3);
    motor_reinit();
    spam_cmd(HVR::LVL0::rawMotors.STOP);
    spam_cmd(HVR::LVL0::rawMotors.SPIN_3D);
    motor_reinit();


    while (true) {
    	vTaskDelay(3);

    	switch(mode) {
    	case ARMING:
    		for(int i=0; i<3; i++)
    		    HVR::LVL0::rawMotors.set_motor_power(i, 0.07);

    		if(init_ticks < xTaskGetTickCount()) {
    			mode = NORMAL;
    			HVR::LVL2::init();
    		}
    	break;

    	case NORMAL:
    		HVR::LVL2::axis_targets[0] = 0.99 * HVR::LVL2::axis_targets[0] + 0.01 * HVR::Telemetry::control_inputs[0]*5;
    		HVR::LVL2::axis_targets[1] = 0.99 * HVR::LVL2::axis_targets[1] + 0.01 * HVR::Telemetry::control_inputs[1]*0.2;

    		HVR::LVL2::run_position_calc();
    		HVR::LVL2::run_position_pid();

        	HVR::LVL1::push_motors();
    	break;
    	}
    }
}

