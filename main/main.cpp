#if 1
#define CONFIG_MICRO_ROS_APP_STACK 4000
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#include <M5Core2.h>
#include <Preferences.h> // this the EEPROM/flash interface

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <FastLED.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include "bytes.h"
#include "i2c.h"
#include "tof.h"
#include "sound.h"
#include "scale.h"

using namespace eurobin_iot;

namespace eurobin_iot
{
	uint8_t mode = 0;
	uint8_t init_mode = 0;
	bool wifi = false;

	namespace modes
	{
		enum
		{
			NONE = 0,
			KEY,
			TOF,
			HALL,
			SCALE,
			SIZE // number of modes
		};
	}
	namespace key
	{
		static const uint8_t pin = 33;
		static const uint8_t led_pin = 32;
		CRGB leds[1];
	}
	namespace hall
	{
		static const uint8_t pin = 33;
	}
}

const char *get_mode(uint8_t mode)
{
	switch (mode)
	{
	case 0:
		return "undefined";
	case 1:
		return "key";
	case 2:
		return "ToF";
	case 3:
		return "Hall";
	case 4:
		return "Scale/force";
	default:
		return "error";
	}
}

#define RCCHECK(fn)                                                                             \
	{                                                                                           \
		rcl_ret_t temp_rc = fn;                                                                 \
		if ((temp_rc != RCL_RET_OK))                                                            \
		{                                                                                       \
			printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);        \
			M5.Lcd.printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
			vTaskDelete(NULL);                                                                  \
		}                                                                                       \
	}
#define RCSOFTCHECK(fn)                                                                           \
	{                                                                                             \
		rcl_ret_t temp_rc = fn;                                                                   \
		if ((temp_rc != RCL_RET_OK))                                                              \
		{                                                                                         \
			printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);        \
			M5.Lcd.printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
		}                                                                                         \
	}

void micro_ros_task(void *arg)
{
	Speaker speaker;
	speaker.begin();
	speaker.InitI2SSpeakOrMic(MODE_SPK);

	printf("starting task...\n");
	M5.Lcd.printf("Starting ROS2 task...\n");

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

	if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK)
	{
		printf("Init: Cannot connect to uROS agent. IP=%s, port=%s\n", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
		M5.Lcd.printf("Cannot connect to uROS agent IP=%s, port=%s\n",  CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
		vTaskDelete(NULL);
	}

	// get the ID
	Preferences prefs;
	prefs.begin("eurobin_iot");
	int id = prefs.getUInt("id", 0);
	printf("My ID is: %d\n", id);

	String node_name = String("eurobin_iot_") + String(id);

	printf("ROS 2 Topic prefix: %s\n", node_name.c_str());

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, node_name.c_str(), "", &support));

	// create publishers
	/// touch button (left)
	String button_topic_name = node_name + "/button_a";
	rcl_publisher_t pub_button;
	std_msgs__msg__Int32 msg_button;
	RCCHECK(rclc_publisher_init_default(
		&pub_button,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		button_topic_name.c_str()));

	/// Time of flight
	String tof_topic_name = node_name + "/tof";
	rcl_publisher_t pub_tof;
	std_msgs__msg__Int16MultiArray msg_tof;
	int16_t data_tof[3]; // signed because no default message for unsigned...
	msg_tof.data.capacity = 3;
	msg_tof.data.size = 3;
	msg_tof.data.data = data_tof;
	if (tof::ok)
	{
		RCCHECK(rclc_publisher_init_default(
			&pub_tof,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
			tof_topic_name.c_str()));
	}

	// scale
	String scale_topic_name = node_name + "/scale";
	rcl_publisher_t pub_scale;
	std_msgs__msg__Int32 msg_scale;
	if (eurobin_iot::init_mode == eurobin_iot::modes::SCALE)
	{
		RCCHECK(rclc_publisher_init_default(
			&pub_scale,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			scale_topic_name.c_str()));
	}

	// key
	String key_topic_name = node_name + "/key";
	rcl_publisher_t pub_key;
	std_msgs__msg__Int32 msg_key;
	if (eurobin_iot::init_mode == eurobin_iot::modes::KEY)
	{
		RCCHECK(rclc_publisher_init_default(
			&pub_key,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			key_topic_name.c_str()));
	}
	// hall sensor
	String hall_topic_name = node_name + "/hall";
	rcl_publisher_t pub_hall;
	std_msgs__msg__Int32 msg_hall;
	if (eurobin_iot::init_mode == eurobin_iot::modes::HALL)
	{
		RCCHECK(rclc_publisher_init_default(
			&pub_hall,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			hall_topic_name.c_str()));
	}

	printf("Entering while loop...\n");
	M5.Lcd.setTextColor(GREEN, BLACK);
	M5.Lcd.printf("ROS2 Node ready\n");

	// for ToF
	uint16_t ambient_count, signal_count, dist;

	M5.Lcd.setTextColor(WHITE, BLACK);

	// show the ID
	M5.Lcd.fillRoundRect(320 - 100, 240 - 70, 100, 70, 15, TFT_GREEN);
	M5.Lcd.setCursor(320 - 100 + 5, 240 - 58);
	M5.Lcd.setTextSize(14);
	M5.Lcd.setTextColor(TFT_WHITE);
	M5.Lcd.printf("%d", id);

	// mode
	M5.Lcd.fillRoundRect(140, 240 - 25, 60, 25, 10, TFT_GREEN);
	M5.Lcd.setCursor(150, 240 - 20);
	M5.Lcd.setTextSize(2);
	M5.Lcd.printf("mode");

	int butt_c_activated = 0;
	int butt_mode_activated = 0;

	eurobin_iot::sound::ding(); // say we are ready!
	while (1)
	{
		M5.update();
		M5.Lcd.setTextSize(2);

		if (eurobin_iot::mode != eurobin_iot::init_mode)
		{
			M5.Lcd.setCursor(0, 240 - 20);
			M5.Lcd.printf("-> RESET\n");
		}

		M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
		M5.Lcd.setCursor(0, 90);

		M5.Lcd.printf("Mode:%s          \n", get_mode(eurobin_iot::mode));

		// button
		M5.Lcd.printf("Buttons: %d %d %d      \n", M5.BtnA.read(), M5.BtnB.read(), M5.BtnC.read());
		msg_button.data = M5.BtnA.read();
		RCSOFTCHECK(rcl_publish(&pub_button, &msg_button, NULL));
		if (msg_button.data == 1)
			eurobin_iot::sound::ding();

		// time-of-flight
		if (tof::ok)
		{
			tof::read(&ambient_count, &signal_count, &dist);
			// M5.Lcd.setCursor(0, 110);
			M5.Lcd.printf("Dist.: %d mm         \n", dist);
			// Serial.print("AC:"); Serial.print(ambient_count); Serial.print(" ");
			// Serial.print("SC:"); Serial.print(signal_count); Serial.print(" D:"); Serial.println(dist);
			msg_tof.data.data[0] = dist;
			msg_tof.data.data[1] = ambient_count;
			msg_tof.data.data[2] = signal_count;
			RCSOFTCHECK(rcl_publish(&pub_tof, &msg_tof, NULL));
		}

		// scale
		if (eurobin_iot::init_mode == eurobin_iot::modes::SCALE) {
			//scale::print();
			int w = scale::weight();
			msg_scale.data = w;
			M5.Lcd.printf("weight: %d grams     \n", w);
			RCSOFTCHECK(rcl_publish(&pub_scale, &msg_scale, NULL));
			if (scale::button()) {
				scale::tare();
				Serial.println("scale::tare");
			}
		}

		// red key
		if (eurobin_iot::init_mode == eurobin_iot::modes::KEY)
		{
			if (!digitalRead(key::pin))
			{
				key::leds[0] = CRGB::Blue;
				M5.Lcd.println(("Key: 1       "));
				FastLED.setBrightness(255);
				FastLED.show();
				msg_key.data = 1;
				eurobin_iot::sound::doorbell();
			}
			else
			{
				M5.Lcd.println(("Key: 0      "));
				key::leds[0] = CRGB::Red;
				FastLED.setBrightness(255);
				FastLED.show();
				msg_key.data = 0;
			}
			RCSOFTCHECK(rcl_publish(&pub_key, &msg_key, NULL));
		}

		// hall sensor
		if (eurobin_iot::init_mode == eurobin_iot::modes::HALL)
		{
			if (digitalRead(hall::pin))
				msg_hall.data = 0;
			else
				msg_hall.data = 1;
			M5.Lcd.printf("Hall: %d\n", msg_hall.data);
			RCSOFTCHECK(rcl_publish(&pub_hall, &msg_hall, NULL));
		}

		// mode
		if (M5.BtnB.read())
			butt_mode_activated++;
		if (butt_mode_activated > 5)
		{
			eurobin_iot::mode = (eurobin_iot::mode + 1) % eurobin_iot::modes::SIZE;
			prefs.putUInt("mode", eurobin_iot::mode);
			butt_mode_activated = 0;
		}

		if (M5.BtnC.read())
		{
			butt_c_activated++;
			M5.Lcd.printf("RESET ID [50] %d", butt_c_activated);
		}
		else
			butt_c_activated = 0;

		if (butt_c_activated >= 50)
		{
			M5.Lcd.printf(" => RESET ID");
			int r = (int)random(100);
			Serial.print("NEW ID:");
			prefs.putUInt("id", r);
			// preferences.end();
			usleep(1000);
			ESP.restart();
		}

		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&pub_button, &node));
	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);
}

extern "C"
{
	void app_main(void)
	{
		initArduino();
		// mode
		Preferences prefs;
		prefs.begin("eurobin_iot");
		eurobin_iot::mode = prefs.getUInt("mode", 0);
		eurobin_iot::init_mode = eurobin_iot::mode;

		Serial.printf("ROS_IOT -> MODE: %d %s\n", eurobin_iot::mode, get_mode(eurobin_iot::mode));

		if (eurobin_iot::mode == eurobin_iot::modes::TOF || eurobin_iot::mode == eurobin_iot::modes::SCALE)
			Wire.begin(); // join i2c bus (address optional for master)
		M5.begin();

		// LCD
		M5.Lcd.fillScreen(BLACK); // Set the screen
		M5.Lcd.setCursor(0, 0);
		M5.Lcd.setTextSize(2);
		M5.Lcd.setTextColor(BLUE);
		M5.Lcd.printf("Eurobin IOT ROS2\n");
		M5.Lcd.printf("SSID: %s\n", CONFIG_ESP_WIFI_SSID);
		M5.Lcd.setTextColor(WHITE);

		// check the time-of-flight
		if (eurobin_iot::mode == eurobin_iot::modes::TOF)
		{
			Serial.println("Initializing I2C...");
			Serial.print("Time of flight: ");
			uint8_t error = tof::check();
			if (tof::ok)
				Serial.println("ok");
			else
			{
				Serial.print("error ");
				Serial.println(error);
			}
		}

		// setup the key button
		if (eurobin_iot::mode == eurobin_iot::modes::KEY)
		{
			pinMode(key::pin, INPUT_PULLUP);
			FastLED.addLeds<SK6812, key::led_pin, GRB>(key::leds, 1);
			key::leds[0] = CRGB::Blue;
			FastLED.setBrightness(0);
		}

		// setup the hall sensor button
		if (eurobin_iot::mode == eurobin_iot::modes::HALL)
		{
			pinMode(hall::pin, INPUT);
		}

		// scale
		if (eurobin_iot::mode == eurobin_iot::modes::SCALE)
		{
			scale::init();
		}

		printf("Starting main...\n");
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
		printf("checking network interface...\n");
		esp_err_t err = uros_network_interface_initialize();
		// for now, we cannot get the error
#endif

		printf("Creating the uROS task...\n");
		// pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
		xTaskCreate(micro_ros_task,
					"uros_task",
					CONFIG_MICRO_ROS_APP_STACK,
					NULL,
					CONFIG_MICRO_ROS_APP_TASK_PRIO,
					NULL);
		printf("Task created\n");
	}
}

#endif
