#if 1
#define CONFIG_MICRO_ROS_APP_STACK 4000
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#include <M5Core2.h>
#include <WiFi.h>
#include <Preferences.h> // this the EEPROM/flash interface

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


#include "bytes.h"
#include "i2c.h"
#include "tof.h"

using namespace eurobin_iot;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); M5.Lcd.printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);M5.Lcd.printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


void micro_ros_task(void * arg)
{
	
	printf("starting task...");
	M5.Lcd.printf("Starting ROS2 task...\n");

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));


	// get the ID
	Preferences prefs;
	prefs.begin("eurobin_iot");
	int id =  prefs.getUInt("id", 0);
	Serial.printf("My ID is: %d\n", id); 

	String node_name = String("eurobin_iot_") + String(id);
	String button_topic_name = node_name + "/button_a";
	String tof_topic_name = node_name + "/tof";
		
	Serial.println(button_topic_name.c_str());
	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, node_name.c_str(), "", &support));

	// create publishers
	/// button
	rcl_publisher_t pub_button;
	std_msgs__msg__Int32 msg_button;
	RCCHECK(rclc_publisher_init_default(
		&pub_button,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		button_topic_name.c_str()));

	/// Time of flight
	rcl_publisher_t pub_tof;
	std_msgs__msg__Int16MultiArray msg_tof;
	int16_t data_tof[3];// signed because no default message for unsigned...
	msg_tof.data.capacity = 3;
	msg_tof.data.size = 3;
	msg_tof.data.data = data_tof;
	if (tof::ok) {
		RCCHECK(rclc_publisher_init_default(
			&pub_tof,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
			tof_topic_name.c_str()));
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

	int butt_c_activated = 0;
	while(1){
		//rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		M5.update();
		M5.Lcd.setTextSize(2);
		M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
		// button
		M5.Lcd.setCursor(0, 90);
		M5.Lcd.printf("Buttons: %d %d %d", M5.BtnA.read(), M5.BtnB.read(), M5.BtnC.read());
		msg_button.data = M5.BtnA.read();
		RCSOFTCHECK(rcl_publish(&pub_button, &msg_button, NULL));
		// time-of-flight
		if (tof::ok) {
			tof::read(&ambient_count, &signal_count, &dist);
			M5.Lcd.setCursor(0, 110);
			M5.Lcd.printf("Dist.: %d mm         \n", dist);
			// Serial.print("AC:"); Serial.print(ambient_count); Serial.print(" ");
			// Serial.print("SC:"); Serial.print(signal_count); Serial.print(" D:"); Serial.println(dist);
			msg_tof.data.data[0] = dist;
			msg_tof.data.data[1] = ambient_count;
			msg_tof.data.data[2] = signal_count;
			RCSOFTCHECK(rcl_publish(&pub_tof, &msg_tof, NULL));
		}
		if (M5.BtnC.read()){ 
			butt_c_activated ++;
			M5.Lcd.printf("RESET ID [50] %d", butt_c_activated);
		}
		else 
			butt_c_activated = 0;

		if (butt_c_activated >= 50) {
			M5.Lcd.printf(" => RESET ID");
			int r = (int) random(100);
			Serial.print("NEW ID:");
			prefs.putUInt("id", r);
			//preferences.end();
			usleep(1000);
			ESP.restart();
		}

		// reset the ID
		usleep(5000);// 5ms
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&pub_button, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

extern "C" {
void app_main(void)
{
	initArduino();
	Wire.begin();          // join i2c bus (address optional for master)
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
	Serial.println("Initializing I2C...");
	Serial.print("Time of flight: ");
	uint8_t error = tof::check();
	if (tof::ok)
		Serial.println("ok");
	else {
		Serial.print("error ");
		Serial.println(error);
	}


    printf("Starting main...\n");
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    printf("checking network interface...\n");
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    printf("Creating the uROS task...\n");
    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
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

