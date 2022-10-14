#define CONFIG_MICRO_ROS_APP_STACK 4000
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#include <M5Core2.h>
#include <WiFi.h>

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
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); M5.Lcd.printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);M5.Lcd.printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;


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


	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "eurobin_iot", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"eurobin_iot/button_a"));

	printf("Publisher created\n");

	msg.data = 0;

	printf("Starting while loop...\n");
	M5.Lcd.setTextColor(GREEN, BLACK);
	M5.Lcd.printf("ROS2 Node ready\n");
	M5.Lcd.setTextColor(WHITE, BLACK);
	
	while(1){
		//rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		M5.update(); 
		M5.Lcd.setCursor(0, 90);
		M5.Lcd.printf("A: %d", M5.BtnA.read());
		printf("Publishing: %d\n", msg.data);
		msg.data = M5.BtnA.read();
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

extern "C" {
void app_main(void)
{
	M5.begin();
    M5.Lcd.fillScreen(BLACK); // Set the screen
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(2);
	M5.Lcd.setTextColor(BLUE);
    M5.Lcd.printf("Eurobin IOT ROS2\n");
	M5.Lcd.printf("SSID: %s\n", CONFIG_ESP_WIFI_SSID);
    M5.Lcd.setTextColor(WHITE);

    printf("Starting main...\n");
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    printf("checking network interface...\n");
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    printf("Creating the task...\n");
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
