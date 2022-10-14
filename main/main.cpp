#if 1
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


	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "eurobin_iot", "", &support));

	// create publishers
	/// button
	rcl_publisher_t pub_button;
	std_msgs__msg__Int32 msg_button;
	RCCHECK(rclc_publisher_init_default(
		&pub_button,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"eurobin_iot/button_a"));

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
			"eurobin_iot/tof"));
	}	

	printf("Entering while loop...\n");
	M5.Lcd.setTextColor(GREEN, BLACK);
	M5.Lcd.printf("ROS2 Node ready\n");
	
	// for ToF
	uint16_t ambient_count, signal_count, dist;

	M5.Lcd.setTextColor(WHITE, BLACK);
	while(1){
		//rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		M5.update(); 
		// button
		M5.Lcd.setCursor(0, 90);
		M5.Lcd.printf("Button A: %d", M5.BtnA.read());
		msg_button.data = M5.BtnA.read();
		RCSOFTCHECK(rcl_publish(&pub_button, &msg_button, NULL));
		// time-of-flight
		if (tof::ok) {
			tof::read(&ambient_count, &signal_count, &dist);
			M5.Lcd.setCursor(0, 110);
			M5.Lcd.printf("Dist.: %d mm         ", dist);
			// Serial.print("AC:"); Serial.print(ambient_count); Serial.print(" ");
			// Serial.print("SC:"); Serial.print(signal_count); Serial.print(" D:"); Serial.println(dist);
			msg_tof.data.data[0] = dist;
			msg_tof.data.data[1] = ambient_count;
			msg_tof.data.data[2] = signal_count;
			RCSOFTCHECK(rcl_publish(&pub_tof, &msg_tof, NULL));
		}
		usleep(5000);// 1ms
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&pub_button, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

extern "C" {
void app_main(void)
{
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
/*
*******************************************************************************
* Copyright (c) 2021 by M5Stack
*                  Equipped with M5Core2 sample source code
*                          配套  M5Core2 示例源代码
* Visit for more information: https://docs.m5stack.com/en/unit/tof
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/unit/tof
*
* Product: ToF.  激光测距
* Date: 2021/8/16
*******************************************************************************
  Please connect to Port A,Use ToF Unit to detect distance and display distance data on the screen in real time.
  请连接端口A,使用ToF Unit检测距离，并在屏幕上实时显示距离数据。
*/

#if 0


#include <M5Core2.h>
#include "bytes.h"
#include "i2c.h"

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID 0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID 0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD 0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START 0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS 0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS 0x14
#define TOF_I2C_ADDRESS 0x29  //I2C address of ToF


using namespace eurobin_iot;


void setup() {
  // put your setup code here, to run once:
  Wire.begin();          // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output
  Serial.println("VLX53LOX test started.");

  M5.begin();
  M5.Lcd.setCursor(50, 0, 4);
  M5.Lcd.println(("VLX53LOX Example"));
}

extern "C"{
void app_main() {
    setup();

	while (true) {
		i2c::write_byte_data_at(TOF_I2C_ADDRESS, VL53L0X_REG_SYSRANGE_START, 0x01);

		byte val = 0;
		int cnt = 0;
		while (cnt < 100) {  // 1 second waiting time max
			delay(10);
			val = i2c::read_byte_data_at(TOF_I2C_ADDRESS, VL53L0X_REG_RESULT_RANGE_STATUS);
			if (val & 0x01) break;
			cnt++;
		}
		if (val & 0x01)
			Serial.println("ready");
		else
			Serial.println("not ready");

		byte* data = i2c::read_block_data_at(address, 	0x14, 12);
		uint16_t acnt = bytes::makeuint16(data[7], data[6]);
		uint16_t scnt = bytes::makeuint16(data[9], data[8]);
		uint16_t dist = bytes::makeuint16(data[11], data[10]);
		byte DeviceRangeStatusInternal = ((data[0] & 0x78) >> 3);
		M5.Lcd.fillRect(0, 35, 319, 239, BLACK);
		M5.Lcd.setCursor(0, 35, 4);
		M5.Lcd.print("ambient count: ");
		M5.Lcd.println(acnt);
		M5.Lcd.print("signal count: ");
		M5.Lcd.println(scnt);
		M5.Lcd.print("distance: ");
		M5.Lcd.println(dist);
		Serial.println(dist);
		M5.Lcd.print("status: ");
		M5.Lcd.println(DeviceRangeStatusInternal);
		delay(1000);
	}
}
}

#endif

