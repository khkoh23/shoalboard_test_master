/*
 * 
 *
 * 
 */

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/temperature_sensor.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>

//#include <std_msgs/msg/int32.h>
//#include <std_msgs/msg/u_int16.h>
#include <shoalbot_interfaces/msg/bms.h>
#include <shoalbot_interfaces/msg/debug.h>
#include <shoalbot_interfaces/msg/di_state.h>
#include <shoalbot_interfaces/msg/do_navigation.h>
#include <shoalbot_interfaces/msg/encoder_count.h>
#include <shoalbot_interfaces/msg/imu_odom.h>
#include <shoalbot_interfaces/msg/kinco_config.h>
#include <shoalbot_interfaces/msg/position_cmd.h>
#include <shoalbot_interfaces/msg/speed_cmd.h>
//#include <shoalbot_interfaces/srv/pos_actual.h>

#include "esp32_serial_transport.h"
#include "kinco_twai.h"
#include "icm42688_register.h"
#include "amip4k_register.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define ROS_NAMESPACE CONFIG_MICRO_ROS_NAMESPACE

#define JETSON_22 GPIO_NUM_0 	// strapping pin
#define PWM_1 GPIO_NUM_1
#define PWM_2 GPIO_NUM_2
#define RS2_DE (GPIO_NUM_3)		// strapping pin
#define DO_6 GPIO_NUM_4
#define DO_7 GPIO_NUM_5
#define CAN1_TX GPIO_NUM_6
#define CAN1_RX GPIO_NUM_7
#define RS1_DE (GPIO_NUM_8)
#define SPI_CS_L GPIO_NUM_9
#define SPI_CS_IMU GPIO_NUM_10
#define SPI_MOSI GPIO_NUM_11 
#define SPI_SCLK GPIO_NUM_12
#define SPI_MISO GPIO_NUM_13 
#define DO_17 GPIO_NUM_14
#define I2C_SDA GPIO_NUM_15
#define I2C_SCL GPIO_NUM_16
#define RS1_TX (GPIO_NUM_17)
#define RS1_RX (GPIO_NUM_18)
#define RS2_TX (GPIO_NUM_19)		// usb-jtag
#define RS2_RX (GPIO_NUM_20)		// usb-jtag
#define DO_18 GPIO_NUM_21
// GPIO22-25 not available
// GPIO26-32 for SPI0/1 flash and PSRAM
// GPIO33-37 for SPIIO4~SPIIO7, SPIDQS Octal flash or Octal PSRAM
#define SPI_CS_R GPIO_NUM_35
#define DO_10 GPIO_NUM_36
#define DO_11 GPIO_NUM_37
#define DO_12 GPIO_NUM_38
#define DO_13 GPIO_NUM_39
#define DO_14 GPIO_NUM_40
#define DO_15 GPIO_NUM_41
#define DO_16 GPIO_NUM_42
// GPIO_NUM_43
// GPIO_NUM_44
// GPIO_NUM_45					// strapping pin
// GPIO_NUM_46 					// strapping pin
#define DO_19 GPIO_NUM_47
// GPIO_NUM_48

#define my_G 9.80665;
#define my_MAX 50

static const char *MASTER_GPIO_TAG = "master_gpio";
static const char *ESTOP_LEDC_TAG = "estop_ledc";
//static const char *TEMPERATURE_SENSOR_TAG = "temperature_sensor";
static const char *KINCO_TWAI_TAG = "kinco_twai";
static const char *MASTER_I2C_TAG = "master_i2c";
//static const char *AMR_STATE_TAG = "amr_state";
//static const char *MICRO_ROS_TAG = "micro_ros";
static const char *ICM42688_AMIP4K_SPI_TAG = "icm42688_amip4k_spi";
static const char *BMS_UART_TAG = "bms_uart";
static const char *NC_UART_TAG = "nc_uart";
static const char *SHOALBOARD_TEST_TAG = "shoalboard_test";


/* -------------------- global -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

rcl_publisher_t bms_publisher, distate_publisher, imuodom_publisher;
rcl_publisher_t encoder_publisher, debug_publisher;
shoalbot_interfaces__msg__Bms bms_msg;
shoalbot_interfaces__msg__Debug debug_msg;
shoalbot_interfaces__msg__DiState distate_msg;
shoalbot_interfaces__msg__ImuOdom imuodom_msg;
shoalbot_interfaces__msg__EncoderCount encoder_msg;
rcl_subscription_t donavigation_subscriber, kinco_subscriber, position_subscriber, speed_subscriber;
//rcl_subscription_t debug_subscriber;
shoalbot_interfaces__msg__DoNavigation donavigation_msg;
shoalbot_interfaces__msg__KincoConfig kinco_msg;
shoalbot_interfaces__msg__PositionCmd position_msg;
shoalbot_interfaces__msg__SpeedCmd speed_msg;
//std_msgs__msg__UInt16 debug_msg;
//rcl_service_t position_service;
//shoalbot_interfaces__srv__PosActual_Request position_req;
//shoalbot_interfaces__srv__PosActual_Response position_res;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t bms_timer, distate_timer, imuodom_timer;
rcl_timer_t encoder_timer, debug_timer;
rcl_timer_t debugdistate_timer;
rcl_init_options_t init_options;
bool micro_ros_init_successful;
const int timeout_ms = 100; // Timeout for each ping attempt
const uint8_t attempts = 1; // Number of ping attempts
const uint64_t spin_timeout = RCL_MS_TO_NS(1); // Spin period
int64_t time_offset = 0;
float orientation_q_[4];
uint8_t slave_do; // MSB to LSB: DO_9, DO_8, DO_5 to DO_0
uint8_t slave_pass1_pass2_bms; //MSB to LSB: x, x, x, x, x, PASS_2, PASS_1, BMS
uint16_t master_do; // Bit 11 to LSB: DO_19 to DO_10, DO_7, DO_6
uint32_t all_do; // Bit 19 to LSB: DO_19 to DO_0
uint32_t slave_di; //Bit 23 to LSB: DI_23 to DI_0
uint8_t master_to_slave_read_cmd[1] = {0xFF}; // command
uint8_t master_to_slave_buffer[4] = {
	0b10111011, // command
	0b00001011, // at slave: DO_9, DO_8, DO_5, DO_4, DO_3, DO_2, DO_1, DO_0
	0b00000000, // at slave: x, x, x, x, x, PASS_2, PASS_1, BMS
	0b00000000, // battery percentage 
};
uint8_t slave_to_master_buffer[6] = {
	0b00000000, // at slave: DI_23, DI_22, DI_21, DI_20, DI_19, DI_18, DI_17, DI_16
	0b00000000, // at slave: DI_15, DI_14, DI_13, DI_12, DI_11, DI_10, DI_09, DI_08
	0b00000000, // at slave: DI_7, DI_6, DI_5, DI_4, DI_3, DI_2, DI_1, DI_0
	0b00000000, // at slave: DO_9. DO_8. DO_5. DO_4. DO_3, DO_2, DO_1, DO_0
	0b00000000, // at slave: x, x, x, x, x, x, x, BOOTKEY
	0b00000000, // esp_reset_reason (SLAVE)
};
bool isResetpressed;
bool toStop; // stop signal during DirectLinear and DirectRotation srv
bool toImurest; // calibrate imu at rest
uint8_t amr_state, previous_amr_state;
bool is_new_amr_state, toDisableKinco;
bool new_master_gpio_do;
uint16_t left_kinco_error, right_kinco_error;
bool new_kinco;
int8_t kinco_mode = 3;
bool new_position;
bool new_speed;
float left_profile_speed = 636.61975, left_profile_acc = 10.61033, left_profile_dec = 10.61033;  // 0.5m/s, 0.5m/s2, 0.5m/s2
float right_profile_speed = 636.61975, right_profile_acc = 10.61033, right_profile_dec = 10.61033; 
uint16_t kinco_Kvp = 345, kinco_Kvi = 0, kinco_Kvi32 = 3; 
uint8_t kinco_SpeedFbN = 7; 
int16_t kinco_Kpp = 10, kinco_KVelocityFF = 100;
int32_t	left_kinco_pos, right_kinco_pos;
float left_kinco_speed, right_kinco_speed; 
float accel_fsr, gyro_fsr;
float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
float accel_x_calib, accel_y_calib, accel_z_calib, gyro_x_calib, gyro_y_calib, gyro_z_calib;
float temp_esp; 
const uint8_t WRA = 0b1000; // Write Address (0x8+address) 0b1000
const uint8_t WRD = 0b1010; // Write Data (0xA+data) 0b1010
const uint8_t RD0 = 0b1100; // Read bytes 0 + 1 (2LSB) (0xC+address) 0b1100
const uint8_t RD1 = 0b1110; // Read Bytes 2 + 3 (2MSB) (0xE) 0b1110
const uint8_t NOP = 0b0000; // Output read Register 
const uint8_t HWA = 0b0000;
uint32_t amip4k_statidrev, amip4k_cfg1 = 0x00FF0900, amip4k_cfg2 = 0x0000000A, amip4k_cfg3 = 0x00008000; // these are the reset value of cfgn
int32_t left_count_now, right_count_now, left_count_prev, right_count_prev;
int32_t left_counter_now, right_counter_now, left_counter_prev, right_counter_prev;
int64_t odom_time_now, odom_time_prev;
float left_speed_m, right_speed_m, left_speed_filtered, right_speed_filtered;
const uint16_t cpr = 4096;
const float wheel_diameter = 0.15, wheel_base = 0.469;
float linear_vel, angular_vel;
float heading, x_pos, y_pos;

void odom_help_euler_to_quat(float roll, float pitch, float yaw, float *q) {
	float cy = cos(yaw * 0.5);
	float sy = sin(yaw * 0.5);
	float cp = cos(pitch * 0.5);
	float sp = sin(pitch * 0.5);
	float cr = cos(roll * 0.5);
	float sr = sin(roll * 0.5);
	orientation_q_[0] = cy * cp * cr + sy * sp * sr;
	orientation_q_[1] = cy * cp * sr - sy * sp * cr;
	orientation_q_[2] = sy * cp * sr + cy * sp * cr;
	orientation_q_[3] = sy * cp * cr - cy * sp * sr;
}

int64_t get_millisecond(void) { // Get the number of seconds since boot
	return (esp_timer_get_time() / 1000ULL);
}

static void sync_time(void) { // Calculate the time difference between the microROS agent and the MCU
	int64_t now = get_millisecond();
	RCSOFTCHECK(rmw_uros_sync_session(50));
	int64_t ros_time_ms = rmw_uros_epoch_millis();
	time_offset = ros_time_ms - now; 
}

struct timespec get_timespec(void) { // Get timestamp
	struct timespec tp = {};
	int64_t now = get_millisecond() + time_offset; // deviation of synchronous time
	tp.tv_sec = now / 1000;
	tp.tv_nsec = (now % 1000) * 1000000;
	return tp;
}

typedef struct {
    float values[my_MAX];
    int k; // k stores the index of the current array read to create a circular memory through the array
    int data_points_count;
    float out;
    uint8_t i; // i is a loop counter
} moving_average_filter_t;

void moving_average_filter_begin(moving_average_filter_t* obj, const uint8_t set_data_points_count) {
    obj->k = 0; //initialize so that we start to write at index 0
    if (set_data_points_count < my_MAX) obj->data_points_count = set_data_points_count;
    else obj->data_points_count = my_MAX;
    for (obj->i = 0; obj->i < obj->data_points_count; obj->i++) obj->values[obj->i] = 0; // fill the array with 0's
}

float moving_average_filter_process(moving_average_filter_t* obj, const float in) {
    obj->out = 0;
    obj->values[obj->k] = in;
    obj->k = (obj->k + 1) % obj->data_points_count;
    for (obj->i = 0; obj->i < obj->data_points_count; obj->i++) obj->out += obj->values[obj->i]; 
    return obj->out / obj->data_points_count;
}

moving_average_filter_t odom_left_speed_filter, odom_right_speed_filter;

const int bms_uart_buffer_size = 132; // const int bms_uart_buffer_size = 127; // 
const uint8_t bms_uart_read_tout = 3; // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
const uint8_t bms_uart_buffer_bias = 73;
int16_t bms_temperature_1, bms_temperature_2, bms_temperature_3;
int16_t bms_current;
uint16_t bms_voltage, bms_mh, bms_capacity ,bms_cycle;

typedef struct {
	float temperature;
	float current;
	float voltage;
	float battery_level;
	uint16_t cycle;
} shoalbot_bms_t;

shoalbot_bms_t my_bms;
const int nc_uart_buffer_size = 127;
const uint8_t nc_uart_read_tout = 3; // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
bool isBootkeylongpressed;
esp_reset_reason_t master_reason, slave_reason;
// uint8_t crash_i; // malicious counter
//static size_t uart_port = UART_NUM_0;


/* -------------------- master gpio do -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

void master_gpio_do_task(void* arg) {
	while(1) {
		if(new_master_gpio_do) {
			new_master_gpio_do = false;
            gpio_set_level(DO_6, master_do & 0x0001);
            gpio_set_level(DO_7, master_do & 0x0002);
            gpio_set_level(DO_10, master_do & 0x0004);
            gpio_set_level(DO_11, master_do & 0x0008);
            gpio_set_level(DO_12, master_do & 0x0010);
            gpio_set_level(DO_13, master_do & 0x0020);
            gpio_set_level(DO_14, master_do & 0x0040);
            gpio_set_level(DO_15, master_do & 0x0080);
            gpio_set_level(DO_16, master_do & 0x0100);
            gpio_set_level(DO_17, master_do & 0x0200);
            gpio_set_level(DO_18, master_do & 0x0400);
            gpio_set_level(DO_19, master_do & 0x0800);
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	vTaskDelete(NULL);
}


/* -------------------- estop ledc -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

ledc_timer_config_t ledc_timer_config_1 = {
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.timer_num = LEDC_TIMER_0,
	.duty_resolution = LEDC_TIMER_13_BIT, // set duty resolution to 13 bits
	.freq_hz = 20,
	.clk_cfg = LEDC_AUTO_CLK,
};
ledc_channel_config_t ledc_channel_config_1 = {
	.gpio_num = PWM_1,
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.channel = LEDC_CHANNEL_0,
	.timer_sel = LEDC_TIMER_0,
	.duty = 819, // set duty to 10%
	.hpoint = 0,
};
ledc_timer_config_t ledc_timer_config_2 = {
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.timer_num = LEDC_TIMER_1,
	.duty_resolution = LEDC_TIMER_13_BIT, // set duty resolution to 13 bits
	.freq_hz = 20,
	.clk_cfg = LEDC_AUTO_CLK,
};
ledc_channel_config_t ledc_channel_config_2 = {
	.gpio_num = PWM_2,
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.channel = LEDC_CHANNEL_1,
	.timer_sel = LEDC_TIMER_1,
	.duty = 819, // set duty to 10%
	.hpoint = 0,
};


/* -------------------- temperature sensor -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

temperature_sensor_handle_t temperature_sensor_handle = NULL;
temperature_sensor_config_t temperature_sensor_config = {
	.range_min = -10,
	.range_max = 80,
};

void temperature_sensor_task(void *arg) { 
	while (1) {
		ESP_ERROR_CHECK(temperature_sensor_get_celsius(temperature_sensor_handle, &temp_esp));
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	vTaskDelete(NULL);
}

/* -------------------- kinco twai -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

void twai_task(void *arg) { // Kinco motor task
	while (1) {
		if (is_new_amr_state) {
			if (toDisableKinco) {
				kinco_twai_Rx1Pdo(0x20A, 0x0006); // QuS, EnV
				kinco_twai_Rx2Pdo(0x30A, 0, 0); // target speed 0 
				kinco_twai_Rx3Pdo(0x401, 0, left_profile_speed); // target position 0
				kinco_twai_Rx3Pdo(0x402, 0, right_profile_speed);
			}
			else {
				kinco_twai_Rx1Pdo(0x20A, 0x006F); // Rel, Imd, EnO, QuS, EnV, SwO
			}
			is_new_amr_state = false;
		}
		if (isResetpressed) { // check reset for releasing from ESTOP and ERROR state
				kinco_twai_Rx2Pdo(0x30A, 0, 0); // target speed 0 
				kinco_twai_Rx3Pdo(0x401, 0, left_profile_speed); // target position 0
				kinco_twai_Rx3Pdo(0x402, 0, right_profile_speed);
		}
		if (new_kinco) {
			kinco_twai_setOperationMode(0x1, kinco_mode);
			kinco_twai_setOperationMode(0x2, kinco_mode);
			// Velocity and Position loop parameters
			kinco_twai_setKvp(0x1, kinco_Kvp); 
			kinco_twai_setKvp(0x2, kinco_Kvp); 
			kinco_twai_setKvi(0x1, kinco_Kvi); 
			kinco_twai_setKvi(0x2, kinco_Kvi); 
			kinco_twai_setKvi32(0x1, kinco_Kvi32); 
			kinco_twai_setKvi32(0x2, kinco_Kvi32); 
			kinco_twai_setSpeedFbN(0x1, kinco_SpeedFbN); 
			kinco_twai_setSpeedFbN(0x2, kinco_SpeedFbN); 
			kinco_twai_setKpp(0x1, kinco_Kpp); 
			kinco_twai_setKpp(0x2, kinco_Kpp); 
			kinco_twai_setKVelocityFF(0x1, kinco_KVelocityFF); 
			kinco_twai_setKVelocityFF(0x2, kinco_KVelocityFF); 
			new_kinco = false;
		}
/*		if (new_position_callback) {
			position_res.posactual_inc = kinco_twai_getPosActual(position_req.node_id);
			//res_in->posactual_inc = kinco_twai_getPosActual(req_in->node_id);
			new_position_callback = false;
		}*/
		switch (kinco_mode) {
			case 1: // position control mode
				if (new_position) {
					kinco_twai_Rx1Pdo(0x20A, 0x006F); // Rel, Imd, EnO, QuS, EnV, SwO
					kinco_twai_Rx3Pdo(0x401, left_kinco_pos, left_profile_speed); 
					kinco_twai_Rx3Pdo(0x402, right_kinco_pos, right_profile_speed);
					kinco_twai_Rx4Pdo(0x501, left_profile_acc, left_profile_dec); 
					kinco_twai_Rx4Pdo(0x502, right_profile_acc, right_profile_dec);
					kinco_twai_Rx1Pdo(0x20A, 0x007F); // Rel, Imd, SeP, EnO, QuS, EnV, SwO
					new_position = false;
				}
				break;
			case 3:  // velocity control mode
				if (new_speed) {
					kinco_twai_Rx1Pdo(0x20A, 0x006F); // Rel, Imd, EnO, QuS, EnV, SwO
					kinco_twai_Rx2Pdo(0x30A, left_kinco_speed, right_kinco_speed); 
					new_speed = false;
				}
				break;
			default:
				break;
		}
		left_kinco_error = kinco_twai_getErrorState(1);
		right_kinco_error = kinco_twai_getErrorState(2);
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	vTaskDelete(NULL);
}


/* -------------------- master i2c -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

i2c_master_bus_config_t i2c_master_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = I2C_SCL,
    .sda_io_num = I2C_SDA,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
	.trans_queue_depth = 10,
};
i2c_master_bus_handle_t i2c_master_bus_handle;
i2c_device_config_t i2c_device_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x0A,
    .scl_speed_hz = 100000,
};
i2c_master_dev_handle_t i2c_master_dev_handle;

void i2c_master_help_set(uint8_t* data) {
	ESP_ERROR_CHECK(i2c_master_transmit(i2c_master_dev_handle, data, 4, pdMS_TO_TICKS(10)));
}

void i2c_master_help_get(uint8_t *cmd) {
	ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_master_dev_handle, cmd, sizeof(cmd), slave_to_master_buffer, 6, pdMS_TO_TICKS(10)));
	vTaskDelay(pdMS_TO_TICKS(1));
	slave_di = ((uint32_t)slave_to_master_buffer[0]) << 16 | ((uint32_t)slave_to_master_buffer[1]) << 8 | ((uint32_t)slave_to_master_buffer[2]);
	slave_do = slave_to_master_buffer[3];
	amr_state = slave_to_master_buffer[4];
	isBootkeylongpressed = (slave_to_master_buffer[4] & 0x1);
	slave_reason = slave_to_master_buffer[5]; 
	all_do = (uint32_t)(master_do & 0xFFC) << 8 | (uint32_t)(slave_do & 0xC0) << 2 | (uint32_t)(master_do & 0x3) << 6 | (uint32_t)(slave_do & 0x3F) << 0;
}

void i2c_task(void *arg) { // I2C master task
	while (1) {
		i2c_master_help_get(master_to_slave_read_cmd);
		if ((slave_di & 0x00200000) >> 21) isResetpressed = true;
 		else isResetpressed = false;
 		vTaskDelay(pdMS_TO_TICKS(50));
 		i2c_master_help_set(master_to_slave_buffer);
 		vTaskDelay(pdMS_TO_TICKS(50));
	}
	vTaskDelete(NULL);
}


/* -------------------- amr state -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

void amr_state_machine(void* arg) { 
	while(1) {
		if (previous_amr_state!=amr_state) {
			previous_amr_state = amr_state;
			is_new_amr_state = true;
		}
		switch (amr_state) {
			case 0: // SHTDWN
				break;
			case 1: // ESTOP
				toDisableKinco = true;
				break;
			case 2: // ERROR
				toDisableKinco = true;
				break;
			case 3: // LOWBAT
				toDisableKinco = false;
				break;
			case 4: // CHRGNG
				toDisableKinco = true;
				break;
			case 5: // IDLE
				toDisableKinco = false;
				break;
			case 6: // SHWBAT
				toDisableKinco = false;
				break;
			case 11: // BLOCK
				toDisableKinco = false;
				break;
			case 12: // LEFT
				toDisableKinco = false;
				break;
			case 13: // RIGHT
				toDisableKinco = false;
				break;
			case 14: // MOVE
				toDisableKinco = false;
				break;
			case 15: // FAIL
				toDisableKinco = false;
				break;
			case 16: // AUTOCHRGNG
				toDisableKinco = true;
				break;
			default:
				break;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	vTaskDelete(NULL);
}


/* -------------------- icm42688 amip4k spi -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

spi_bus_config_t spi_bus_config = {
	.mosi_io_num = SPI_MOSI,
    .miso_io_num = SPI_MISO,
    .sclk_io_num = SPI_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};
spi_device_interface_config_t spi_device_interface_config_imu = {
	.command_bits = 8,
    .address_bits = 0,
    .dummy_bits = 0,
    .clock_speed_hz = 1000000, // 1 MHz
    .duty_cycle_pos = 128,
    .mode = 3,
    .spics_io_num = SPI_CS_IMU,
    .cs_ena_posttrans = 0,
    .cs_ena_pretrans = 0, 
    .queue_size = 5,
};
spi_device_interface_config_t spi_device_interface_config_l = {
	.command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .clock_speed_hz = 1000000, // 1 MHz
    .duty_cycle_pos = 128,
    .mode = 0,
    .spics_io_num = SPI_CS_L,
	.cs_ena_pretrans = 0, 
    .cs_ena_posttrans = 1,
    .queue_size = 5,
};
spi_device_interface_config_t spi_device_interface_config_r = {
	.command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .clock_speed_hz = 1000000, // 1 MHz
    .duty_cycle_pos = 128,
    .mode = 0,
    .spics_io_num = SPI_CS_R,
	.cs_ena_pretrans = 0, 
    .cs_ena_posttrans = 1,
    .queue_size = 5,
};
spi_device_handle_t spi_device_handle_imu, spi_device_handle_l, spi_device_handle_r;

/* 7:0 WHOAMI
Reset value: 0x47
*/ 
void icm42688_spi_assert_who_am_i(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_WHO_AM_I | 0x80, .tx_buffer = NULL, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_polling_transmit(spi_device_handle_imu, &read_trans));
	assert(recvbuf == 0x47);
}

/* 7:6 Reserved ; 5 TEMP_DIS ; 4 IDLE ; 3:2 GYRO_MODE ; 1:0 ACCEL_MODE
Reset value: 0x00
*/
void icm42688_spi_pwr_mgmt0(const uint8_t msg) { //0x4E
	spi_transaction_t write_trans = {.length = 2*8, .cmd = IMU_PWR_MGMT0, .tx_buffer = &msg, .rx_buffer = 0};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &write_trans));
}

/* 7:4 ACCEL_UI_FILT_BW ; 3:0 GYRO_UI_FILT_BW
Reset value: 0x11
*/
void icm42688_spi_gyro_accel_config0(const uint8_t msg) {
	spi_transaction_t write_trans = {.length = 2*8, .cmd = IMU_GYRO_ACCEL_CONFIG0, .tx_buffer = &msg, .rx_buffer = 0};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &write_trans));
}

/* 7:5 ACCEL_FS_SEL ; 4 Reserved ; 3:0 ACCEL_ODR
Reset value: 0x06
*/
void icm42688_spi_accel_config0(const uint8_t msg) {
	spi_transaction_t write_trans = {.length = 2*8, .cmd = IMU_ACCEL_CONFIG0, .tx_buffer = &msg, .rx_buffer = 0};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &write_trans));
}

/* 7:5 Reserved ; 4:3 ACCEL_UI_FILTER_ORD ; 2:1 ACCEL_DEC2_M2_ORD ; 0 Reserved
Reset value: 0x0D
*/
void icm42688_spi_accel_config1(const uint8_t msg) {
	spi_transaction_t write_trans = {.length = 2*8, .cmd = IMU_ACCEL_CONFIG1, .tx_buffer = &msg, .rx_buffer = 0};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &write_trans));
}

/* 7:5 GYRO_FS_SEL ; 4 Reserved ; 3:0 GYRO_ODR
Reset value: 0x06
*/
void icm42688_spi_gyro_config0(const uint8_t msg) {
	spi_transaction_t write_trans = {.length = 2*8, .cmd = IMU_GYRO_CONFIG0, .tx_buffer = &msg, .rx_buffer = 0};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &write_trans));
}

/* 7:5 TEMP_FILT_BW ; 4 Reserved ; 3:2 GYRO_UI_FILTER_ORD ; 1:0 GYRO_DEC2_M2_ORD
Reset value: 0x16
*/
void icm42688_spi_gyro_config1(const uint8_t msg) {
	spi_transaction_t write_trans = {.length = 2*8, .cmd = IMU_GYRO_CONFIG1, .tx_buffer = &msg, .rx_buffer = 0};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &write_trans));
}

/* 7:2 Reserved; 1 GYRO_AAF_DIS ; 0 GYRO_NF_DIS
Reset value: 0xA0
*/
void icm42688_spi_gyro_config_static2(const uint8_t msg) {
	spi_transaction_t write_trans = {.length = 2*8, .cmd = IMU_GYRO_CONFIG_STATIC2, .tx_buffer = &msg, .rx_buffer = 0};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &write_trans));
}

/* 7:0 GYRO_X_NF_COSWZ[7:0]
Reset value 0xXX (factory trimmed)
*/
void icm42688_spi_gyro_config_static6(const uint8_t msg) {
	spi_transaction_t write_trans = {.length = 2*8, .cmd = IMU_GYRO_CONFIG_STATIC6, .tx_buffer = &msg, .rx_buffer = 0};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &write_trans));
}

/* 7:0 GYRO_Y_NF_COSWZ[7:0]
Reset value 0xXX (factory trimmed)
*/
void icm42688_spi_gyro_config_static7(const uint8_t msg) {
	spi_transaction_t write_trans = {.length = 2*8, .cmd = IMU_GYRO_CONFIG_STATIC7, .tx_buffer = &msg, .rx_buffer = 0};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &write_trans));
}

/* 7:0 GYRO_Z_NF_COSWZ[7:0]
Reset value 0xXX (factory trimmed)
*/
void icm42688_spi_gyro_config_static8(const uint8_t msg) {
	spi_transaction_t write_trans = {.length = 2*8, .cmd = IMU_GYRO_CONFIG_STATIC8, .tx_buffer = &msg, .rx_buffer = 0};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &write_trans));
}

/* 7:6 Reserved ; 5 GYRO_Z_NF_COSWZ_SEL[0] ; 4 GYRO_Y_NF_COSWZ_SEL[0] ; 3 GYRO_X_NF_COSWZ_SEL[0]
	2 GYRO_Z_NF_COSWZ[8] ; 1 GYRO_Y_NF_COSWZ[8] ; 0 GYRO_X_NF_COSWZ[8]
Reset value 0xXX (factory trimmed)
*/
void icm42688_spi_gyro_config_static9(const uint8_t msg) {
	spi_transaction_t write_trans = {.length = 2*8, .cmd = IMU_GYRO_CONFIG_STATIC9, .tx_buffer = &msg, .rx_buffer = 0};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &write_trans));
}

/* 7: Reserved ; 6:4 GYRO_NF_BW_SEL ; 3:0 Reserved
Reset value 0x11
*/
void icm42688_spi_gyro_config_static10(const uint8_t msg) {
	spi_transaction_t write_trans = {.length = 2*8, .cmd = IMU_GYRO_CONFIG_STATIC10, .tx_buffer = &msg, .rx_buffer = 0};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &write_trans));
}

/* 7:0 TEMP_DATA[7:0]
Reset value: 0x00
*/ 
uint8_t icm42688_spi_temp_data0(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_TEMP_DATA0 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	return recvbuf;
}

/* 7:0 TEMP_DATA[15:8]
Reset value: 0x80
*/ 
uint8_t icm42688_spi_temp_data1(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_TEMP_DATA1 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	return recvbuf;
}

/* 7:0 ACCEL_DATA_X[7:0]
Reset value: 0x00
*/ 
uint8_t icm42688_spi_accel_data_x0(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_ACCEL_DATA_X0 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	return recvbuf;
}

/* 7:0 ACCEL_DATA_X[15:8]
Reset value: 0x80
*/ 
uint8_t icm42688_spi_accel_data_x1(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_ACCEL_DATA_X1 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	return recvbuf;
}

/* 7:0 ACCEL_DATA_Y[7:0]
Reset value: 0x00
*/ 
uint8_t icm42688_spi_accel_data_y0(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_ACCEL_DATA_Y0 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	return recvbuf;
}

/* 7:0 ACCEL_DATA_Y[15:8]
Reset value: 0x80
*/ 
uint8_t icm42688_spi_accel_data_y1(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_ACCEL_DATA_Y1 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	return recvbuf;
}

/* 7:0 ACCEL_DATA_Z[7:0]
Reset value: 0x00
*/ 
uint8_t icm42688_spi_accel_data_z0(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_ACCEL_DATA_Z0 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	return recvbuf;
}

/* 7:0 ACCEL_DATA_Z[15:8]
Reset value: 0x80
*/ 
uint8_t icm42688_spi_accel_data_z1(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_ACCEL_DATA_Z1 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	return recvbuf;
}

/* 7:0 GYRO_DATA_X[7:0]
Reset value: 0x00
*/ 
uint8_t icm42688_spi_gyro_data_x0(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_GYRO_DATA_X0 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	return recvbuf;
}

/* 7:0 GYRO_DATA_X[15:8]
Reset value: 0x80
*/ 
uint8_t icm42688_spi_gyro_data_x1(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_GYRO_DATA_X1 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	return recvbuf;
}

/* 7:0 GYRO_DATA_Y[7:0]
Reset value: 0x00
*/ 
uint8_t icm42688_spi_gyro_data_y0(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_GYRO_DATA_Y0 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	return recvbuf;
}

/* 7:0 GYRO_DATA_Y[15:8]
Reset value: 0x80
*/ 
uint8_t icm42688_spi_gyro_data_y1(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_GYRO_DATA_Y1 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	return recvbuf;
}

/* 7:0 GYRO_DATA_Z[7:0]
Reset value: 0x00
*/ 
uint8_t icm42688_spi_gyro_data_z0(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_GYRO_DATA_Z0 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	return recvbuf;
}

/* 7:0 GYRO_DATA_Z[15:8]
Reset value: 0x80
*/ 
uint8_t icm42688_spi_gyro_data_z1(void) {
	uint8_t recvbuf;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_GYRO_DATA_Z1 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	return recvbuf;
}

/* Frequency lower bound 1000Hz, upper bound 3000Hz
*/
void icm42688_spi_help_gyro_nf_freq(const uint16_t msg) {
	int16_t NF_COSWZ = 0;
    uint8_t NF_COSWZ_SEL = 0;
    const float COSWZ = cos(2*M_PI*msg/32.0);
    if (fabs(COSWZ) <= 0.875) {
        NF_COSWZ = round(COSWZ*256);
        NF_COSWZ_SEL = 0;
    }
    else {
        NF_COSWZ_SEL = 1;
        if (COSWZ > 0.875) NF_COSWZ = round(8*(1-COSWZ)*256);
        else if (COSWZ < -0.875) NF_COSWZ = round(-8*(1+COSWZ)*256);
    }
	/*NF_COSWZ have 9-bit values to be inserted into x, y z (8-bit LSB); x, y, z (1-bit MSB)
	*/
	uint8_t buf1, buf2, buf3;
	buf1 = (NF_COSWZ & 0x00FF) >> 0;
	buf2 = (NF_COSWZ & 0xFF00) >> 8;
	icm42688_spi_gyro_config_static6(buf1);
	icm42688_spi_gyro_config_static7(buf1);
	icm42688_spi_gyro_config_static8(buf1);
	buf3 = (NF_COSWZ_SEL<<5) | (NF_COSWZ_SEL<<4) | (NF_COSWZ_SEL<<3) | (buf2<<2) | (buf2<<1) | (buf2<<0);
	icm42688_spi_gyro_config_static9(buf3);
}

/* 1st order 00 ; 2nd order 01 ; 3rd order 10 ; Reserved 11
*/
void icm42688_spi_help_gyro_ui_filter_order(const uint8_t msg) {
	uint8_t recvbuf, buf1 = 0;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_GYRO_CONFIG1 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	buf1 = (recvbuf & 0b11110011) | (msg<<2); // mask and affect only bit 3:2
	icm42688_spi_gyro_config1(buf1);
}

/* 1st order 00 ; 2nd order 01 ; 3rd order 10 ; Reserved 11
*/
void icm42688_spi_help_accel_ui_filter_order(const uint8_t msg) {
	uint8_t recvbuf, buf1 = 0;
	spi_transaction_t read_trans = {.length = 8, .cmd = IMU_ACCEL_CONFIG1 | 0x80, .tx_buffer = 0, .rx_buffer = &recvbuf};
	ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_imu, &read_trans));
	buf1 = (recvbuf & 0b11100111) | (msg<<3); // mask and affect only bit 4:3
	icm42688_spi_accel_config1(buf1);
}

float icm42688_spi_help_get_temp(void) {
	uint8_t temp_data0 = icm42688_spi_temp_data0();
	uint8_t temp_data1 = icm42688_spi_temp_data1();
	int16_t temp_data = (temp_data1 << 8) | temp_data0;
	float temp = (temp_data/132.48) + 25;
	return temp;
}

float icm42688_spi_help_get_accel_x(void) {
	uint8_t accel_data_x0 = icm42688_spi_accel_data_x0();
	uint8_t accel_data_x1 = icm42688_spi_accel_data_x1();
	int16_t accel_data_x = (accel_data_x1 << 8) | accel_data_x0;
	float accel_x = (accel_data_x*accel_fsr/32768.0) * my_G;
	return accel_x;
}

float icm42688_spi_help_get_accel_y(void) {
	uint8_t accel_data_y0 = icm42688_spi_accel_data_y0();
	uint8_t accel_data_y1 = icm42688_spi_accel_data_y1();
	int16_t accel_data_y = (accel_data_y1 << 8) | accel_data_y0;
	float accel_y = (accel_data_y*accel_fsr/32768.0) * my_G;
	return accel_y;
}

float icm42688_spi_help_get_accel_z(void) {
	uint8_t accel_data_z0 = icm42688_spi_accel_data_z0();
	uint8_t accel_data_z1 = icm42688_spi_accel_data_z1();
	int16_t accel_data_z = (accel_data_z1 << 8) | accel_data_z0;
	float accel_z = (accel_data_z*accel_fsr/32768.0) * my_G;
	return accel_z;
}

float icm42688_spi_help_get_gyro_x(void) {
	uint8_t gyro_data_x0 = icm42688_spi_gyro_data_x0();
	uint8_t gyro_data_x1 = icm42688_spi_gyro_data_x1();
	int16_t gyro_data_x = (gyro_data_x1 << 8) | gyro_data_x0;
	float gyro_x = (gyro_data_x*gyro_fsr/32768.0) * (M_PI/180.0);
	return gyro_x;
}

float icm42688_spi_help_get_gyro_y(void) {
	uint8_t gyro_data_y0 = icm42688_spi_gyro_data_y0();
	uint8_t gyro_data_y1 = icm42688_spi_gyro_data_y1();
	int16_t gyro_data_y = (gyro_data_y1 << 8) | gyro_data_y0;
	float gyro_y = (gyro_data_y*gyro_fsr/32768.0) * (M_PI/180.0);
	return gyro_y;
}

float icm42688_spi_help_get_gyro_z(void) {
	uint8_t gyro_data_z0 = icm42688_spi_gyro_data_z0();
	uint8_t gyro_data_z1 = icm42688_spi_gyro_data_z1();
	int16_t gyro_data_z = (gyro_data_z1 << 8) | gyro_data_z0;
	float gyro_z = (gyro_data_z*gyro_fsr/32768.0) * (M_PI/180.0);
	return gyro_z;
}

void icm42688_spi_help_resting_calibration(const uint16_t n) {
	for (uint16_t i = 0; i < n; i++) {
		accel_x_calib += icm42688_spi_help_get_accel_x();
		accel_y_calib += icm42688_spi_help_get_accel_y();
		accel_z_calib += icm42688_spi_help_get_accel_z();
		gyro_x_calib += icm42688_spi_help_get_gyro_x();
		gyro_y_calib += icm42688_spi_help_get_gyro_y();
		gyro_z_calib += icm42688_spi_help_get_gyro_z();
	}
	accel_x_calib /= n;
	accel_y_calib /= n;
	accel_z_calib /= n; accel_z_calib -= my_G;
	gyro_x_calib /= n;
	gyro_y_calib /= n;
	gyro_z_calib /= n; 
}

/* Read-only 
31:2 CNT/TVAL 0 ; 1 TRG 0 : 0 ERR 0
Reset value: 0x00000000 i.e. 0b 0000 0000 0000 0000 0000 0000 0000 0000
*/
int32_t amip4k_spi_get_mval(const spi_device_handle_t handle) { // SPI read 32 bit
	uint16_t sendbuf, recvbuf1, recvbuf2, recvbuf3;
	sendbuf = SPI_SWAP_DATA_TX(((RD0 << 12) | (HWA << 8) | AMIP4K_MVAL_A), 16);
	spi_transaction_t trans = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf1};
	ESP_ERROR_CHECK(spi_device_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((RD1 << 12) | (HWA << 8) | 0x00), 16);
	spi_transaction_t trans2 = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf2};
	ESP_ERROR_CHECK(spi_device_transmit(handle, &trans2));
	sendbuf = SPI_SWAP_DATA_TX(((NOP << 12) | (HWA << 8) | 0x00), 16);
	spi_transaction_t trans3 = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf3};
	ESP_ERROR_CHECK(spi_device_transmit(handle, &trans3));
	recvbuf2 = SPI_SWAP_DATA_RX(recvbuf2, 16);
	recvbuf3 = SPI_SWAP_DATA_RX(recvbuf3, 16);
	uint32_t recvbuf = recvbuf3<<16 | recvbuf2;
	return recvbuf;
}

/* 31:28 ASICID ; 27:24 ASICREV ; 23:16 - ; 
15 EKOVL ; 14 - ; 13 ZSTAT ; 12 TRGOVL ; 11 TRGZ ; 10 TRGTIM ; 9 TRGPIN ; 8 ESOFF
7 ECOFF ; 6 ESGAIN ; 5 ECGAIN ; 4 EABZ ; 3 EFAST ; 2 ESADC ; 1 ECADC ; 0 EVLOW
Reset value: 0x43000000 i.e. 0b 0100 0011 0000 0000 0000 0000 0000 0000
*/ 
unsigned int amip4k_spi_assert_stat_id_rev(const spi_device_handle_t handle) { // SPI read 32 bit
	uint16_t sendbuf, recvbuf1, recvbuf2, recvbuf3;
	sendbuf = SPI_SWAP_DATA_TX(((RD0 << 12) | (HWA << 8) | AMIP4K_STAT_A), 16);
	spi_transaction_t trans = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf1};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((RD1 << 12) | (HWA << 8) | 0x00), 16);
	spi_transaction_t trans2 = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf2};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans2));
	sendbuf = SPI_SWAP_DATA_TX(((NOP << 12) | (HWA << 8) | 0x00), 16);
	spi_transaction_t trans3 = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf3};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans3));
	recvbuf2 = SPI_SWAP_DATA_RX(recvbuf2, 16);
	recvbuf3 = SPI_SWAP_DATA_RX(recvbuf3, 16);
	uint32_t recvbuf = recvbuf3<<16 | recvbuf2;
	assert(recvbuf3 == 0x4300);
	return recvbuf;
}

/* 31 TRI 0 ; 30 LKOVL 0 ; 29 LOFF  0 ; 28 LGAIN 0 ; 27 LABZ 0 ; 26 LFAST 0 ; 25 LADC 0 ; 24 LVLOW 0 ;
23 HLD 1 ; 22 MKOVL 1 ; 21 MOFF 1 ; 20 MGAIN 1 ; 19 MABZ 1 ; 18 MFAST 1 ; 17 MADC 1 ; 16 MVLOW 1 ;
15:14 GAIN 00 ; 13:11 DH 001 ; 10:8 TPP 001 ;
7:5 MODE 000 ; 4:0 IR 00000
Reset value: 0x00FF0900 i.e. 0b 0000 0000 1111 1111 0000 1001 0000 0000
*/ 
unsigned int amip4k_spi_get_cfg1(const spi_device_handle_t handle) { // SPI read 32 bit
	uint16_t sendbuf, recvbuf1, recvbuf2, recvbuf3;
	sendbuf = SPI_SWAP_DATA_TX(((RD0 << 12) | (HWA << 8) | AMIP4K_CFG1_A), 16);
	spi_transaction_t trans = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf1};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((RD1 << 12) | (HWA << 8) | 0x00), 16);
	spi_transaction_t trans2 = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf2};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans2));
	sendbuf = SPI_SWAP_DATA_TX(((NOP << 12) | (HWA << 8) | 0x00), 16);
	spi_transaction_t trans3 = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf3};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans3));
	recvbuf2 = SPI_SWAP_DATA_RX(recvbuf2, 16);
	recvbuf3 = SPI_SWAP_DATA_RX(recvbuf3, 16);
	uint32_t recvbuf = recvbuf3<<16 | recvbuf2;
	return recvbuf;
}

/* 31 TRI 0 ; 30 LKOVL 0 ; 29 LOFF  0 ; 28 LGAIN 0 ; 27 LABZ 0 ; 26 LFAST 0 ; 25 LADC 0 ; 24 LVLOW 0 ;
23 HLD 1 ; 22 MKOVL 1 ; 21 MOFF 1 ; 20 MGAIN 1 ; 19 MABZ 1 ; 18 MFAST 1 ; 17 MADC 1 ; 16 MVLOW 1 ;
15:14 GAIN 00 ; 13:11 DH 001 ; 10:8 TPP 001 ;
7:5 MODE 000 ; 4:0 IR 00000
Reset value: 0x00FF0900 i.e. 0b 0000 0000 1111 1111 0000 1001 0000 0000
*/ 
void amip4k_spi_set_cfg1(const spi_device_handle_t handle, const uint32_t msg) { // SPI write 32 bit
	uint16_t sendbuf;
	sendbuf = SPI_SWAP_DATA_TX(((WRA << 12) | (HWA << 8) | AMIP4K_CFG1_A), 16);
	spi_transaction_t trans = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = NULL};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRD << 12) | (HWA << 8) | ((msg & 0x000000FF) >> 0)), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRA << 12) | (HWA << 8) | AMIP4K_CFG1_B), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRD << 12) | (HWA << 8) | ((msg & 0x0000FF00) >> 8)), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRA << 12) | (HWA << 8) | AMIP4K_CFG1_C), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRD << 12) | (HWA << 8) | ((msg & 0x00FF0000) >> 16)), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRA << 12) | (HWA << 8) | AMIP4K_CFG1_D), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRD << 12) | (HWA << 8) | ((msg & 0xFF000000) >> 24)), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
}

/* 31 ENA_AJ 0 ; 30:29 LP 00 ; 28 DISMON 0 ; 27 DISV0 0 ; 26 DISKSC 0 ; 25 DISK360 0 ; 24 TEAEN 0 ;
23 TRGSLP 0 ; 22 PHBER 0 ; 21:16 PH 000000 ;
15 ASYNC 0 ; 14:8 SYNC 0000000 ;
7:5 IRDIV2 000 ; 4:3 OFFSCTL 01 ; 2:1 GAINCTL 01 ; 0 DISCTL 0 ;
Reset value: 0x0000000A i.e. 0b 0000 0000 0000 0000 0000 0000 0000 1010
*/ 
unsigned int amip4k_spi_get_cfg2(const spi_device_handle_t handle) { // SPI read 32 bit
	uint16_t sendbuf, recvbuf1, recvbuf2, recvbuf3;
	sendbuf = SPI_SWAP_DATA_TX(((RD0 << 12) | (HWA << 8) | AMIP4K_CFG2_A), 16);
	spi_transaction_t trans = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf1};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((RD1 << 12) | (HWA << 8) | 0x00), 16);
	spi_transaction_t trans2 = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf2};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans2));
	sendbuf = SPI_SWAP_DATA_TX(((NOP << 12) | (HWA << 8) | 0x00), 16);
	spi_transaction_t trans3 = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf3};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans3));
	recvbuf2 = SPI_SWAP_DATA_RX(recvbuf2, 16);
	recvbuf3 = SPI_SWAP_DATA_RX(recvbuf3, 16);
	uint32_t recvbuf = recvbuf3<<16 | recvbuf2;
	return recvbuf;
}

/* 31 ENA_AJ 0 ; 30:29 LP 00 ; 28 DISMON 0 ; 27 DISV0 0 ; 26 DISKSC 0 ; 25 DISK360 0 ; 24 TEAEN 0 ;
23 TRGSLP 0 ; 22 PHBER 0 ; 21:16 PH 000000 ;
15 ASYNC 0 ; 14:8 SYNC 0000000 ;
7:5 IRDIV2 000 ; 4:3 OFFSCTL 01 ; 2:1 GAINCTL 01 ; 0 DISCTL 0 ;
Reset value: 0x0000000A i.e. 0b 0000 0000 0000 0000 0000 0000 0000 1010
*/ 
void amip4k_spi_set_cfg2(const spi_device_handle_t handle, const uint32_t msg) { // SPI write 32 bit
	uint16_t sendbuf;
	sendbuf = SPI_SWAP_DATA_TX(((WRA << 12) | (HWA << 8) | AMIP4K_CFG2_A), 16);
	spi_transaction_t trans = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = NULL};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRD << 12) | (HWA << 8) | ((msg & 0x000000FF) >> 0)), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRA << 12) | (HWA << 8) | AMIP4K_CFG2_B), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRD << 12) | (HWA << 8) | ((msg & 0x0000FF00) >> 8)), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRA << 12) | (HWA << 8) | AMIP4K_CFG2_C), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRD << 12) | (HWA << 8) | ((msg & 0x00FF0000) >> 16)), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRA << 12) | (HWA << 8) | AMIP4K_CFG2_D), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRD << 12) | (HWA << 8) | ((msg & 0xFF000000) >> 24)), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
}

/* 31 - 0 ; 30 SE_VR_int 0 ; 29 SE_halb 0 ; 28 SE_amp2 0 ; 27:26 - 00 ; 25:24 VT 00 ; 
23:16 T 0000 0000 ;
15 MXSHR 1 ; 14 PHRENA 0 ; 13 ABMX 0 ; 12 PHIOUTZ 0 ; 11:10 ZDEL2 00 ; 9 ZDEL 0 ; 8 DISZ 0 ;
7:6 ZMODE 00 ; 5 Z4 0 ; 4:0 - 00000 ;
Reset value: 0x00008000 i.e. 0b 0000 0000 0000 0000 1000 0000 0000 0000
*/ 
unsigned int amip4k_spi_get_cfg3(const spi_device_handle_t handle) { // SPI read 32 bit
	uint16_t sendbuf, recvbuf1, recvbuf2, recvbuf3;
	sendbuf = SPI_SWAP_DATA_TX(((RD0 << 12) | (HWA << 8) | AMIP4K_CFG3_A), 16);
	spi_transaction_t trans = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf1};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((RD1 << 12) | (HWA << 8) | 0x00), 16);
	spi_transaction_t trans2 = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf2};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans2));
	sendbuf = SPI_SWAP_DATA_TX(((NOP << 12) | (HWA << 8) | 0x00), 16);
	spi_transaction_t trans3 = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = &recvbuf3};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans3));
	recvbuf2 = SPI_SWAP_DATA_RX(recvbuf2, 16);
	recvbuf3 = SPI_SWAP_DATA_RX(recvbuf3, 16);
	uint32_t recvbuf = recvbuf3<<16 | recvbuf2;
	return recvbuf;
}

/* 31 - 0 ; 30 SE_VR_int 0 ; 29 SE_halb 0 ; 28 SE_amp2 0 ; 27:26 - 00 ; 25:24 VT 00 ; 
23:16 T 0000 0000 ;
15 MXSHR 1 ; 14 PHRENA 0 ; 13 ABMX 0 ; 12 PHIOUTZ 0 ; 11:10 ZDEL2 00 ; 9 ZDEL 0 ; 8 DISZ 0 ;
7:6 ZMODE 00 ; 5 Z4 0 ; 4:0 - 00000 ;
Reset value: 0x00008000 i.e. 0b 0000 0000 0000 0000 1000 0000 0000 0000
*/ 
void amip4k_spi_set_cfg3(const spi_device_handle_t handle, const uint32_t msg) { // SPI write 32 bit
	uint16_t sendbuf;
	sendbuf = SPI_SWAP_DATA_TX(((WRA << 12) | (HWA << 8) | AMIP4K_CFG3_A), 16);
	spi_transaction_t trans = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = NULL};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRD << 12) | (HWA << 8) | ((msg & 0x000000FF) >> 0)), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRA << 12) | (HWA << 8) | AMIP4K_CFG3_B), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRD << 12) | (HWA << 8) | ((msg & 0x0000FF00) >> 8)), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRA << 12) | (HWA << 8) | AMIP4K_CFG3_C), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRD << 12) | (HWA << 8) | ((msg & 0x00FF0000) >> 16)), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRA << 12) | (HWA << 8) | AMIP4K_CFG3_D), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRD << 12) | (HWA << 8) | ((msg & 0xFF000000) >> 24)), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
}

/* 7 TRGCAL 1 ; 6 - ; 5 SETHWA 1 ; 4 WCFG 1 ; 3 RESIC 1 ; 2 CLRZ 1 ; 1 RESCTL 1 ; 0 RESCNT 1 
All bits write-only
*/ 
void amip4k_spi_set_cmd(const spi_device_handle_t handle, const uint8_t msg) { // SPI write 8 bit
	uint16_t sendbuf;
	sendbuf = SPI_SWAP_DATA_TX(((WRA << 12) | (HWA << 8) | AMIP4K_CMD_A), 16);
	spi_transaction_t trans = {.length = 16, .tx_buffer = &sendbuf, .rx_buffer = NULL};
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
	sendbuf = SPI_SWAP_DATA_TX(((WRD << 12) | (HWA << 8) | msg), 16);
	trans.tx_buffer = &sendbuf;
	ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &trans));
}

void amip4k_spi_help_interpolation_rate(const spi_device_handle_t handle, const uint8_t rate){
    if (rate == Interpolation_rate_16 || rate == Interpolation_rate_8 || rate == Interpolation_rate_4) {
		amip4k_cfg1 = (amip4k_cfg1 & 0xFFFFFFE0) | Interpolation_rate_32; 
		amip4k_cfg2 = (amip4k_cfg2 & 0xFFFFFF1F) | (rate << 5);

	}
	else { // this does not require further interpolation divider
		amip4k_cfg1 = (amip4k_cfg1 & 0xFFFFFFE0) | rate; 
		amip4k_cfg2 = (amip4k_cfg2 & 0xFFFFFF1F) | 0;
	}
	amip4k_spi_set_cfg1(handle, amip4k_cfg1);
	amip4k_spi_set_cfg2(handle, amip4k_cfg2);
}

/* Return 31:2 CNT/TVAL from MVAL
*/
int32_t amip4k_spi_help_mval(const spi_device_handle_t handle) {
	int32_t measured_value = amip4k_spi_get_mval(handle) >> 2;
	return measured_value;
}

void spi_task(void *arg) { // IMU and safety encoder data task
	while (1) {

		if (toImurest) {
			gyro_x_calib = icm42688_spi_help_get_gyro_x();
			gyro_y_calib = icm42688_spi_help_get_gyro_y();
			gyro_z_calib = icm42688_spi_help_get_gyro_z();
			toImurest = false;
		}
		else {
			accel_x = icm42688_spi_help_get_accel_x() - accel_x_calib; 
			accel_y = icm42688_spi_help_get_accel_y() - accel_y_calib; 
			accel_z = icm42688_spi_help_get_accel_z() - accel_z_calib; 
			gyro_x = icm42688_spi_help_get_gyro_x() - gyro_x_calib; 
			gyro_y = icm42688_spi_help_get_gyro_y() - gyro_y_calib; 
			gyro_z = icm42688_spi_help_get_gyro_z() - gyro_z_calib; 
		}

		left_count_now = amip4k_spi_help_mval(spi_device_handle_l) * -1; // beware of the direction
		right_count_now = amip4k_spi_help_mval(spi_device_handle_r);
		if (left_count_now > left_count_prev) { // moving forward or reset after moving backward
			if (abs(left_count_now - left_count_prev) > 1024) left_counter_now = left_counter_now - (2048 - left_count_now + left_count_prev); //reset backward
			else left_counter_now = left_counter_now + (left_count_now - left_count_prev); //moving forward
		}
		else if (left_count_now < left_count_prev) { //moving backward or reset after moving forward
			if (abs(left_count_now - left_count_prev) > 1024) left_counter_now = left_counter_now + (left_count_now - left_count_prev + 2048);  //reset forward
			else left_counter_now = left_counter_now - (left_count_prev - left_count_now); //moving backward
		}
		else {} 
		left_count_prev = left_count_now;
		if (right_count_now > right_count_prev) { // moving forward or reset after moving backward
			if (abs(right_count_now - right_count_prev) > 1024) right_counter_now = right_counter_now - (2048 - right_count_now + right_count_prev); //reset backward
			else right_counter_now = right_counter_now + (right_count_now - right_count_prev); //moving forward
		}
		else if (right_count_now < right_count_prev) { //moving backward or reset after moving forward
			if (abs(right_count_now - right_count_prev) > 1024) right_counter_now = right_counter_now + (right_count_now - right_count_prev + 2048); //reset forward
			else right_counter_now = right_counter_now - (right_count_prev - right_count_now); //moving backward
		}
		else {} 
		right_count_prev = right_count_now;
		encoder_msg.left = left_counter_now;
		encoder_msg.right = right_counter_now;

		odom_time_now= esp_timer_get_time();
		float vel_dt = (odom_time_now - odom_time_prev) / 1000000.0;
		odom_time_prev = odom_time_now;
		left_speed_m = (left_counter_now - left_counter_prev) / vel_dt / cpr * M_PI * wheel_diameter; // measure m/s
		right_speed_m = (right_counter_now - right_counter_prev) / vel_dt / cpr * M_PI * wheel_diameter; // measure m/s
		if (left_counter_now==left_counter_prev) {
			left_speed_m = 0; // if (abs(left_counter_now-left_counter_prev) < 2) left_speed_m = 0;
		}
		if (right_counter_now==right_counter_prev) {
			right_speed_m = 0; // if (abs(right_counter_now-right_counter_prev) < 2) right_speed_m = 0; // 
		}
		left_counter_prev = left_counter_now;
		right_counter_prev = right_counter_now;
		left_speed_filtered = moving_average_filter_process(&odom_left_speed_filter, left_speed_m);
		right_speed_filtered = moving_average_filter_process(&odom_right_speed_filter, right_speed_m);
		float Vx = (right_speed_filtered + left_speed_filtered) / 2.0; // robot m/s
		float Vy = 0.0;
		float Wz = (right_speed_filtered - left_speed_filtered) / wheel_base; // robot rad/s
		linear_vel = Vx;
		angular_vel = Wz;
		// Odom update
		float delta_heading = Wz * vel_dt; // radians
		heading += delta_heading;
		float cos_h = cos(heading);
		float sin_h = sin(heading);
		float delta_x = (Vx * cos_h - Vy * sin_h) * vel_dt; // m
		float delta_y = (Vx * sin_h + Vy * cos_h) * vel_dt; // m
		x_pos += delta_x;
		y_pos += delta_y;
		if(heading > M_PI) heading -= 2 * M_PI;
        else if(heading <= -M_PI) heading += 2 * M_PI;
		odom_help_euler_to_quat(0, 0, heading, orientation_q_);

		vTaskDelay(pdMS_TO_TICKS(5));
	}
	vTaskDelete(NULL);
}


/* -------------------- bms uart -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

uart_config_t bms_uart_config = {
	.baud_rate = 9600,
	.data_bits = UART_DATA_8_BITS,
	.parity = UART_PARITY_DISABLE,
	.stop_bits = UART_STOP_BITS_1,
	.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	.rx_flow_ctrl_thresh = 122,
	.source_clk = UART_SCLK_DEFAULT, // .source_clk = UART_SCLK_APB,
};

static void bms_uart_help_echo_send(const uart_port_t port, const char* str, uint8_t length) {
    if (uart_write_bytes(port, str, length) != length) {
        // ESP_LOGE(BMS_UART_TAG, "Send data critical failure.");
        // abort();
    }
}

uint8_t atd(uint8_t val) { // bms_uart_help_ascii_to_dec
	if (val >= '0' && val <= '9') val = val -48;
	else if(val >= 'A' && val <= 'F') val = val -65 + 10;
	else {}
	return val;
}

/*Byte 0:6 SOI (0x7E); 7:8 SOI ; 73:76 temperature 1 ; 77:80 temperature 2 ; 81:84 temperature 3
85: 88 current ; 89:92 voltage ; 93:96 remaining charge ; 97:98 ?? ; 99: 102 capacity ; 103:106 cycle
127: EOI (0x0D)
*/
void bms_uart_task(void *arg) { 
	while (1) {
		ESP_ERROR_CHECK(uart_flush(UART_NUM_1));
		uint8_t* data = (uint8_t*) malloc(bms_uart_buffer_size); // Allocate buffers for UART
		uint8_t j = 0; // index of data
		bms_uart_help_echo_send(UART_NUM_1, "\r\n", 2);
		vTaskDelay(pdMS_TO_TICKS(10));
		int len = uart_read_bytes(UART_NUM_1, data, bms_uart_buffer_size, (200/portTICK_PERIOD_MS)); // Read data from UART
		vTaskDelay(pdMS_TO_TICKS(10));
		if (len > 0) { // Process data
			// ESP_LOGI(BMS_UART_TAG, "Received %u bytes:", len);
            // printf("[ ");
            // for (int i = 0; i < len; i++) {
            //     printf("0x%.2X ", (uint8_t)data[i]);
            // }
            //printf("] \n");
			for (int k = 0; k < len; k++) { // searching for packet header 0x7E
				if ((uint8_t)data[k]==0x7E){
					j = k;
					//printf("index: %d  \n", j);
				}
			}
			// Temperature should minus 40 to convert to degree celcius
			bms_temperature_1 = (atd(data[j+73])<<12) + (atd(data[j+74])<<8) + (atd(data[j+75])<<4) + (atd(data[j+76])<<0);
			// printf("Temperature 1: %d  ", bms_temperature_1);
			bms_temperature_2 = (atd(data[j+77])<<12) + (atd(data[j+78])<<8) + (atd(data[j+79])<<4) + (atd(data[j+80])<<0);
			// printf("Temperature 2: %d  ", bms_temperature_2);
			bms_temperature_3 = (atd(data[j+81])<<12) + (atd(data[j+82])<<8) + (atd(data[j+83])<<4) + (atd(data[j+84])<<0);
			// printf("Temperature 3: %d  ", bms_temperature_3);
			my_bms.temperature = ((bms_temperature_1 + bms_temperature_2 + bms_temperature_3) / 3.0) - 40;
			// Current unit is 10mA
			bms_current = (int16_t) ( (atd(data[j+85])<<12) + (atd(data[j+86])<<8) + (atd(data[j+87])<<4) + (atd(data[j+88])<<0) );
			// printf("Current: %d  ", bms_current);
			my_bms.current = bms_current * 0.01;
			// Voltage unit is 1mV
			bms_voltage = (atd(data[j+89])<<12) + (atd(data[j+90])<<8) + (atd(data[j+91])<<4) + (atd(data[j+92])<<0);
			// printf("Voltage: %u  ", bms_voltage);
			my_bms.voltage = bms_voltage * 0.001;
			// Percentage, remaining and overall uint is 10mAh
			bms_mh = (atd(data[j+93])<<12) + (atd(data[j+94])<<8) + (atd(data[j+95])<<4) + (atd(data[j+96])<<0);
			// printf("Mh: %u  ", bms_mh);
			bms_capacity = (atd(data[j+99])<<12) + (atd(data[j+100])<<8) + (atd(data[j+101])<<4) + (atd(data[j+102])<<0);
			// printf("Capacity: %u  ", bms_capacity);
			my_bms.battery_level = (float) bms_mh / bms_capacity * 100.0;
			// Cycle
			bms_cycle = (atd(data[j+103])<<12) + (atd(data[j+104])<<8) + (atd(data[j+105])<<4) + (atd(data[j+106])<<0);
			// printf("Cycle: %u \n", bms_cycle);
			my_bms.cycle = bms_cycle;
		}
		else {}
		master_to_slave_buffer[3] = (uint8_t) (my_bms.battery_level);
		free(data);
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
	vTaskDelete(NULL);
}


/* -------------------- nc uart -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

uart_config_t nc_uart_config = {
	.baud_rate = 9600,
	.data_bits = UART_DATA_8_BITS,
	.parity = UART_PARITY_DISABLE,
	.stop_bits = UART_STOP_BITS_1,
	.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	.rx_flow_ctrl_thresh = 122,
	.source_clk = UART_SCLK_DEFAULT, 
};

static void nc_uart_help_echo_send(const uart_port_t port, const char* str, uint8_t length) {
    if (uart_write_bytes(port, str, length) != length) {
        // ESP_LOGE(NC_UART_TAG, "Send data critical failure.");
        // abort();
    }
}

void nc_uart_task(void *arg) {  
	while (1) {
		ESP_ERROR_CHECK(uart_flush(UART_NUM_2));
		uint8_t* data = (uint8_t*) malloc(nc_uart_buffer_size); // Allocate buffers for UART
		int len = uart_read_bytes(UART_NUM_2, data, nc_uart_buffer_size, (100/portTICK_PERIOD_MS)); // Read data from UART
		if (len > 0) { // Write data back to UART
			nc_uart_help_echo_send(UART_NUM_2, "\r\n", 2);
			char prefix[] = "RS485 Received: [";
			nc_uart_help_echo_send(UART_NUM_2, prefix, (sizeof(prefix) - 1));
			ESP_LOGI(NC_UART_TAG, "Received %u bytes:", len);
			printf("[ ");
			for (int i = 0; i < len; i++) {
				printf("0x%.2X ", (uint8_t)data[i]);
				nc_uart_help_echo_send(UART_NUM_2, (const char*) &data[i], 1); 
				if (data[i] == '\r') { // Add a Newline character if get a return charater from paste
					nc_uart_help_echo_send(UART_NUM_2, "\n", 1);
				}
			}
			// printf("] \n");
			nc_uart_help_echo_send(UART_NUM_2, "]\r\n", 3);
		}
		else { // Echo a "." to show alive while waiting for input
			nc_uart_help_echo_send(UART_NUM_2, ".", 1); 
			ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, (10/portTICK_PERIOD_MS)));
		}
		free(data);
		// if (crash_i == 9) {  // check for crash
		// 	printf("Now esp is crashed\n");
		// 	assert(0);
		// }
		// else crash_i ++;
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
	vTaskDelete(NULL);
}


/* -------------------- micro ros -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

enum micro_ros_agent_state {
	WAITING_AGENT,
	AGENT_AVAILABLE,
	AGENT_CONNECTED,
	AGENT_DISCONNECTED
} uros_state;

void donavigation_callback(const void * msgin) {
	all_do = donavigation_msg.set_do;
	uint8_t navigation_intent_index = donavigation_msg.intent;
	toStop = donavigation_msg.stop;
	toImurest = donavigation_msg.imu_rest;
	master_do = ((all_do & 0xFFC00) >> 8) | ((all_do & 0xC0) >> 6); // MSB to LSB: DO_19 to DO_10, DO_7, DO_6
	slave_do = ((all_do & 0x300) >> 2) | ((all_do & 0x3F) >> 0); // MSB to LSB: DO_9, DO_8, DO_5 to DO_0
	master_to_slave_buffer[1] = slave_do;
//	master_to_slave_buffer[2] = navigation_intent_index;
	master_to_slave_buffer[2] = slave_pass1_pass2_bms;
	new_master_gpio_do = true;
}

void kinco_callback(const void * msgin) {
	kinco_mode = kinco_msg.mode;
	kinco_Kvp = kinco_msg.k_vp;
	kinco_Kvi = kinco_msg.k_vi;
	kinco_Kvi32 = kinco_msg.k_vi_32;
	kinco_SpeedFbN = kinco_msg.speed_fb_n;
	kinco_Kpp = kinco_msg.k_pp;
	kinco_KVelocityFF = kinco_msg.k_velocity_ff;
	new_kinco = true;
}

void position_callback(const void * msgin) {
	left_kinco_pos = position_msg.left_position;
	right_kinco_pos = position_msg.right_position;
	left_profile_speed = position_msg.left_speed;
	right_profile_speed = position_msg.right_speed;
	left_profile_acc = position_msg.left_acc;
	right_profile_acc = position_msg.right_acc;
	left_profile_dec = position_msg.left_dec;
	right_profile_dec = position_msg.right_dec;
	new_position = true;
}

void speed_callback(const void * msgin) {
	left_kinco_speed = speed_msg.left_speed;
	right_kinco_speed = speed_msg.right_speed;
	new_speed = true;
}

// bool new_debug;
// uint16_t debug_num;
// void debug_callback(const void * msgin) {
// 	debug_num = debug_msg.data;
// 	new_debug = true;
// }

// bool new_position_callback;
// void positionservice_callback(const void * req, void * res) {
// 	//res_in->posactual_inc = kinco_twai_getPosActual(req_in->node_id);
// 	new_position_callback = true;
// }

void imuodom_ros_init(void) {
	imuodom_msg.frame_id = micro_ros_string_utilities_set(imuodom_msg.frame_id, "imu_odom");
	imuodom_msg.child_frame_id = micro_ros_string_utilities_set(imuodom_msg.child_frame_id, "base_link");
	imuodom_msg.angular_velocity_x = 0.0;
	imuodom_msg.angular_velocity_y = 0.0;
	imuodom_msg.angular_velocity_z = 0.0;
	imuodom_msg.linear_acceleration_x = 0.0;
	imuodom_msg.linear_acceleration_y = 0.0;
	imuodom_msg.linear_acceleration_z = 0.0;
	imuodom_msg.pose_position_x = 0.0;
	imuodom_msg.pose_position_y = 0.0;
	imuodom_msg.pose_orientation_x = 0;
	imuodom_msg.pose_orientation_y = 0;
	imuodom_msg.pose_orientation_z = 0;
	imuodom_msg.pose_orientation_w = 1;			
	imuodom_msg.twist_linear_x = 0.0;
	imuodom_msg.twist_angular_z = 0.0;
}

void bms_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		bms_msg.temperature = my_bms.temperature;
		bms_msg.voltage = my_bms.voltage;
		bms_msg.current = my_bms.current;
		bms_msg.battery_level = my_bms.battery_level;
		bms_msg.charge_cycle = my_bms.cycle;
		RCSOFTCHECK(rcl_publish(&bms_publisher, &bms_msg, NULL));
	}
}

// void debug_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
// 	RCLC_UNUSED(last_call_time);
// 	if (timer != NULL) {
// 		debug_msg.esp_master_reset_reason = (uint16_t)reason;
// 		debug_msg.esp_master_temperature = temp_esp;
// 		RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
// 	}
// }

// void distate_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
// 	RCLC_UNUSED(last_call_time);
// 	if (timer != NULL) {
// 		distate_msg.get_di = slave_di;
// 		distate_msg.get_do = all_do;
// 		distate_msg.state = amr_state;
// 		distate_msg.left_kinco_error = left_kinco_error;
// 		distate_msg.right_kinco_error = right_kinco_error;
//         RCSOFTCHECK(rcl_publish(&distate_publisher, &distate_msg, NULL));
// 	}
// }

void debugdistate_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		debug_msg.esp_master_reset_reason = (uint8_t) master_reason;
		debug_msg.esp_slave_reset_reason = (uint8_t) slave_reason;
		debug_msg.esp_master_temperature = temp_esp;
		distate_msg.get_di = slave_di;
		distate_msg.get_do = all_do;
		distate_msg.state = amr_state;
		distate_msg.left_kinco_error = left_kinco_error;
		distate_msg.right_kinco_error = right_kinco_error;
		RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
		RCSOFTCHECK(rcl_publish(&distate_publisher, &distate_msg, NULL));
	}
}

void encoder_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		struct timespec time_stamp = get_timespec();
		encoder_msg.stamp.sec = time_stamp.tv_sec;
		encoder_msg.stamp.nanosec = time_stamp.tv_nsec;
		RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));
	}
}

void imuodom_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		struct timespec time_stamp = get_timespec();
		imuodom_msg.stamp.sec = time_stamp.tv_sec;
		imuodom_msg.stamp.nanosec = time_stamp.tv_nsec;
		imuodom_msg.angular_velocity_x = gyro_x; 
		imuodom_msg.angular_velocity_y = gyro_y; 
		imuodom_msg.angular_velocity_z = gyro_z; 
		imuodom_msg.linear_acceleration_x = accel_x; 
		imuodom_msg.linear_acceleration_y = accel_y; 
		imuodom_msg.linear_acceleration_z = accel_z; 
		imuodom_msg.pose_position_x = x_pos;
		imuodom_msg.pose_position_y = y_pos;
		imuodom_msg.pose_orientation_x = (double)orientation_q_[1];
		imuodom_msg.pose_orientation_y = (double)orientation_q_[2];
		imuodom_msg.pose_orientation_z = (double)orientation_q_[3];
		imuodom_msg.pose_orientation_w = (double)orientation_q_[0];
		imuodom_msg.twist_linear_x = linear_vel;
		imuodom_msg.twist_angular_z = angular_vel;
		RCSOFTCHECK(rcl_publish(&imuodom_publisher, &imuodom_msg, NULL));
	}
}

bool create_entities(void) {
	allocator = rcl_get_default_allocator();
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
	RCCHECK(rclc_node_init_default(&node, "shoalbot_master", "", &support));
	RCCHECK(rclc_publisher_init_default(&bms_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_interfaces, msg, Bms), "bms"));
	RCCHECK(rclc_publisher_init_default(&debug_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_interfaces, msg, Debug), "debug"));
	RCCHECK(rclc_publisher_init_default(&distate_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_interfaces, msg, DiState), "di_state"));
	RCCHECK(rclc_publisher_init_default(&encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_interfaces, msg, EncoderCount), "encoder"));
	RCCHECK(rclc_publisher_init_best_effort(&imuodom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_interfaces, msg, ImuOdom), "imu_odom"));
	RCCHECK(rclc_subscription_init_default(&donavigation_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_interfaces, msg, DoNavigation), "do_navigation"));
	RCCHECK(rclc_subscription_init_default(&kinco_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_interfaces, msg, KincoConfig), "kinco"));
	RCCHECK(rclc_subscription_init_default(&position_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_interfaces, msg, PositionCmd), "position"));
	RCCHECK(rclc_subscription_init_default(&speed_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_interfaces, msg, SpeedCmd), "speed"));
//	RCCHECK(rclc_subscription_init_default(&debug_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16), "debug"));
//	RCCHECK(rclc_service_init_default(&position_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(shoalbot_interfaces, srv, PosActual), "position_actual"));
	RCCHECK(rclc_timer_init_default(&bms_timer, &support, RCL_S_TO_NS(5), bms_timer_callback));
//	RCCHECK(rclc_timer_init_default(&debug_timer, &support, RCL_MS_TO_NS(200), debug_timer_callback));
//	RCCHECK(rclc_timer_init_default(&distate_timer, &support, RCL_MS_TO_NS(200), distate_timer_callback));
	RCCHECK(rclc_timer_init_default(&debugdistate_timer, &support, RCL_MS_TO_NS(200), debugdistate_timer_callback));
	RCCHECK(rclc_timer_init_default(&encoder_timer, &support, RCL_MS_TO_NS(50), encoder_timer_callback));
	RCCHECK(rclc_timer_init_default(&imuodom_timer, &support, RCL_MS_TO_NS(10), imuodom_timer_callback));
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &bms_timer));
//	RCCHECK(rclc_executor_add_timer(&executor, &debug_timer));
//	RCCHECK(rclc_executor_add_timer(&executor, &distate_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &debugdistate_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &encoder_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &imuodom_timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &donavigation_subscriber, &donavigation_msg, &donavigation_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &kinco_subscriber, &kinco_msg, &kinco_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &position_subscriber, &position_msg, &position_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &speed_subscriber, &speed_msg, &speed_callback, ON_NEW_DATA));
//	RCCHECK(rclc_executor_add_subscription(&executor, &debug_subscriber, &debug_msg, &debug_callback, ON_NEW_DATA));
//	RCCHECK(rclc_executor_add_service(&executor, &position_service, &position_req, &position_res, positionservice_callback));
	sync_time();
	return true;
}

void destroy_entities(void) {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
	RCCHECK(rcl_publisher_fini(&bms_publisher, &node));
	RCCHECK(rcl_publisher_fini(&debug_publisher, &node));
	RCCHECK(rcl_publisher_fini(&distate_publisher, &node));
	RCCHECK(rcl_publisher_fini(&encoder_publisher, &node));
	RCCHECK(rcl_publisher_fini(&imuodom_publisher, &node));
	RCCHECK(rcl_timer_fini(&bms_timer));
//	RCCHECK(rcl_timer_fini(&debug_timer));
//	RCCHECK(rcl_timer_fini(&distate_timer));
	RCCHECK(rcl_timer_fini(&debugdistate_timer));
	RCCHECK(rcl_timer_fini(&encoder_timer));
	RCCHECK(rcl_timer_fini(&imuodom_timer));
	RCCHECK(rcl_subscription_fini(&donavigation_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&kinco_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&position_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&speed_subscriber, &node));
//	RCCHECK(rcl_subscription_fini(&debug_subscriber, &node));
//	RCCHECK(rcl_service_fini(&position_service, &node));
	RCCHECK(rcl_node_fini(&node));
	RCCHECK(rclc_support_fini(&support));
}

void micro_ros_task(void * arg) {
    while(1) {
        switch (uros_state) {
            case WAITING_AGENT: // Check for agent connection
                uros_state = (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) ? AGENT_AVAILABLE : WAITING_AGENT;
                break;
            case AGENT_AVAILABLE: // Create micro-ROS entities
                uros_state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
                if (uros_state == WAITING_AGENT) { // Creation failed, release allocated resources
                    destroy_entities();
                };
                break;
            case AGENT_CONNECTED: // Check connection and spin on success
                uros_state = (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                if (uros_state == AGENT_CONNECTED) {
                    RCSOFTCHECK(rclc_executor_spin_some(&executor, spin_timeout));
                }
                break;
            case AGENT_DISCONNECTED: // Connection is lost, destroy entities and go back to first step
                destroy_entities();
                uros_state = WAITING_AGENT;
                break;
            default:
                break;
        }
		vTaskDelay(pdMS_TO_TICKS(5));
    }
	vTaskDelete(NULL);
}


/* -------------------- shoalboard test -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

uint8_t test_step;
bool isPrint;

void shoalboard_test_task(void * arg) {
    while(1) {
        switch (test_step) {
            case 0: // Check for bootkey input
				if (isPrint) {
					if (isBootkeylongpressed) {
						ESP_LOGI(SHOALBOARD_TEST_TAG, "Bootkey is pressed");
						test_step = 1;
						isPrint = false;
						break;
					}
					else {}
				}
				else {
					ESP_LOGI(SHOALBOARD_TEST_TAG, "Press BOOTKEY to begin ..........");
					isPrint = true;
				}
                break;
            case 1: 

                break;
            case 2: 

                break;
            case 3: 

                break;
            default:
                break;
        }
		vTaskDelay(pdMS_TO_TICKS(5));
    }
	vTaskDelete(NULL);
}

/* -------------------- app main -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

void app_main(void) {
	master_reason = esp_reset_reason();
	imuodom_ros_init();
	uros_state = WAITING_AGENT;
	
	ESP_LOGI(MASTER_GPIO_TAG, "Configure gpio direction");
	ESP_ERROR_CHECK(gpio_set_direction(DO_6, GPIO_MODE_OUTPUT));
	ESP_ERROR_CHECK(gpio_set_direction(DO_7, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(DO_10, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(DO_11, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(DO_12, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(DO_13, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(DO_14, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(DO_15, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(DO_16, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(DO_17, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(DO_18, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(DO_19, GPIO_MODE_OUTPUT));
	
	ESP_LOGI(ESTOP_LEDC_TAG, "Configure ledc timer");
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_config_1));
	vTaskDelay(pdMS_TO_TICKS(48)); // Essential for pulse A/B 50ms phase shift
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_config_2));
	ESP_LOGI(ESTOP_LEDC_TAG, "Configure ledc channel");
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_config_1));
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_config_2));

//	ESP_LOGI(TEMPERATURE_SENSOR_TAG, "Install temperature sensor driver");
//	ESP_ERROR_CHECK(temperature_sensor_install(&temperature_sensor_config, &temperature_sensor_handle));
//	ESP_LOGI(TEMPERATURE_SENSOR_TAG, "Enable temperature sensor");
//	ESP_ERROR_CHECK(temperature_sensor_enable(temperature_sensor_handle));

//	ESP_LOGI(KINCO_TWAI_TAG, "Install and start twai driver");
//	kinco_twai_init(CAN1_TX, CAN1_RX);

//	vTaskDelay(pdMS_TO_TICKS(1000)); // Essential wait for power supply to start

	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Initialize spi bus");
	ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_config, SPI_DMA_CH_AUTO));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Allocate imu device on spi bus");
	ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &spi_device_interface_config_imu, &spi_device_handle_imu));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Assert imu who am i");
	icm42688_spi_assert_who_am_i();
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Set imu power mode");
	icm42688_spi_pwr_mgmt0(0b00001111); // -:00, TEMP_DIS:0, IDLE:0, GYROMODE:11, ACCEL_MODE:11
	vTaskDelay(pdMS_TO_TICKS(100)); // Essential wait for imu turn on
	// ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Set imu gyro notch filter frequency and bandwidth");
	// icm42688_spi_help_gyro_nf_freq(1000);
	// ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Set imu anti alias filter");
	// ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Set imu user programmable offset");
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Select imu ui filter block: order");
	icm42688_spi_help_gyro_ui_filter_order(UI_ORDER3);
	icm42688_spi_help_accel_ui_filter_order(UI_ORDER3);
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Select imu ui filter block: bandwidth");
	icm42688_spi_gyro_accel_config0(0b01010101); // ACCEL_UI_FILTER_BW:5, GYRO_UI_FILT_BW:5
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Select imu ui path odr and fsr");
	accel_fsr = 2.0; // +/-2g
	icm42688_spi_accel_config0(0b01100111); // ACCEL_FS_SEL:011, -:0, ACCEL_ODR:0111
	gyro_fsr = 250.0; // +/-250dps
	icm42688_spi_gyro_config0(0b01100111); // GYRO_FS_SEL:011, -:0, GYRO_ODR:0111
	vTaskDelay(pdMS_TO_TICKS(5)); // Essential wait for imu config done
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Calibrate imu at rest");
	icm42688_spi_help_resting_calibration(500);
	vTaskDelay(pdMS_TO_TICKS(100)); // Essential wait for imu resting calibration

	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Allocate left sincos encoder device on spi bus");
	ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &spi_device_interface_config_l, &spi_device_handle_l));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Allocate right sincos encoder device on spi bus");
	ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &spi_device_interface_config_r, &spi_device_handle_r));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Get sincos encoder statidrev, cfg1, cfg2, cfg3");
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_assert left stat/id/rev: 0x%08X", amip4k_spi_assert_stat_id_rev(spi_device_handle_l));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_assert right stat/id/rev: 0x%08X", amip4k_spi_assert_stat_id_rev(spi_device_handle_r));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_left cfg1: 0x%08X", amip4k_spi_get_cfg1(spi_device_handle_l));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_right cfg1: 0x%08X", amip4k_spi_get_cfg1(spi_device_handle_r));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_left cfg2: 0x%08X", amip4k_spi_get_cfg2(spi_device_handle_l));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_right cfg2: 0x%08X", amip4k_spi_get_cfg2(spi_device_handle_r));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_left cfg3: 0x%08X", amip4k_spi_get_cfg3(spi_device_handle_l));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_right cfg3: 0x%08X", amip4k_spi_get_cfg3(spi_device_handle_r));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Reset and reconfigure IC left and right");
	amip4k_spi_set_cmd(spi_device_handle_l, 0x08); // RESIC 0b00001000
	amip4k_spi_set_cmd(spi_device_handle_r, 0x08); // RESIC 0b00001000
	vTaskDelay(pdMS_TO_TICKS(200)); // Essential wait for sincos encoder reset and reconfig
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Set sincos encoder interpolation rate");
	amip4k_spi_help_interpolation_rate(spi_device_handle_l, Interpolation_rate_4);
	amip4k_spi_help_interpolation_rate(spi_device_handle_r, Interpolation_rate_4);
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Set sincos encoder cfg3 to default value");
	amip4k_spi_set_cfg3(spi_device_handle_l, amip4k_cfg3);
	amip4k_spi_set_cfg3(spi_device_handle_r, amip4k_cfg3);
	vTaskDelay(pdMS_TO_TICKS(100)); // Essential wait for sincos encoder config done
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Reload count value of pre_st, reset all error flag, zstat IC left and right");
	amip4k_spi_set_cmd(spi_device_handle_l, 0x01); // RESCNT 0b00000001
	amip4k_spi_set_cmd(spi_device_handle_r, 0x01); // RESCNT 0b00000001
	vTaskDelay(pdMS_TO_TICKS(100)); // Essential wait for sincos encoder reset count done
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_left cfg1: 0x%08X", amip4k_spi_get_cfg1(spi_device_handle_l));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_right cfg1: 0x%08X", amip4k_spi_get_cfg1(spi_device_handle_r));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_left cfg2: 0x%08X", amip4k_spi_get_cfg2(spi_device_handle_l));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_right cfg2: 0x%08X", amip4k_spi_get_cfg2(spi_device_handle_r));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_left cfg3: 0x%08X", amip4k_spi_get_cfg3(spi_device_handle_l));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_right cfg3: 0x%08X", amip4k_spi_get_cfg3(spi_device_handle_r)); 

	moving_average_filter_begin(&odom_left_speed_filter, 20);
	moving_average_filter_begin(&odom_right_speed_filter, 20);

	ESP_LOGI(MASTER_I2C_TAG, "Allocate i2c master bus");
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &i2c_master_bus_handle));
	ESP_LOGI(MASTER_I2C_TAG, "Add i2c master bus device");
	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_master_bus_handle, &i2c_device_config, &i2c_master_dev_handle));
//	ESP_LOGI(MASTER_I2C_TAG, "Write i2c data");
//	i2c_master_help_set(master_to_slave_buffer); // Essential to turn on DO_0, DO_1, DO_3
//	vTaskDelay(pdMS_TO_TICKS(1000)); // Essential wait for kinco to power up

//	ESP_LOGI(KINCO_TWAI_TAG, "Initialize and configure kinco motor driver");
//	kinco_twai_setDin1Function(0x1, 0xA002); // set Din1 to halt
//	kinco_twai_setDin1Function(0x2, 0xA002); 
//	kinco_twai_setDinPolarity(0x1, 0xFFFE); // set Din1 to normally closed
//	kinco_twai_setDinPolarity(0x2, 0xFFFE); 
//	kinco_twai_setHaltMode(0x1, 2); // stop by quick stop dec
//	kinco_twai_setHaltMode(0x2, 2);
//	kinco_twai_setRx1Id(0x1, 0x20A); // Combined COB-ID: 0x20A
//	kinco_twai_setRx1Id(0x2, 0x20A); 
//	kinco_twai_setRx1Pdo1(0x1, 0x60400010); // Controlword 2-byte
//	kinco_twai_setRx1Pdo1(0x2, 0x60400010); 
//	kinco_twai_setGroupRx1Pdo(0x1, 1); // number of data: 1
//	kinco_twai_setGroupRx1Pdo(0x2, 1); 
//	kinco_twai_setRx2Id(0x1, 0x30A); // Combined COB-ID: 0x30A
//	kinco_twai_setRx2Id(0x2, 0x30A); 
//	kinco_twai_setRx2Pdo1(0x1, 0x60FF0020); // TargetSpeed 4-byte
//	kinco_twai_setRx2Pdo1(0x2, 0x60C10220); // dummy 4-byte
//	kinco_twai_setRx2Pdo2(0x1, 0x60C10220); // dummy 4-byte
//	kinco_twai_setRx2Pdo2(0x2, 0x60FF0020); // TargetSpeed 4-byte
//	kinco_twai_setGroupRx2Pdo(0x1, 2); // number of data: 2
//	kinco_twai_setGroupRx2Pdo(0x2, 2); 
//	kinco_twai_setRx3Id(0x1, 0x401); // COB-ID: 0x401
//	kinco_twai_setRx3Id(0x2, 0x402); // COB-ID: 0x402
//	kinco_twai_setRx3Pdo1(0x1, 0x607A0020); // TargetPosition 4-byte
//	kinco_twai_setRx3Pdo1(0x2, 0x607A0020); 
//	kinco_twai_setRx3Pdo2(0x1, 0x60810020); // ProfileSpeed 4-byte
//	kinco_twai_setRx3Pdo2(0x2, 0x60810020); 
//	kinco_twai_setGroupRx3Pdo(0x1, 2); // number of data: 2
//	kinco_twai_setGroupRx3Pdo(0x2, 2); 
//	kinco_twai_setRx4Id(0x1, 0x501); // COB-ID: 0x501
//	kinco_twai_setRx4Id(0x2, 0x502); // COB-ID: 0x502
//	kinco_twai_setRx4Pdo1(0x1, 0x60830020); // ProfileAcc 4-byte
//	kinco_twai_setRx4Pdo1(0x2, 0x60830020); 
//	kinco_twai_setRx4Pdo2(0x1, 0x60840020); // ProfileDec 4-byte
//	kinco_twai_setRx4Pdo2(0x2, 0x60840020); 
//	kinco_twai_setGroupRx4Pdo(0x1, 2); // number of data: 2
//	kinco_twai_setGroupRx4Pdo(0x2, 2); 
//	kinco_twai_setMaxSpeedRPM (0x1, 3000); //SMC60S-0040-30MBK-5DSU 3000 rpm
//	kinco_twai_setMaxSpeedRPM (0x2, 3000); 
//	kinco_twai_setMaxFollowingError(0x1, 10000); //655360 inc
//	kinco_twai_setMaxFollowingError(0x2, 10000); 
//	kinco_twai_setKvp(0x1, kinco_Kvp); //200 DEC
//	kinco_twai_setKvp(0x2, kinco_Kvp); 
//	kinco_twai_setKvi(0x1, kinco_Kvi); //1 DEC
//	kinco_twai_setKvi(0x2, kinco_Kvi); 
//	kinco_twai_setKvi32(0x1, kinco_Kvi32); //0 DEC
//	kinco_twai_setKvi32(0x2, kinco_Kvi32); 
//	kinco_twai_setSpeedFbN(0x1, kinco_SpeedFbN); //7 DEC
//	kinco_twai_setSpeedFbN(0x2, kinco_SpeedFbN); 
//	kinco_twai_setKpp(0x1, kinco_Kpp); //10 Hz
//	kinco_twai_setKpp(0x2, kinco_Kpp); 
//	kinco_twai_setKVelocityFF(0x1, kinco_KVelocityFF); //100%
//	kinco_twai_setKVelocityFF(0x2, kinco_KVelocityFF); 
//	kinco_twai_setOperationMode(0x1, 3); //set velocity control mode
//	kinco_twai_setOperationMode(0x2, 3);
//	kinco_twai_Rx2Pdo(0x30A, 0, 0); // target speed 0 
//	kinco_twai_Rx3Pdo(0x401, 0, left_profile_speed); // target position 0
//	kinco_twai_Rx3Pdo(0x402, 0, right_profile_speed);
//	kinco_twai_Rx4Pdo(0x501, left_profile_acc, left_profile_dec); 
//	kinco_twai_Rx4Pdo(0x502, right_profile_acc, right_profile_dec);

//	ESP_LOGI(BMS_UART_TAG,"Install bms uart driver");
//	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, bms_uart_buffer_size * 2, 0, 0, NULL, 0)); // no tx buffer, no event queue
//	ESP_LOGI(BMS_UART_TAG,"Configure bms uart parameter");
//	ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &bms_uart_config));
//	ESP_LOGI(BMS_UART_TAG,"Assign signals of bms uart peripheral to gpio pins");
//	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, RS1_TX, RS1_RX, RS1_DE, UART_PIN_NO_CHANGE));
//	ESP_LOGI(BMS_UART_TAG,"Set bms uart to rs485 half duplex mode");
//	ESP_ERROR_CHECK(uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX));
//	ESP_LOGI(BMS_UART_TAG,"Set bms uart read threshold timeout for TOUT feature");
//	ESP_ERROR_CHECK(uart_set_rx_timeout(UART_NUM_1, bms_uart_read_tout));
	// ESP_LOGI(BMS_UART_TAG, "Test bms uart receive loop");
	// bms_uart_help_echo_send(UART_NUM_1, "\r\n", 2);
//	ESP_LOGI(BMS_UART_TAG, "Discard all data in the bms uart rx buffer");
//	ESP_ERROR_CHECK(uart_flush(UART_NUM_1));
	
//	ESP_LOGI(NC_UART_TAG,"Install nc uart driver");
//	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, nc_uart_buffer_size * 2, 0, 0, NULL, 0)); // no tx buffer, no event queue
//	ESP_LOGI(NC_UART_TAG,"Configure nc uart parameter");
//	ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &nc_uart_config));
//	ESP_LOGI(NC_UART_TAG,"Assign signals of nc uart peripheral to gpio pins");
//	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, RS2_TX, RS2_RX, RS2_DE, UART_PIN_NO_CHANGE));
//	ESP_LOGI(NC_UART_TAG,"Set nc uart to rs485 half duplex mode");
//	ESP_ERROR_CHECK(uart_set_mode(UART_NUM_2, UART_MODE_RS485_HALF_DUPLEX));
//	ESP_LOGI(NC_UART_TAG,"Set nc uart read threshold timeout for TOUT feature");
//	ESP_ERROR_CHECK(uart_set_rx_timeout(UART_NUM_2, nc_uart_read_tout));
	// ESP_LOGI(NC_UART_TAG, "Test nc uart receive loop");
	// nc_uart_help_echo_send(UART_NUM_2, "Start RS485 UART test.\r\n", 24);
//	ESP_LOGI(NC_UART_TAG, "Discard all data in the nc uart rx buffer");
//	ESP_ERROR_CHECK(uart_flush(UART_NUM_2));

	ESP_LOGI(MASTER_GPIO_TAG, "Create master gpio task");
	xTaskCreate(master_gpio_do_task, "master_gpio_do_task", 4096, NULL, 5, NULL);
//	ESP_LOGI(TEMPERATURE_SENSOR_TAG, "Create temperature sensor task");
//	xTaskCreate(temperature_sensor_task, "temperature_sensor_task", 4096, NULL, 1, NULL);
	ESP_LOGI(MASTER_I2C_TAG, "Create i2c task");
	xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 5, NULL);
//	ESP_LOGI(AMR_STATE_TAG, "Create amr state task");
//	xTaskCreate(amr_state_machine, "amr_state_machine", 4096, NULL, 7, NULL);
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Create imu and encoder task");
	xTaskCreate(spi_task, "spi_task", 4096, NULL, 5, NULL);
//	ESP_LOGI(KINCO_TWAI_TAG, "Create kinco twai task");
//	xTaskCreate(twai_task, "twai_task", 4096, NULL, 5, NULL);
//	ESP_LOGI(BMS_UART_TAG, "Create bms uart task");
//	xTaskCreate(bms_uart_task, "bms_uart_task", 4096, NULL, 5, NULL);
//	ESP_LOGI(NC_UART_TAG, "Create nc uart task");
//	xTaskCreate(nc_uart_task, "nc_uart_task", 4096, NULL, 1, NULL);

//	#if defined(RMW_UXRCE_TRANSPORT_CUSTOM) 
//		rmw_uros_set_custom_transport(true, (void *) &uart_port, esp32_serial_open, esp32_serial_close, esp32_serial_write, esp32_serial_read);
//	#else
//	#error micro-ROS transports misconfigured
//	#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

	esp_intr_dump(NULL);

//	ESP_LOGI(MICRO_ROS_TAG, "Create micro ros task");
	vTaskDelay(pdMS_TO_TICKS(1000));
//	xTaskCreatePinnedToCore(micro_ros_task, "micro_ros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL, 1);

	ESP_LOGI(SHOALBOARD_TEST_TAG, "Create shoalboard test task");
	xTaskCreate(shoalboard_test_task, "shoalboard_test_task", 16000, NULL, 1, NULL);
}