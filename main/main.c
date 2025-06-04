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
#include "driver/twai.h"

#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"

#include "icm42688_register.h"
#include "amip4k_register.h"

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

static const char *MASTER_GPIO_TAG = "master_gpio";
static const char *ESTOP_LEDC_TAG = "estop_ledc";
static const char *KINCO_TWAI_TAG = "kinco_twai";
static const char *MASTER_I2C_TAG = "master_i2c";
static const char *ICM42688_AMIP4K_SPI_TAG = "icm42688_amip4k_spi";
static const char *BMS_UART_TAG = "bms_uart";
static const char *NC_UART_TAG = "nc_uart";
static const char *SHOALBOARD_TEST_TAG = "shoalboard_test";


/* -------------------- global -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

uint8_t slave_do; // MSB to LSB: DO_9, DO_8, DO_5 to DO_0
uint8_t slave_pass1_pass2_bms; //MSB to LSB: x, x, x, x, x, PASS_2, PASS_1, BMS
uint16_t master_do; // Bit 11 to LSB: DO_19 to DO_10, DO_7, DO_6
uint32_t all_do; // Bit 19 to LSB: DO_19 to DO_0
uint32_t slave_di; //Bit 23 to LSB: DI_23 to DI_0
uint8_t master_to_slave_read_cmd[1] = {0xFF}; // command
uint8_t master_to_slave_buffer[4] = {
	0b10111011, // command
	0b00000000, // at slave: DO_9, DO_8, DO_5, DO_4, DO_3, DO_2, DO_1, DO_0
	0b00000000, // at slave: x, x, x, x, x, PASS_2, PASS_1, BMS
	0b00000000, // battery percentage 
};
uint8_t slave_to_master_buffer[6] = {
	0b00000000, // at slave: DI_23, DI_22, DI_21, DI_20, DI_19, DI_18, DI_17, DI_16
	0b00000000, // at slave: DI_15, DI_14, DI_13, DI_12, DI_11, DI_10, DI_09, DI_08
	0b00000000, // at slave: DI_7, DI_6, DI_5, DI_4, DI_3, DI_2, DI_1, DI_0
	0b00000000, // at slave: DO_9. DO_8. DO_5. DO_4. DO_3, DO_2, DO_1, DO_0
	0b00000000, // at slave: x, x, x, x, x, x, x, BOOTKEY
	0b00000000, // interval count
};
float accel_fsr, gyro_fsr;
float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
float accel_x_calib, accel_y_calib, accel_z_calib, gyro_x_calib, gyro_y_calib, gyro_z_calib;
const uint8_t WRA = 0b1000; // Write Address (0x8+address) 0b1000
const uint8_t WRD = 0b1010; // Write Data (0xA+data) 0b1010
const uint8_t RD0 = 0b1100; // Read bytes 0 + 1 (2LSB) (0xC+address) 0b1100
const uint8_t RD1 = 0b1110; // Read Bytes 2 + 3 (2MSB) (0xE) 0b1110
const uint8_t NOP = 0b0000; // Output read Register 
const uint8_t HWA = 0b0000;
uint32_t amip4k_cfg1 = 0x00FF0900, amip4k_cfg2 = 0x0000000A, amip4k_cfg3 = 0x00008000; // these are the reset value of cfgn
uint16_t left_amip4k_stat, right_amip4k_stat;
int32_t left_count_now, right_count_now, left_count_prev, right_count_prev;
int32_t left_counter_now, right_counter_now, left_counter_prev, right_counter_prev;
const int bms_uart_buffer_size = 132; // const int bms_uart_buffer_size = 127; // 
const uint8_t bms_uart_read_tout = 3; // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
const uint8_t bms_uart_buffer_bias = 73;
const int nc_uart_buffer_size = 127;
const uint8_t nc_uart_read_tout = 3; // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
bool isBootkeylongpressed;
uint8_t interval_count;


/* -------------------- master gpio do -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

void master_gpio_do_task(void* arg) {
	while(1) {
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


/* -------------------- kinco twai -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN1_TX, CAN1_RX, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

twai_message_t transmit_message = {
	.data_length_code = 8,
	.data = {0, 0, 0, 0, 0, 0, 0, 0},
};

void kinco_twai_task(void* arg) { // echo back received twai message, with identifier +1 and each data bytes +1
	while(1) {
		twai_message_t receive_message;
		if (twai_receive(&receive_message, pdMS_TO_TICKS(100)) == ESP_OK) {
			transmit_message.identifier = receive_message.identifier + 1;
			for (int twai_i = 0; twai_i < receive_message.data_length_code; twai_i++) {
				transmit_message.data[twai_i] = receive_message.data[twai_i] + 1;
			}
			ESP_ERROR_CHECK(twai_transmit(&transmit_message, pdMS_TO_TICKS(100)));
		}
		else {}
		vTaskDelay(pdMS_TO_TICKS(50));
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
	isBootkeylongpressed = (slave_to_master_buffer[4] & 0x1);
	interval_count = slave_to_master_buffer[5]; 
	all_do = (uint32_t)(master_do & 0xFFC) << 8 | (uint32_t)(slave_do & 0xC0) << 2 | (uint32_t)(master_do & 0x3) << 6 | (uint32_t)(slave_do & 0x3F) << 0;
}

void i2c_task(void *arg) { // I2C master task
	while (1) {
		i2c_master_help_get(master_to_slave_read_cmd);
 		vTaskDelay(pdMS_TO_TICKS(50));
 		i2c_master_help_set(master_to_slave_buffer);
 		vTaskDelay(pdMS_TO_TICKS(50));
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
unsigned int amip4k_spi_get_stat_id_rev(const spi_device_handle_t handle) { // SPI read 32 bit
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
	// assert(recvbuf3 == 0x4300);
	return recvbuf;
}

uint8_t amip4k_spi_help_assert_asicid_asicrev(const spi_device_handle_t handle){
	uint32_t buffer = amip4k_spi_get_stat_id_rev(handle);
	uint8_t value = (buffer & 0xFF000000) >> 24;
	assert(value == 0x43);
	return value;
}

uint16_t amip4k_spi_help_monitor(const spi_device_handle_t handle){
	uint32_t buffer = amip4k_spi_get_stat_id_rev(handle);
	uint16_t value = (buffer & 0x0000FFFF) >> 0;
	return value;
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

		accel_x = icm42688_spi_help_get_accel_x() - accel_x_calib; 
		accel_y = icm42688_spi_help_get_accel_y() - accel_y_calib; 
		accel_z = icm42688_spi_help_get_accel_z() - accel_z_calib; 
		gyro_x = icm42688_spi_help_get_gyro_x() - gyro_x_calib; 
		gyro_y = icm42688_spi_help_get_gyro_y() - gyro_y_calib; 
		gyro_z = icm42688_spi_help_get_gyro_z() - gyro_z_calib; 

		left_count_now = amip4k_spi_help_mval(spi_device_handle_l) * -1; // beware of the direction
		right_count_now = amip4k_spi_help_mval(spi_device_handle_r);
		left_amip4k_stat = amip4k_spi_help_monitor(spi_device_handle_l);
		right_amip4k_stat = amip4k_spi_help_monitor(spi_device_handle_r);
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
    }
}

void bms_uart_task(void *arg) { 
	while (1) {
		ESP_ERROR_CHECK(uart_flush(UART_NUM_1));
		uint8_t* data = (uint8_t*) malloc(bms_uart_buffer_size); // Allocate buffers for UART
		int len = uart_read_bytes(UART_NUM_1, data, bms_uart_buffer_size, (100/portTICK_PERIOD_MS)); // Read data from UART
		if (len > 0) { // Write data back to UART
			bms_uart_help_echo_send(UART_NUM_1, "\r\n", 2);
			char prefix[] = "Shoalbot's RS485_1 received: [";
			bms_uart_help_echo_send(UART_NUM_1, prefix, (sizeof(prefix) - 1));
			for (int i = 0; i < len; i++) {
				bms_uart_help_echo_send(UART_NUM_1, (const char*) &data[i], 1); 
				if (data[i] == '\r') { // Add a Newline character if get a return charater from paste
					bms_uart_help_echo_send(UART_NUM_1, "\n", 1);
				}
			}
			bms_uart_help_echo_send(UART_NUM_1, "]\r\n", 3);
		}
		else { // Echo a "." to show alive while waiting for input
			bms_uart_help_echo_send(UART_NUM_1, ".", 1); 
			ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_1, (10/portTICK_PERIOD_MS)));
		}
		free(data);
		vTaskDelay(pdMS_TO_TICKS(50));
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
    }
}

void nc_uart_task(void *arg) {  
	while (1) {
		ESP_ERROR_CHECK(uart_flush(UART_NUM_2));
		uint8_t* data = (uint8_t*) malloc(nc_uart_buffer_size); // Allocate buffers for UART
		int len = uart_read_bytes(UART_NUM_2, data, nc_uart_buffer_size, (100/portTICK_PERIOD_MS)); // Read data from UART
		if (len > 0) { // Write data back to UART
			nc_uart_help_echo_send(UART_NUM_2, "\r\n", 2);
			char prefix[] = "Shoalbot's RS485_2 received: [";
			nc_uart_help_echo_send(UART_NUM_2, prefix, (sizeof(prefix) - 1));
			for (int i = 0; i < len; i++) {
				nc_uart_help_echo_send(UART_NUM_2, (const char*) &data[i], 1); 
				if (data[i] == '\r') { // Add a Newline character if get a return charater from paste
					nc_uart_help_echo_send(UART_NUM_2, "\n", 1);
				}
			}
			nc_uart_help_echo_send(UART_NUM_2, "]\r\n", 3);
		}
		else { // Echo a "." to show alive while waiting for input
			nc_uart_help_echo_send(UART_NUM_2, ".", 1); 
			ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, (10/portTICK_PERIOD_MS)));
		}
		free(data);
		vTaskDelay(pdMS_TO_TICKS(50));
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
	
	ESP_ERROR_CHECK(gpio_reset_pin(DO_13)); //neccessary reset MTCK
	ESP_ERROR_CHECK(gpio_reset_pin(DO_14)); //MTDO
	ESP_ERROR_CHECK(gpio_reset_pin(DO_15)); //MTDI
	ESP_ERROR_CHECK(gpio_reset_pin(DO_16)); //MTMS

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

	ESP_LOGI(KINCO_TWAI_TAG, "Install twai driver");
	ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
	ESP_LOGI(KINCO_TWAI_TAG, "Start twai driver");
	ESP_ERROR_CHECK(twai_start());

	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Initialize spi bus");
	ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_config, SPI_DMA_CH_AUTO));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Allocate imu device on spi bus");
	ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &spi_device_interface_config_imu, &spi_device_handle_imu));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Assert imu who am i");
	icm42688_spi_assert_who_am_i();
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Set imu power mode");
	icm42688_spi_pwr_mgmt0(0b00001111); // -:00, TEMP_DIS:0, IDLE:0, GYROMODE:11, ACCEL_MODE:11
	vTaskDelay(pdMS_TO_TICKS(100)); // Essential wait for imu turn on
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

	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Allocate left sincos device on spi bus");
	ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &spi_device_interface_config_l, &spi_device_handle_l));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Allocate right sincos device on spi bus");
	ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &spi_device_interface_config_r, &spi_device_handle_r));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Assert sincos id rev");
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_assert left asic id/rev: 0x%02X", amip4k_spi_help_assert_asicid_asicrev(spi_device_handle_l));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_assert right asic id/rev: 0x%02X", amip4k_spi_help_assert_asicid_asicrev(spi_device_handle_r));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Reset and reconfigure IC left and right");
	amip4k_spi_set_cmd(spi_device_handle_l, 0x08); // RESIC 0b00001000
	amip4k_spi_set_cmd(spi_device_handle_r, 0x08); // RESIC 0b00001000
	vTaskDelay(pdMS_TO_TICKS(200)); // Essential wait for sincos encoder reset and reconfig
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Set sincos interpolation rate");
	amip4k_spi_help_interpolation_rate(spi_device_handle_l, Interpolation_rate_4);
	amip4k_spi_help_interpolation_rate(spi_device_handle_r, Interpolation_rate_4);
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Set sincos cfg3: DISZ");
	amip4k_cfg3 = 0x00008100; // DISZ=1
	amip4k_spi_set_cfg3(spi_device_handle_l, amip4k_cfg3);
	amip4k_spi_set_cfg3(spi_device_handle_r, amip4k_cfg3);
	vTaskDelay(pdMS_TO_TICKS(100)); // Essential wait for sincos config done
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Reload count value of pre_st, reset all error flag, zstat IC left and right");
	amip4k_spi_set_cmd(spi_device_handle_l, 0x01); // RESCNT 0b00000001
	amip4k_spi_set_cmd(spi_device_handle_r, 0x01); // RESCNT 0b00000001
	vTaskDelay(pdMS_TO_TICKS(100)); // Essential wait for sincos reset count done
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_left cfg1: 0x%08X", amip4k_spi_get_cfg1(spi_device_handle_l));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_right cfg1: 0x%08X", amip4k_spi_get_cfg1(spi_device_handle_r));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_left cfg2: 0x%08X", amip4k_spi_get_cfg2(spi_device_handle_l));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_right cfg2: 0x%08X", amip4k_spi_get_cfg2(spi_device_handle_r));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_left cfg3: 0x%08X", amip4k_spi_get_cfg3(spi_device_handle_l));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_right cfg3: 0x%08X", amip4k_spi_get_cfg3(spi_device_handle_r)); 
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_left stat: 0x%04X", amip4k_spi_help_monitor(spi_device_handle_l));
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "_right stat: 0x%04X", amip4k_spi_help_monitor(spi_device_handle_r));

	ESP_LOGI(MASTER_I2C_TAG, "Allocate i2c master bus");
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &i2c_master_bus_handle));
	ESP_LOGI(MASTER_I2C_TAG, "Add i2c master bus device");
	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_master_bus_handle, &i2c_device_config, &i2c_master_dev_handle));

	ESP_LOGI(BMS_UART_TAG,"Install bms uart driver");
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, bms_uart_buffer_size * 2, 0, 0, NULL, 0)); // no tx buffer, no event queue
	ESP_LOGI(BMS_UART_TAG,"Configure bms uart parameter");
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &bms_uart_config));
	ESP_LOGI(BMS_UART_TAG,"Assign signals of bms uart peripheral to gpio pins");
	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, RS1_TX, RS1_RX, RS1_DE, UART_PIN_NO_CHANGE));
	ESP_LOGI(BMS_UART_TAG,"Set bms uart to rs485 half duplex mode");
	ESP_ERROR_CHECK(uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX));
	ESP_LOGI(BMS_UART_TAG,"Set bms uart read threshold timeout for TOUT feature");
	ESP_ERROR_CHECK(uart_set_rx_timeout(UART_NUM_1, bms_uart_read_tout));
	ESP_LOGI(BMS_UART_TAG, "Discard all data in the bms uart rx buffer");
	ESP_ERROR_CHECK(uart_flush(UART_NUM_1));
	
	ESP_LOGI(NC_UART_TAG,"Install nc uart driver");
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, nc_uart_buffer_size * 2, 0, 0, NULL, 0)); // no tx buffer, no event queue
	ESP_LOGI(NC_UART_TAG,"Configure nc uart parameter");
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &nc_uart_config));
	ESP_LOGI(NC_UART_TAG,"Assign signals of nc uart peripheral to gpio pins");
	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, RS2_TX, RS2_RX, RS2_DE, UART_PIN_NO_CHANGE));
	ESP_LOGI(NC_UART_TAG,"Set nc uart to rs485 half duplex mode");
	ESP_ERROR_CHECK(uart_set_mode(UART_NUM_2, UART_MODE_RS485_HALF_DUPLEX));
	ESP_LOGI(NC_UART_TAG,"Set nc uart read threshold timeout for TOUT feature");
	ESP_ERROR_CHECK(uart_set_rx_timeout(UART_NUM_2, nc_uart_read_tout));
	ESP_LOGI(NC_UART_TAG, "Discard all data in the nc uart rx buffer");
	ESP_ERROR_CHECK(uart_flush(UART_NUM_2));

	ESP_LOGI(MASTER_GPIO_TAG, "Create master gpio task");
	xTaskCreate(master_gpio_do_task, "master_gpio_do_task", 4096, NULL, 5, NULL);
	ESP_LOGI(MASTER_I2C_TAG, "Create i2c task");
	xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 5, NULL);
	ESP_LOGI(ICM42688_AMIP4K_SPI_TAG, "Create imu and encoder task");
	xTaskCreate(spi_task, "spi_task", 4096, NULL, 5, NULL);
	ESP_LOGI(KINCO_TWAI_TAG, "Create kinco twai task");
	xTaskCreate(kinco_twai_task, "kinco_twai_task", 4096, NULL, 1, NULL);
	ESP_LOGI(BMS_UART_TAG, "Create bms uart task");
	xTaskCreate(bms_uart_task, "bms_uart_task", 4096, NULL, 1, NULL);
	ESP_LOGI(NC_UART_TAG, "Create nc uart task");
	xTaskCreate(nc_uart_task, "nc_uart_task", 4096, NULL, 1, NULL);

	esp_intr_dump(NULL);

	vTaskDelay(pdMS_TO_TICKS(1000));

//	ESP_LOGI(SHOALBOARD_TEST_TAG, "Create shoalboard test task");
//	xTaskCreate(shoalboard_test_task, "shoalboard_test_task", 16000, NULL, 1, NULL);

	setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

	bool write = false;
    while(1){
		char tmp[10] = {};
		uint8_t D;
		printf("\nWrite the command to execute\n");
		printf("for writing IO -> W-DXX_H (w-dXX_h) or W-DXX_L (w-dXX_l)   BMS: 50   PASS_1: 51   PASS_2: 52\n");

		if (scanf("%9s", tmp) == 1) {
			printf("Received : %s          ", tmp);
			if(strcmp(tmp, "p2") == 0 || strcmp(tmp, "p1") == 0 || strcmp(tmp, "bms") == 0 || strcmp(tmp, "bootkey") == 0) {
				// something
			}
			else {
				write = (tmp[0] == 'W' || tmp[0] == 'w') ? 1 : 0;
				uint8_t index = 2;
				uint8_t digits = 0;
				if(tmp[1] == '-') {
					while(tmp[index] != '_' && tmp[index] != '\0') {
						++digits;
						++index;
					}
					digits--;

					if (digits > 0 && digits < 3) { // valid iff digits==1,2
						if (digits == 2) {
							D = (tmp[3] - '0') * 10 + (tmp[4] - '0'); 
						} 
						else {
							D = tmp[3] - '0'; 
						}
						if (write) {
							if (tmp[index + 1] == 'H' || tmp[index + 1] == 'h') {
								printf("Writing HIGH on D%d\n", D);
								switch(D) {
									case 0: master_to_slave_buffer[1] |= 0x01; break;
									case 1: master_to_slave_buffer[1] |= 0x02; break;
									case 2: master_to_slave_buffer[1] |= 0x04; break;
									case 3: master_to_slave_buffer[1] |= 0x08; break;
									case 4: master_to_slave_buffer[1] |= 0x10; break;
									case 5: master_to_slave_buffer[1] |= 0x20; break;
									case 8: master_to_slave_buffer[1] |= 0x40; break;
									case 9: master_to_slave_buffer[1] |= 0x80; break;
									case 6: master_do |= 0x0001; break;
									case 7: master_do |= 0x0002; break;
									case 10: master_do |= 0x0004; break;
									case 11: master_do |= 0x0008; break;
									case 12: master_do |= 0x0010; break;
									case 13: master_do |= 0x0020; break;
									case 14: master_do |= 0x0040; break;
									case 15: master_do |= 0x0080; break;
									case 16: master_do |= 0x0100; break;
									case 17: master_do |= 0x0200; break;
									case 18: master_do |= 0x0400; break;
									case 19: master_do |= 0x0800; break;
									case 50: master_to_slave_buffer[2] |= 0x01; break; // BMS
									case 51: master_to_slave_buffer[2] |= 0x02; break; // PASS_1
									case 52: master_to_slave_buffer[2] |= 0x04; break; // PASS_2
									default: printf("No such kind of IO\n"); break;
								}
								write = false;
							} 
							else if (tmp[index + 1] == 'L' || tmp[index + 1] == 'l') {
								printf("Writing LOW on D%d\n", D);
								switch(D) {
									case 0: master_to_slave_buffer[1] &= 0xFE; break;
									case 1: master_to_slave_buffer[1] &= 0xFD; break;
									case 2: master_to_slave_buffer[1] &= 0xFB; break;
									case 3: master_to_slave_buffer[1] &= 0xF7; break;
									case 4: master_to_slave_buffer[1] &= 0xEF; break;
									case 5: master_to_slave_buffer[1] &= 0xDF; break;
									case 8: master_to_slave_buffer[1] &= 0xBF; break;
									case 9: master_to_slave_buffer[1] &= 0x7F; break;
									case 6: master_do &= 0xFFFE; break;
									case 7: master_do &= 0xFFFD; break;
									case 10: master_do &= 0xFFFB; break;
									case 11: master_do &= 0xFFF7; break;
									case 12: master_do &= 0xFFEF; break;
									case 13: master_do &= 0xFFDF; break;
									case 14: master_do &= 0xFFBF; break;
									case 15: master_do &= 0xFF7F; break;
									case 16: master_do &= 0xFEFF; break;
									case 17: master_do &= 0xFDFF; break;
									case 18: master_do &= 0xFBFF; break;
									case 19: master_do &= 0xF7FF; break;
									case 50: master_to_slave_buffer[2] &= 0xFE; break; // BMS
									case 51: master_to_slave_buffer[2] &= 0xFD; break; // PASS_1
									case 52: master_to_slave_buffer[2] &= 0xFB; break; // PASS_2
									default: printf("No such kind of IO\n"); break;
								}
								write = false;
							} 
							else {
								printf("INVALID!!\n");
							}
						} // back to if (write)
						else {
							printf("INVALID!!\n");
						}
					} // back to if (digits > 0 && digits < 3)
					else {
						printf("No such kind of IO\n");
					}
				} // back to if (tmp[1] == '-')
		
				else {
					printf("INVALID!!\n");
				}
			}
    	} // back to if (scanf("%9s", tmp) == 1)
		vTaskDelay(pdMS_TO_TICKS(500));
		printf("Status BOOTKEY DI 23 22 21 20 19 18 17 16 15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0 \n");
		printf("       %d          %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d\n", (uint8_t) isBootkeylongpressed,
		(uint8_t) ((slave_di & 0x800000)>>23), (uint8_t) ((slave_di & 0x400000)>>22), (uint8_t) ((slave_di & 0x200000)>>21), (uint8_t) ((slave_di & 0x100000)>>20), 
		(uint8_t) ((slave_di & 0x080000)>>19), (uint8_t) ((slave_di & 0x040000)>>18), (uint8_t) ((slave_di & 0x020000)>>17), (uint8_t) ((slave_di & 0x010000)>>16), 
		(uint8_t) ((slave_di & 0x008000)>>15), (uint8_t) ((slave_di & 0x004000)>>14), (uint8_t) ((slave_di & 0x002000)>>13), (uint8_t) ((slave_di & 0x001000)>>12),
		(uint8_t) ((slave_di & 0x000800)>>11), (uint8_t) ((slave_di & 0x000400)>>10), (uint8_t) ((slave_di & 0x000200)>>9), (uint8_t) ((slave_di & 0x000100)>>8),
		(uint8_t) ((slave_di & 0x000080)>>7), (uint8_t) ((slave_di & 0x000040)>>6), (uint8_t) ((slave_di & 0x000020)>>5), (uint8_t) ((slave_di & 0x000010)>>4),
		(uint8_t) ((slave_di & 0x000008)>>3), (uint8_t) ((slave_di & 0x000004)>>2), (uint8_t) ((slave_di & 0x000002)>>1), (uint8_t) ((slave_di & 0x000001)));
		printf("Estop pulses interval count: %d   Left encoder: %ld   Right encoder: %ld\n", interval_count, left_counter_now, right_counter_now);
		printf("AM-IP4k status: Left: %d   Right: %d\n", left_amip4k_stat, right_amip4k_stat);
		printf("Acc_X: %f   Acc_Y: %f   Acc_Z: %f   Gyr_X: %f   Gyr_Y: %f   Gyr_Z: %f\n", accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
    } // back to while (1)
}