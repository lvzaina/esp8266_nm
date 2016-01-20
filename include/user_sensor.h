#ifndef __USER_SENSOR_H__
#define __USER_SENSOR_H__

#include "user_config.h"
#include "driver/key.h"

#define SENSOR_KEY_NUM    1

#define SENSOR_KEY_IO_MUX     PERIPHS_IO_MUX_MTDI_U
#define SENSOR_KEY_IO_NUM     12
#define SENSOR_KEY_IO_FUNC    FUNC_GPIO12

#define SENSOR_LINK_LED_IO_MUX     PERIPHS_IO_MUX_MTMS_U
#define SENSOR_LINK_LED_IO_NUM     14
#define SENSOR_LINK_LED_IO_FUNC    FUNC_GPIO14

#define SENSOR_WIFI_LED_IO_MUX     PERIPHS_IO_MUX_MTCK_U
#define SENSOR_WIFI_LED_IO_NUM     13
#define SENSOR_WIFI_LED_IO_FUNC    FUNC_GPIO13

#define SENSOR_UNUSED_LED_IO_MUX     PERIPHS_IO_MUX_MTDO_U
#define SENSOR_UNUSED_LED_IO_NUM     15
#define SENSOR_UNUSED_LED_IO_FUNC    FUNC_GPIO15

#if HUMITURE_SUB_DEVICE
bool user_mvh3004_read_th(uint8 *data);
void user_mvh3004_init(void);
#endif

void user_sensor_init(uint8 active);

#define SENSOR_ALARM_IO_MUX		PERIPHS_IO_MUX_GPIO0_U
#define SENSOR_ALARM_IO_NUM		0	
#define SENSOR_ALARM_IO_FUNC	FUNC_GPIO0

#define SENSOR_PIR_IO_MUX		PERIPHS_IO_MUX_GPIO5_U
#define SENSOR_PIR_IO_NUM		5
#define SENSOR_PIR_IO_FUNC		FUNC_GPIO5

#define SENSOR_BEEP_IO_MUX		PERIPHS_IO_MUX_GPIO4_U
#define SENSOR_BEEP_IO_NUM		4
#define SENSOR_BEEP_IO_FUNC		FUNC_GPIO4
#define BUZZER(x) GPIO_OUTPUT_SET(GPIO_ID_PIN(4), x) 

void user_beep_timer_cb( uint8 times);
void user_beep_timer_cb_long( uint8 times);
void user_alert_led_timer_cb( uint8 times);
void ICACHE_FLASH_ATTR user_unlink_led_init( uint32_t frequency);
void ICACHE_FLASH_ATTR user_unlink_led_done();
void ICACHE_FLASH_ATTR start_pir();
void ICACHE_FLASH_ATTR init_netviom_device();
void ICACHE_FLASH_ATTR stop_pir();
void ICACHE_FLASH_ATTR close_netviom_device();

void ICACHE_FLASH_ATTR user_sensor_long_press(void);
#endif
