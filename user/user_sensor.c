/******************************************************************************
 * Copyright 2013-2014 Espressif Systems (Wuxi)
 *
 * FileName: user_humiture.c
 *
 * Description: humiture demo's function realization
 *
 * Modification history:
 *     2014/5/1, v1.0 create this file.
*******************************************************************************/
#include "ets_sys.h"
#include "osapi.h"
#include "os_type.h"
#include "user_interface.h"
#include "smartconfig.h"
#include "user_devicefind.h"

#if SENSOR_DEVICE
#include "user_sensor.h"

LOCAL struct keys_param keys;
LOCAL struct single_key_param *single_key[SENSOR_KEY_NUM];
LOCAL os_timer_t sensor_sleep_timer;
LOCAL os_timer_t link_led_timer;
LOCAL uint8 link_led_level = 0;
LOCAL uint32 link_start_time;

#if HUMITURE_SUB_DEVICE
#include "driver/i2c_master.h"

#define MVH3004_Addr    0x88

LOCAL uint8 humiture_data[4];

/******************************************************************************
 * FunctionName : user_mvh3004_burst_read
 * Description  : burst read mvh3004's internal data
 * Parameters   : uint8 addr - mvh3004's address
 *                uint8 *pData - data point to put read data
 *                uint16 len - read length
 * Returns      : bool - true or false
*******************************************************************************/
LOCAL bool ICACHE_FLASH_ATTR
user_mvh3004_burst_read(uint8 addr, uint8 *pData, uint16 len)
{
    uint8 ack;
    uint16 i;

    i2c_master_start();
    i2c_master_writeByte(addr);
    ack = i2c_master_getAck();

    if (ack) {
        os_printf("addr not ack when tx write cmd \n");
        i2c_master_stop();
        return false;
    }

    i2c_master_stop();
    i2c_master_wait(40000);

    i2c_master_start();
    i2c_master_writeByte(addr + 1);
    ack = i2c_master_getAck();

    if (ack) {
        os_printf("addr not ack when tx write cmd \n");
        i2c_master_stop();
        return false;
    }

    for (i = 0; i < len; i++) {
        pData[i] = i2c_master_readByte();

        i2c_master_setAck((i == (len - 1)) ? 1 : 0);
    }

    i2c_master_stop();

    return true;
}

/******************************************************************************
 * FunctionName : user_mvh3004_read_th
 * Description  : read mvh3004's humiture data
 * Parameters   : uint8 *data - where data to put
 * Returns      : bool - ture or false
*******************************************************************************/
bool ICACHE_FLASH_ATTR
user_mvh3004_read_th(uint8 *data)
{
    return user_mvh3004_burst_read(MVH3004_Addr, data, 4);
}

/******************************************************************************
 * FunctionName : user_mvh3004_init
 * Description  : init mvh3004, mainly i2c master gpio
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void ICACHE_FLASH_ATTR
user_mvh3004_init(void)
{
    i2c_master_gpio_init();
}

uint8 *ICACHE_FLASH_ATTR
user_mvh3004_get_poweron_th(void)
{
    return humiture_data;
}
#endif

#define gpio_io_wait    os_delay_us

LOCAL void ICACHE_FLASH_ATTR
user_link_led_init(void)
{
    PIN_FUNC_SELECT(SENSOR_LINK_LED_IO_MUX, SENSOR_LINK_LED_IO_FUNC);
    PIN_FUNC_SELECT(SENSOR_UNUSED_LED_IO_MUX, SENSOR_UNUSED_LED_IO_FUNC);
    GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_UNUSED_LED_IO_NUM), 0);
}

void ICACHE_FLASH_ATTR
user_link_led_output(uint8 level)
{
    GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), level);
}

LOCAL void ICACHE_FLASH_ATTR
user_link_led_timer_cb(void)
{
    link_led_level = (~link_led_level) & 0x01;
    GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), link_led_level);
}

void ICACHE_FLASH_ATTR
user_link_led_timer_init(void)
{
    link_start_time = system_get_time();

    os_timer_disarm(&link_led_timer);
    os_timer_setfn(&link_led_timer, (os_timer_func_t *)user_link_led_timer_cb, NULL);
    os_timer_arm(&link_led_timer, 50, 1);
    link_led_level = 0;
    GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), link_led_level);
}

void ICACHE_FLASH_ATTR
user_link_led_timer_done(void)
{
    os_timer_disarm(&link_led_timer);
    GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), 0);
}

void ICACHE_FLASH_ATTR
smartconfig_done(sc_status status, void *pdata)
{
    switch(status) {
        case SC_STATUS_WAIT:
            os_printf("SC_STATUS_WAIT\n");
            break;
        case SC_STATUS_FIND_CHANNEL:
            os_printf("SC_STATUS_FIND_CHANNEL\n");
            break;
        case SC_STATUS_GETTING_SSID_PSWD:
            os_printf("SC_STATUS_GETTING_SSID_PSWD\n");
			sc_type *type = pdata;
            if (*type == SC_TYPE_ESPTOUCH) {
                os_printf("SC_TYPE:SC_TYPE_ESPTOUCH\n");
            } else {
                os_printf("SC_TYPE:SC_TYPE_AIRKISS\n");
            }
            break;
        case SC_STATUS_LINK:
            os_printf("SC_STATUS_LINK\n");
            struct station_config *sta_conf = pdata;
	
	        wifi_station_set_config(sta_conf);
	        wifi_station_disconnect();
	        wifi_station_connect();
            break;
        case SC_STATUS_LINK_OVER:
            os_printf("SC_STATUS_LINK_OVER\n");
            if (pdata != NULL) {
                uint8 phone_ip[4] = {0};

                os_memcpy(phone_ip, (uint8*)pdata, 4);
                os_printf("Phone ip: %d.%d.%d.%d\n",phone_ip[0],phone_ip[1],phone_ip[2],phone_ip[3]);
            }
            smartconfig_stop();
			user_link_led_timer_done();
			user_devicefind_init();
            break;
    }
	
}

/******************************************************************************
 * FunctionName : user_humiture_long_press
 * Description  : humiture key's function, needed to be installed
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
user_sensor_long_press(void)
{
    os_printf(" LVZAINA ===> SmartConfig start! \n");
	
	uint32 gpio_status;
	gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	//clear interrupt status
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);
	
	user_link_led_timer_init();	
	
	smartconfig_stop();
    smartconfig_start(smartconfig_done);
	
    os_printf(" LVZAINA ===> SmartConfig end! \n");
}

#ifdef SENSOR_DEEP_SLEEP
void ICACHE_FLASH_ATTR
user_sensor_deep_sleep_enter(void)
{
    system_deep_sleep(SENSOR_DEEP_SLEEP_TIME > link_start_time \
    ? SENSOR_DEEP_SLEEP_TIME - link_start_time : 30000000);
}

void ICACHE_FLASH_ATTR
user_sensor_deep_sleep_disable(void)
{
    os_timer_disarm(&sensor_sleep_timer);
}

void ICACHE_FLASH_ATTR
user_sensor_deep_sleep_init(uint32 time)
{
    os_timer_disarm(&sensor_sleep_timer);
    os_timer_setfn(&sensor_sleep_timer, (os_timer_func_t *)user_sensor_deep_sleep_enter, NULL);
    os_timer_arm(&sensor_sleep_timer, time, 0);
}
#endif

/******************************************************************************
 * FunctionName : user_humiture_init
 * Description  : init humiture function, include key and mvh3004
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void ICACHE_FLASH_ATTR
user_sensor_init(uint8 active)
{
    user_link_led_init();

    wifi_status_led_install(SENSOR_WIFI_LED_IO_NUM, SENSOR_WIFI_LED_IO_MUX, SENSOR_WIFI_LED_IO_FUNC);

    if (wifi_get_opmode() != SOFTAP_MODE) {
        single_key[0] = key_init_single(SENSOR_KEY_IO_NUM, SENSOR_KEY_IO_MUX, SENSOR_KEY_IO_FUNC,
                                        user_sensor_long_press, NULL);

        keys.key_num = SENSOR_KEY_NUM;
        keys.single_key = single_key;

        key_init(&keys);

        if (GPIO_INPUT_GET(GPIO_ID_PIN(SENSOR_KEY_IO_NUM)) == 0) {
            user_sensor_long_press();
        }
    }

#if HUMITURE_SUB_DEVICE
    user_mvh3004_init();
    user_mvh3004_read_th(humiture_data);
#endif

#ifdef SENSOR_DEEP_SLEEP
    if (wifi_get_opmode() != STATIONAP_MODE) {
        if (active == 1) {
            user_sensor_deep_sleep_init(SENSOR_DEEP_SLEEP_TIME / 1000 );
        } else {
            user_sensor_deep_sleep_init(SENSOR_DEEP_SLEEP_TIME / 1000 / 3 * 2);
        }
    }
#endif

}
#endif

