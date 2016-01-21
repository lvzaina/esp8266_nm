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
#include "user_netviom.h"
#if SENSOR_DEVICE
#include "user_sensor.h"

LOCAL void ICACHE_FLASH_ATTR
user_beep_timer_cb_close(void);
LOCAL void ICACHE_FLASH_ATTR
user_alert_led_timer_cb_close(void);
LOCAL struct keys_param keys;
LOCAL struct single_key_param *single_key[SENSOR_KEY_NUM];
LOCAL os_timer_t netviom_status_timer;
LOCAL os_timer_t sensor_sleep_timer;
LOCAL os_timer_t link_led_timer;
LOCAL os_timer_t unlink_led_timer;
LOCAL os_timer_t beep_timer;
LOCAL os_timer_t alert_led_timer;
LOCAL os_timer_t beep_timer_close;
LOCAL os_timer_t beep_timer_long_close;
LOCAL os_timer_t alert_led_timer_close;
#if NETVIOM_USE_PIR
LOCAL os_timer_t pir_timer;
#endif
LOCAL uint8 link_led_level = 0;
LOCAL uint8 beep_level = 0;
LOCAL uint8 alert_led_level = 0;
LOCAL uint32 link_start_time;

LOCAL NETVIOM_STATUS_VALUE netviom_old_state = NETVIOM_STATUS_POWERUP;
LOCAL uint8_t netviom_old_server_state = 0;
LOCAL uint8 beep_times = 1;
LOCAL uint8 alert_led_times = 1;
LOCAL uint8 moded = 0;
LOCAL uint8 m_smart = 0;
LOCAL uint8 m_start = 0;
uint8_t open_settings_mode = 0;

uint8_t netviom_router_connected = 0;
uint8_t netviom_server_connected = 0;
LOCAL void ICACHE_FLASH_ATTR
set_beep( uint32_t value);

LOCAL uint8_t netviom_hide = 0;

void ICACHE_FLASH_ATTR user_link_led_timer_done(void);

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

void ICACHE_FLASH_ATTR
smartconfig_done(sc_status status, void *pdata)
{
    switch(status) {
        case SC_STATUS_WAIT:
            os_printf("SC_STATUS_WAIT\n");
            break;
        case SC_STATUS_FIND_CHANNEL:
            os_printf("SC_STATUS_FIND_CHANNEL\n");
			set_beep(0);
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
			user_devicefind_init();

			user_update_status( NETVIOM_STATUS_CONFIG_COMPLETED);
			break;
    }
	
}

#define gpio_io_wait    os_delay_us

LOCAL ICACHE_FLASH_ATTR
void start_connect( uint8 mode_force)
{
	if( moded != 1 || ( mode_force == 1))
	{
		wifi_set_opmode(STATION_MODE);
		moded = 1;	
	}
}

LOCAL void ICACHE_FLASH_ATTR
user_change_status( NETVIOM_STATUS_VALUE state, uint32_t delay_time)
{
	os_timer_disarm(&netviom_status_timer);
    os_timer_setfn(&netviom_status_timer, (os_timer_func_t *)user_update_status, state);
    os_timer_arm(&netviom_status_timer, delay_time, 0);
}

void ICACHE_FLASH_ATTR 
user_quick_status( NETVIOM_STATUS_VALUE state) 
{
	user_change_status( state, 10);
}

LOCAL void ICACHE_FLASH_ATTR
set_led_net( uint32_t value)
{
	if( netviom_hide != 1)
		GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_WIFI_LED_IO_NUM), value);
}

LOCAL void ICACHE_FLASH_ATTR
set_led_alert( uint32_t value)
{
	if( netviom_hide != 1)
		GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), value);
}

LOCAL void ICACHE_FLASH_ATTR
set_led_power( uint32_t value)
{
	if( netviom_hide != 1)
		GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_UNUSED_LED_IO_NUM), value);
}

LOCAL void ICACHE_FLASH_ATTR
set_beep( uint32_t value)
{
	if( netviom_hide != 1)
		BUZZER( value);	
}

LOCAL void ICACHE_FLASH_ATTR
user_unlink_led_cb()
{
    link_led_level = (~link_led_level) & 0x01;
    //GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_WIFI_LED_IO_NUM), link_led_level);
	set_led_net( link_led_level);
}

void ICACHE_FLASH_ATTR
user_unlink_led_init( uint32_t frequency)
{
	link_led_level = 0;
	os_timer_disarm(&unlink_led_timer);
    os_timer_setfn(&unlink_led_timer, (os_timer_func_t *)user_unlink_led_cb, NULL);
    os_timer_arm(&unlink_led_timer, frequency, 1);
}

void ICACHE_FLASH_ATTR
user_unlink_led_done()
{
	os_timer_disarm(&unlink_led_timer);
	set_led_alert( 1);
}

LOCAL void ICACHE_FLASH_ATTR
user_link_led_init(void)
{
    PIN_FUNC_SELECT(SENSOR_LINK_LED_IO_MUX, SENSOR_LINK_LED_IO_FUNC);
    PIN_FUNC_SELECT(SENSOR_UNUSED_LED_IO_MUX, SENSOR_UNUSED_LED_IO_FUNC);
    PIN_FUNC_SELECT(SENSOR_WIFI_LED_IO_MUX, SENSOR_WIFI_LED_IO_FUNC);
	PIN_PULLUP_DIS(SENSOR_UNUSED_LED_IO_MUX);

	//GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), 1);
	set_led_alert( 1);
}

LOCAL void ICACHE_FLASH_ATTR
user_link_led_timer_cb(void)
{
    link_led_level = (~link_led_level) & 0x01;
    //GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), link_led_level);
    //GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_WIFI_LED_IO_NUM), link_led_level);
    //GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_UNUSED_LED_IO_NUM), link_led_level);
	set_led_alert( link_led_level);
	set_led_net( link_led_level);
	set_led_power( link_led_level);
}

void ICACHE_FLASH_ATTR
user_link_led_timer_init(void)
{
    link_start_time = system_get_time();

    os_timer_disarm(&link_led_timer);
    os_timer_setfn(&link_led_timer, (os_timer_func_t *)user_link_led_timer_cb, NULL);
    os_timer_arm(&link_led_timer, 500, 1);
    link_led_level = 0;
    //GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), link_led_level);
    //GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_WIFI_LED_IO_NUM), link_led_level);
    //GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_UNUSED_LED_IO_NUM), link_led_level);
	set_led_alert( link_led_level);
	set_led_net( link_led_level);
	set_led_power( link_led_level);
}

void ICACHE_FLASH_ATTR
user_link_led_timer_done(void)
{
    os_timer_disarm(&link_led_timer);
    //GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), 1);
    //GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_WIFI_LED_IO_NUM), 1);
    //GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_UNUSED_LED_IO_NUM), 1);
	set_led_alert( 1);
	set_led_net( 1);
	set_led_power( 1);
}

void user_alert_led_timer_cb( uint8 times)
{
	alert_led_times = times;

    //GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), 0);
	set_led_alert( 0);

	os_timer_disarm(&alert_led_timer_close);
	os_timer_setfn(&alert_led_timer_close, (os_timer_func_t *)user_alert_led_timer_cb_close, NULL);
	os_timer_arm(&alert_led_timer_close, 500, 0);

}

void user_alert_led_timer_cb_loop()
{
    //GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), 0);
	set_led_alert( 0);

	os_timer_disarm(&alert_led_timer_close);
	os_timer_setfn(&alert_led_timer_close, (os_timer_func_t *)user_alert_led_timer_cb_close, NULL);
	os_timer_arm(&alert_led_timer_close, 500, 0);

}

LOCAL void ICACHE_FLASH_ATTR
user_alert_led_timer_cb_close(void)
{
    //GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), 1);
	set_led_alert( 1);
	if( (--alert_led_times) > 0)
	{
		os_timer_disarm(&alert_led_timer);
		os_timer_setfn(&alert_led_timer, (os_timer_func_t *)user_alert_led_timer_cb_loop, NULL);
		os_timer_arm(&alert_led_timer, 500, 0);
	}
}

void user_beep_timer_cb_long( uint8 time)
{
	beep_times = 1;
	//BUZZER(1);
	set_beep(1);

	os_timer_disarm(&beep_timer_long_close);
	os_timer_setfn(&beep_timer_long_close, (os_timer_func_t *)user_beep_timer_cb_close, NULL);
	os_timer_arm(&beep_timer_long_close, time*1000, 0);

}


void user_beep_timer_cb_long_done( )
{
	//BUZZER(1);

	os_timer_disarm(&beep_timer_long_close);
	set_beep(0);

}

void user_beep_timer_cb( uint8 times)
{
	beep_times = times;

	//BUZZER(1);
	set_beep(1);

	os_timer_disarm(&beep_timer_close);
	os_timer_setfn(&beep_timer_close, (os_timer_func_t *)user_beep_timer_cb_close, NULL);
	os_timer_arm(&beep_timer_close, 500, 0);

}

void user_beep_timer_cb_done( )
{

	//BUZZER(1);

	os_timer_disarm(&beep_timer_close);
	set_beep(0);

}

void user_beep_timer_cb_loop()
{
	//BUZZER(1);
	set_beep(1);

	os_timer_disarm(&beep_timer_close);
	os_timer_setfn(&beep_timer_close, (os_timer_func_t *)user_beep_timer_cb_close, NULL);
	os_timer_arm(&beep_timer_close, 500, 0);

}

LOCAL void ICACHE_FLASH_ATTR
user_beep_timer_cb_close(void)
{
	//BUZZER(0);
	set_beep(0);
	if( (--beep_times) > 0)
	{
		os_timer_disarm(&beep_timer);
		os_timer_setfn(&beep_timer, (os_timer_func_t *)user_beep_timer_cb_loop, NULL);
		os_timer_arm(&beep_timer, 500, 0);
	}
	else
	{
		if( m_smart == 1)
		{
			netviom_hide = 1;
			m_smart = 0;
		}
	}
	
}

LOCAL void ICACHE_FLASH_ATTR
user_beep_init(void)
{
	beep_level = 0;
	//BUZZER(0);
	set_beep(0);

    PIN_FUNC_SELECT(SENSOR_BEEP_IO_MUX, SENSOR_BEEP_IO_FUNC);
	
	PIN_PULLUP_DIS( SENSOR_BEEP_IO_MUX);
	//PIN_PULLUP_EN( SENSOR_BEEP_IO_MUX);
	if( open_settings_mode == 0)
		user_beep_timer_cb( 1);
	else
		user_beep_timer_cb( 2);	
}

/******************************************************************************
 * FunctionName : user_humiture_long_press
 * Description  : humiture key's function, needed to be installed
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void ICACHE_FLASH_ATTR
user_sensor_long_press(void)
{
    if (0 == GPIO_INPUT_GET(GPIO_ID_PIN( SENSOR_KEY_IO_NUM))) 
	{
		os_printf(" LVZAINA ===> SmartConfig start! \n");
		user_update_status( NETVIOM_STATUS_CONFIG);	
	}
}

LOCAL void ICACHE_FLASH_ATTR
user_sensor_short_press(void)
{
    os_printf(" LVZAINA ===> short start! \n");
	
		
	
    os_printf(" LVZAINA ===> short end! \n");
	
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

void ICACHE_FLASH_ATTR 
user_update_status( NETVIOM_STATUS_VALUE state) 
{
	os_printf( " |LVZAINA| ===> %d ( %d, %d)\n", state, netviom_router_connected, netviom_server_connected);
	switch( state)
	{
		case NETVIOM_STATUS_POWERUP: // 0
			//GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_UNUSED_LED_IO_NUM), 0);
			//GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), 0);
			//GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_WIFI_LED_IO_NUM), 0);
			set_led_power( 0);
			set_led_alert( 0);
			set_led_net( 0);
			user_beep_init();	
			user_change_status( NETVIOM_STATUS_INIT, 1000);
			break;
		case NETVIOM_STATUS_INIT: // 1
#if 1
			init_netviom_device();
			//GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_UNUSED_LED_IO_NUM), 1);
			//GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), 1);
			//GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_WIFI_LED_IO_NUM), 1);
			set_led_power( 1);
			set_led_alert( 1);
			set_led_net( 1);

			user_change_status( NETVIOM_STATUS_CONNECTING, FLICKER_FREQUENCY_FAST);
#endif		
			break;
		case NETVIOM_STATUS_CONFIG: // 2
			
			netviom_router_connected = 0;
			netviom_server_connected = 0;
			stop_pir();
			os_timer_disarm(&pir_timer);
			close_netviom_device();
			//BUZZER( 0);
			set_beep(0);
			user_beep_timer_cb_long_done();
			user_beep_timer_cb_done();
			user_unlink_led_done();
			netviom_hide = 0;
			user_link_led_timer_init();	
			user_beep_timer_cb( 1);
			smartconfig_stop();
			smartconfig_start(smartconfig_done);

			break;
		case NETVIOM_STATUS_CONFIG_COMPLETED: // 3
			
			user_link_led_timer_done();
			set_led_alert( 1);
			set_led_net( 1);
			set_led_power( 1);
			set_beep( 0);

			init_netviom_device();
			
			user_beep_timer_cb( 3);
			m_smart = 1;
			
			break;
		case NETVIOM_STATUS_CONNECTING: // 4 
			//GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_UNUSED_LED_IO_NUM), 0);
			set_led_power( 0);
			start_connect( 0);
		case NETVIOM_STATUS_UNCONNECT: // 5
			netviom_router_connected = 0;
			netviom_server_connected = 0;
			// slow led flicker
			user_unlink_led_init( FLICKER_FREQUENCY_SLOW);
    
			break;
		case NETVIOM_STATUS_CONNECTED:		// 6 
			if( netviom_router_connected == 1)
			{
				if( netviom_server_connected != netviom_old_server_state || 
					netviom_old_state != NETVIOM_STATUS_CONNECTED)
				{
					if( netviom_server_connected  == 1)
					{

						os_printf( " |LVZAINA| ===> server connected! \n");
						
						user_unlink_led_done();

						//GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_UNUSED_LED_IO_NUM), 0);
						//GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_LINK_LED_IO_NUM), 1);
						//GPIO_OUTPUT_SET(GPIO_ID_PIN(SENSOR_WIFI_LED_IO_NUM), 0);
						set_led_power( 0);
						set_led_alert( 1);
						set_led_net( 0);
						
						//close_netviom_device();	
						//init_netviom_device();
#if NETVIOM_USE_PIR
						//stop_pir();
						os_timer_disarm(&pir_timer);
						os_timer_setfn(&pir_timer, (os_timer_func_t *)start_pir, NULL);
						if( m_start == 0)
						{
							os_timer_arm(&pir_timer, PIR_START_TIME, 0);	
							m_start = 1;
						}
						else
						{
							os_timer_arm(&pir_timer, PIR_CONFIG_START_TIME, 0);	
						}
#endif
					}
					else
					{
						// fast led flicker
						user_unlink_led_init( FLICKER_FREQUENCY_FAST);
					}
					netviom_old_server_state = netviom_server_connected;
				}
			}
			else
			{
				// slow led flicker
				user_unlink_led_init( FLICKER_FREQUENCY_SLOW);
			}
			break;
		case NETVIOM_STATUS_HIDE_MODE: // 7
			set_led_alert( 1);
			set_led_net( 1);
			set_led_power( 1);
			set_beep( 0);
			netviom_hide = 1;
			break;
		case NETVIOM_STATUS_OPEN_MODE: // 8
			netviom_hide = 0;
			set_led_alert( 1);
			set_led_power( 0);
			//user_beep_timer_cb( 1);
			user_quick_status( NETVIOM_STATUS_CONNECTED);
			break;
	}
	netviom_old_state = state;
}

/******************************************************************************
 * FunctionName : user_humiture_init
 * Description  : init humiture function, include key and mvh3004
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void ICACHE_FLASH_ATTR
user_sensor_init(uint8 active)
{
	uint32_t t_value = 9;
	user_link_led_init();
	user_change_status( NETVIOM_STATUS_POWERUP, 300);
	
	PIN_FUNC_SELECT( SENSOR_KEY_IO_MUX, SENSOR_KEY_IO_FUNC);
	gpio_output_set(0, 0, 0, GPIO_ID_PIN( SENSOR_KEY_IO_NUM));
	t_value = GPIO_INPUT_GET(GPIO_ID_PIN(SENSOR_KEY_IO_NUM)); 
	os_printf( " |LVZAINA| ===> MODE: [%d]\n", t_value);
	if ( t_value == 0) 
	{
		open_settings_mode = 1;
		/*user_sensor_short_press();*/
	}
   
    //wifi_status_led_install(SENSOR_WIFI_LED_IO_NUM, SENSOR_WIFI_LED_IO_MUX, SENSOR_WIFI_LED_IO_FUNC);
	/*
    if (wifi_get_opmode() != SOFTAP_MODE) {
		PIN_PULLUP_EN(SENSOR_KEY_IO_MUX);
        single_key[0] = key_init_single(SENSOR_KEY_IO_NUM, SENSOR_KEY_IO_MUX, SENSOR_KEY_IO_FUNC,
                                        user_sensor_long_press, NULL);

        keys.key_num = SENSOR_KEY_NUM;
        keys.single_key = single_key;

        key_init(&keys);

        if (GPIO_INPUT_GET(GPIO_ID_PIN(SENSOR_KEY_IO_NUM)) == 0) {
            user_sensor_long_press();
        }
    }
	
	ETS_GPIO_INTR_DISABLE(); // Disable gpio interrupts

	ETS_GPIO_INTR_ATTACH( user_sensor_long_press, NULL); // GPIO interrupt handler
	PIN_FUNC_SELECT( SENSOR_PIR_IO_MUX, SENSOR_PIR_IO_FUNC);
	GPIO_DIS_OUTPUT( SENSOR_PIR_IO_NUM);
	PIN_PULLUP_DIS( SENSOR_PIR_IO_MUX);
	//GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, BIT( SENSOR_PIR_IO_NUM)); // Clear GPIO status
	gpio_pin_intr_state_set( GPIO_ID_PIN( SENSOR_PIR_IO_NUM), GPIO_PIN_INTR_POSEDGE); // Interrupt on POSETIVE edge

	ETS_GPIO_INTR_ENABLE(); // Enable gpio interrupts

	*/

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

