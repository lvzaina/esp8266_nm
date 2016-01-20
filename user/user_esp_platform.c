/******************************************************************************
 * Copyright 2013-2014 Espressif Systems (Wuxi)
 *
 * FileName: user_esp_platform.c
 *
 * Description: The client mode configration.
 *              Check your hardware connection with the host while use this mode.
 *
 * Modification history:
 *     2014/5/09, v1.0 create this file.
*******************************************************************************/
#include "ets_sys.h"
#include "os_type.h"
#include "mem.h"
#include "osapi.h"
#include "gpio.h"
#include "driver/key.h"
#include "user_interface.h"

#include "espconn.h"
#include "user_esp_platform.h"
#include "user_iot_version.h"
#include "upgrade.h"
#include "user_netviom.h"

#if ESP_PLATFORM

#define ESP_DEBUG

#ifdef ESP_DEBUG
#define ESP_DBG os_printf
#else
#define ESP_DBG
#endif

#define ACTIVE_FRAME    "{\"nonce\": %d,\"path\": \"/v1/device/activate/\", \"method\": \"POST\", \"body\": {\"encrypt_method\": \"PLAIN\", \"token\": \"%s\", \"bssid\": \""MACSTR"\",\"rom_version\":\"%s\"}, \"meta\": {\"Authorization\": \"token %s\"}}\n"

#if SENSOR_DEVICE
#include "user_sensor.h"

#if HUMITURE_SUB_DEVICE
#define UPLOAD_FRAME  "{\"nonce\": %d, \"path\": \"/v1/datastreams/tem_hum/datapoint/\", \"method\": \"POST\", \
\"body\": {\"datapoint\": {\"x\": %s%d.%02d,\"y\": %d.%02d}}, \"meta\": {\"Authorization\": \"token %s\"}}\n"
#elif FLAMMABLE_GAS_SUB_DEVICE
#define UPLOAD_FRAME  "{\"nonce\": %d, \"path\": \"/v1/datastreams/flammable_gas/datapoint/\", \"method\": \"POST\", \
\"body\": {\"datapoint\": {\"x\": %d.%03d}}, \"meta\": {\"Authorization\": \"token %s\"}}\n"
#endif

LOCAL uint32 count = 0;
#endif

#define UPGRADE_FRAME  "{\"path\": \"/v1/messages/\", \"method\": \"POST\", \"meta\": {\"Authorization\": \"token %s\"},\
\"get\":{\"action\":\"%s\"},\"body\":{\"pre_rom_version\":\"%s\",\"rom_version\":\"%s\"}}\n"


LOCAL struct espconn user_conn;
LOCAL struct _esp_tcp user_tcp;
 struct esp_platform_saved_param esp_param;
LOCAL uint8 device_status;
LOCAL uint8 device_recon_count = 0;
LOCAL uint32 active_nonce = 0;
LOCAL uint8 iot_version[20] = {0};
struct rst_info rtc_info;
void user_esp_platform_check_ip(uint8 reset_flag);

/******************************************************************************
 * FunctionName : user_esp_platform_get_token
 * Description  : get the espressif's device token
 * Parameters   : token -- the parame point which write the flash
 * Returns      : none
*******************************************************************************/
void ICACHE_FLASH_ATTR
user_esp_platform_get_token(uint8_t *token)
{
    if (token == NULL) {
        return;
    }

    os_memcpy(token, esp_param.token, sizeof(esp_param.token));
}

/******************************************************************************
 * FunctionName : user_esp_platform_set_token
 * Description  : save the token for the espressif's device
 * Parameters   : token -- the parame point which write the flash
 * Returns      : none
*******************************************************************************/
void ICACHE_FLASH_ATTR
user_esp_platform_set_token(uint8_t *token)
{
    if (token == NULL) {
        return;
    }

    esp_param.activeflag = 0;
    os_memcpy(esp_param.token, token, os_strlen(token));

    system_param_save_with_protect(ESP_PARAM_START_SEC, &esp_param, sizeof(esp_param));
}

/******************************************************************************
 * FunctionName : user_esp_platform_set_active
 * Description  : set active flag
 * Parameters   : activeflag -- 0 or 1
 * Returns      : none
*******************************************************************************/
void ICACHE_FLASH_ATTR
user_esp_platform_set_active(uint8 activeflag)
{
    esp_param.activeflag = activeflag;

    system_param_save_with_protect(ESP_PARAM_START_SEC, &esp_param, sizeof(esp_param));
}

void ICACHE_FLASH_ATTR
user_esp_platform_set_connect_status(uint8 status)
{
    device_status = status;
}

/******************************************************************************
 * FunctionName : user_esp_platform_get_connect_status
 * Description  : get each connection step's status
 * Parameters   : none
 * Returns      : status
*******************************************************************************/
uint8 ICACHE_FLASH_ATTR
user_esp_platform_get_connect_status(void)
{
    uint8 status = wifi_station_get_connect_status();

    if (status == STATION_GOT_IP) {
        status = (device_status == 0) ? DEVICE_CONNECTING : device_status;
    }

    ESP_DBG("status %d\n", status);
    return status;
}

/******************************************************************************
 * FunctionName : user_esp_platform_parse_nonce
 * Description  : parse the device nonce
 * Parameters   : pbuffer -- the recivce data point
 * Returns      : the nonce
*******************************************************************************/
int ICACHE_FLASH_ATTR
user_esp_platform_parse_nonce(char *pbuffer)
{
    char *pstr = NULL;
    char *pparse = NULL;
    char noncestr[11] = {0};
    int nonce = 0;
    pstr = (char *)os_strstr(pbuffer, "\"nonce\": ");

    if (pstr != NULL) {
        pstr += 9;
        pparse = (char *)os_strstr(pstr, ",");

        if (pparse != NULL) {
            os_memcpy(noncestr, pstr, pparse - pstr);
        } else {
            pparse = (char *)os_strstr(pstr, "}");

            if (pparse != NULL) {
                os_memcpy(noncestr, pstr, pparse - pstr);
            } else {
                pparse = (char *)os_strstr(pstr, "]");

                if (pparse != NULL) {
                    os_memcpy(noncestr, pstr, pparse - pstr);
                } else {
                    return 0;
                }
            }
        }

        nonce = atoi(noncestr);
    }

    return nonce;
}

#if AP_CACHE
/******************************************************************************
 * FunctionName : user_esp_platform_ap_change
 * Description  : add the user interface for changing to next ap ID.
 * Parameters   :
 * Returns      : none
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
user_esp_platform_ap_change(void)
{
    uint8 current_id;
    uint8 i = 0;
    ESP_DBG("user_esp_platform_ap_is_changing\n");

    current_id = wifi_station_get_current_ap_id();
    ESP_DBG("current ap id =%d\n", current_id);

    if (current_id == AP_CACHE_NUMBER - 1) {
       i = 0;
    } else {
       i = current_id + 1;
    }
    while (wifi_station_ap_change(i) != true) {
       i++;
       if (i == AP_CACHE_NUMBER - 1) {
    	   i = 0;
       }
    }

}
#endif

#if 0
#define gpio_io_wait    os_delay_us
LOCAL void ICACHE_FLASH_ATTR
GPIO_INTER(void)
{
	os_printf(" LVZAINA ===> SmartConfig start! \n");
	uint32 gpio_status;
	uint8 index = 0;
	gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	//clear interrupt status
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);

	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);
	while(index < 3)
	{
		os_printf(" LVZAINA ===> SmartConfig... \n");
		gpio_output_set(0, 1<<13, 1<<13, 0);
		gpio_output_set(1<<15,0 , 1<<15, 0);
		gpio_output_set(0, 1<<14, 1<<14, 0);
		gpio_io_wait(500000);
		gpio_output_set(0, 1<<15, 1<<15, 0);
		gpio_output_set(1<<13,0 , 1<<13, 0);
		gpio_output_set(1<<14,0 , 1<<14, 0);
		gpio_io_wait(500000);
		/*if( is_wificonfig_ok == 1)*/
			/*break;*/
		index++;
	}


}

LOCAL struct keys_param keys;
LOCAL struct single_key_param *single_key[1];
#endif

/******************************************************************************
 * FunctionName : user_esp_platform_init
 * Description  : device parame init based on espressif platform
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void ICACHE_FLASH_ATTR
user_esp_platform_init(void)
{

	os_sprintf(iot_version,"%s%d.%d.%dt%d(%s)",VERSION_TYPE,IOT_VERSION_MAJOR,\
	IOT_VERSION_MINOR,IOT_VERSION_REVISION,device_type,UPGRADE_FALG);
	os_printf("IOT VERSION = %s\n",iot_version);

	system_param_load(ESP_PARAM_START_SEC, 0, &esp_param, sizeof(esp_param));

	struct rst_info *rtc_info = system_get_rst_info();

	os_printf("reset reason: %x\n", rtc_info->reason);

	if (rtc_info->reason == REASON_WDT_RST ||
		rtc_info->reason == REASON_EXCEPTION_RST ||
		rtc_info->reason == REASON_SOFT_WDT_RST) {
		if (rtc_info->reason == REASON_EXCEPTION_RST) {
			os_printf("Fatal exception (%d):\n", rtc_info->exccause);
		}
		os_printf("epc1=0x%08x, epc2=0x%08x, epc3=0x%08x, excvaddr=0x%08x, depc=0x%08x\n",
				rtc_info->epc1, rtc_info->epc2, rtc_info->epc3, rtc_info->excvaddr, rtc_info->depc);
	}

	/***add by tzx for saving ip_info to avoid dhcp_client start****/
    struct dhcp_client_info dhcp_info;
    struct ip_info sta_info;
    system_rtc_mem_read(64,&dhcp_info,sizeof(struct dhcp_client_info));
	if(dhcp_info.flag == 0x01 ) {
		if (true == wifi_station_dhcpc_status())
		{
			wifi_station_dhcpc_stop();
		}
		sta_info.ip = dhcp_info.ip_addr;
		sta_info.gw = dhcp_info.gw;
		sta_info.netmask = dhcp_info.netmask;
		if ( true != wifi_set_ip_info(STATION_IF,&sta_info)) {
			os_printf("set default ip wrong\n");
		}
	}
    os_memset(&dhcp_info,0,sizeof(struct dhcp_client_info));
    system_rtc_mem_write(64,&dhcp_info,sizeof(struct rst_info));


#if AP_CACHE
    //wifi_station_ap_number_set(AP_CACHE_NUMBER);
#endif

#if 0
    {
        char sofap_mac[6] = {0x16, 0x34, 0x56, 0x78, 0x90, 0xab};
        char sta_mac[6] = {0x12, 0x34, 0x56, 0x78, 0x90, 0xab};
        struct ip_info info;

        wifi_set_macaddr(SOFTAP_IF, sofap_mac);
        wifi_set_macaddr(STATION_IF, sta_mac);

        IP4_ADDR(&info.ip, 192, 168, 3, 200);
        IP4_ADDR(&info.gw, 192, 168, 3, 1);
        IP4_ADDR(&info.netmask, 255, 255, 255, 0);
        wifi_set_ip_info(STATION_IF, &info);

        IP4_ADDR(&info.ip, 10, 10, 10, 1);
        IP4_ADDR(&info.gw, 10, 10, 10, 1);
        IP4_ADDR(&info.netmask, 255, 255, 255, 0);
        wifi_set_ip_info(SOFTAP_IF, &info);
    }
#endif

    if (esp_param.activeflag != 1) {
#ifdef SOFTAP_ENCRYPT
        struct softap_config config;
        char password[33];
        char macaddr[6];

        wifi_softap_get_config(&config);
        wifi_get_macaddr(SOFTAP_IF, macaddr);

        os_memset(config.password, 0, sizeof(config.password));
        os_sprintf(password, MACSTR "_%s", MAC2STR(macaddr), PASSWORD);
        os_memcpy(config.password, password, os_strlen(password));
        config.authmode = AUTH_WPA_WPA2_PSK;

        wifi_softap_set_config(&config);
#endif
		
		wifi_station_set_hostname( HOST_NAME );
    }

#if SENSOR_DEVICE
    user_sensor_init(esp_param.activeflag);
#endif

#if 0
    if (wifi_get_opmode() != SOFTAP_MODE) {
        os_timer_disarm(&client_timer);
        os_timer_setfn(&client_timer, (os_timer_func_t *)user_esp_platform_check_ip, 1);
        os_timer_arm(&client_timer, 100, 0);
    }
	
	// 2015-12-27 
	single_key[0] = key_init_single( 12, PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12, NULL, GPIO_INTER);
	keys.key_num = 1;
	keys.single_key = single_key;
	key_init(&keys);
#endif
}

#endif
