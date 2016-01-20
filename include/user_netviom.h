#ifndef __USER_NETVIOM_H__
#define __USER_NETVIOM_H__

#include "c_types.h"
#define NETVIOM_DEVICE_ID			"700100000046"

#define NETVIOM_SERVER_PING_TIME	20*1000 // ms, 保活间隔时长
#define NETVIOM_PIR_PING_DELAY_TIME	2500 // ms, 保活间隔时长

#define USE_DNS					true // if true, device will try to resolve REMOTE_HOST, otherwise REMOTE_IP will be used.

#define NETVIOM_USE_PIR			true 

#if NETVIOM_USE_PIR
	#define PIR_TIME_INTERVAL_40M	400*1000 // us ,PIR 报警脉宽
	#define PIR_TIME_INTERVAL_200M	2000*1000 // us ,PIR 报警脉宽
	#define PIR_START_TIME			45*1000 // ms ,开机延时
	#define PIR_CONFIG_START_TIME	5*1000 // ms ,配置完成PIR启动延时
	#define PIR_DELAY_TIME			3*1000 // ms ,每次报警间隔延时
	#define PIR_TIMEOUT_INTERVAL	500 // ms ,报警触法,服务端回应超时时长
#endif


#define REMOTE_PORT				10068 // server port
#define REMOTE_HOST				"alert.netviom.com" // only if USE_DNS true
#define REMOTE_IP				"192.168.2.23" // only used if USE_DNS false

#define HOST_NAME				"netviom_PIR" // name of this device

#define SENSOR_TYPE_ID_ST		"st" // 
#define SENSOR_TYPE_ID_IO_0		"100" // IO 1
#define SENSOR_TYPE_ID_IO_1		"101" // IO 2
#define SENSOR_TYPE_ID_IO_2		"102" // IO 3
#define SENSOR_TYPE_ID_PIR		"200" // 红外
#define SENSOR_TYPE_ID_ALARM	"300" // 紧急传感器
#define SENSOR_TYPE_ID_TIME		"tm" // 

// flicker frequency
#define FLICKER_FREQUENCY_FAST 250 
#define FLICKER_FREQUENCY_SLOW 500 

typedef enum {
  NETVIOM_STATUS_POWERUP=0,
  NETVIOM_STATUS_INIT,
  NETVIOM_STATUS_CONFIG,
  NETVIOM_STATUS_CONFIG_COMPLETED,
  NETVIOM_STATUS_CONNECTING,
  NETVIOM_STATUS_UNCONNECT,
  NETVIOM_STATUS_CONNECTED,
  NETVIOM_STATUS_HIDE_MODE,
  NETVIOM_STATUS_OPEN_MODE
} NETVIOM_STATUS_VALUE;

void ICACHE_FLASH_ATTR user_update_status( NETVIOM_STATUS_VALUE state); 

typedef struct _rw_info{
	uint32_t gpio_id_0;
	uint32_t gpio_id_1;
	uint32_t gpio_id_2;
	uint32_t gpio_id_pir;
	uint32_t gpio_id_alarm;
	uint32_t time;
	uint32_t key;
} rw_info;

#define FLASH_HEAD_ADDR 0x3C000

#define LVZAINA_FLASH_KEY 0x99

#endif

