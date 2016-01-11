#ifndef __USER_NETVIOM_H__
#define __USER_NETVIOM_H__

#define NETVIOM_DEVICE_ID		"700100000011"


#define USE_DNS					true // if true, device will try to resolve REMOTE_HOST, otherwise REMOTE_IP will be used.
#define REMOTE_PORT				10068 // server port
#define REMOTE_HOST				"alert.netviom.com" // only if USE_DNS true
#define REMOTE_IP				"192.168.2.23" // only used if USE_DNS false

#define HOST_NAME				"netviom_PIR" // name of this device

#define SENSOR_TYPE_ID_IO_0		"100" // IO 1
#define SENSOR_TYPE_ID_IO_1		"101" // IO 2
#define SENSOR_TYPE_ID_IO_2		"102" // IO 3
#define SENSOR_TYPE_ID_PIR		"200" // 红外
#define SENSOR_TYPE_ID_ALARM	"300" // 紧急传感器

#endif

