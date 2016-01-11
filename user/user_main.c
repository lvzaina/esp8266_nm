/******************************************************************************
 * Copyright 2013-2014 Espressif Systems (Wuxi)
 *
 * FileName: user_main.c
 *
 * Description: entry file of user application
 *
 * Modification history:
 *     2014/1/1, v1.0 create this file.
*******************************************************************************/
#include "ets_sys.h"
#include "osapi.h"

#include "os_type.h"
#include "mem.h"
#include "ip_addr.h"
#include "espconn.h"
#include "user_interface.h"

#include "user_devicefind.h"
#include "user_webserver.h"
#include "user_netviom.h"

#if ESP_PLATFORM
#include "user_esp_platform.h"
#endif

#ifdef SERVER_SSL_ENABLE
#include "ssl/cert.h"
#include "ssl/private_key.h"
#else
#ifdef CLIENT_SSL_ENABLE
unsigned char *default_certificate;
unsigned int default_certificate_len = 0;
unsigned char *default_private_key;
unsigned int default_private_key_len = 0;
#endif
#endif

struct	espconn		server_conn;		// server connection
		ip_addr_t	server_ip;			// server ip address
		esp_tcp		server_tcp;			// server tcp connection
LOCAL	os_timer_t	ping_timer;

struct	espconn		server_post_conn;	// server connection
		ip_addr_t	serveri_post_ip;	// server ip address
		esp_tcp		server_post_tcp;	// server tcp connection
LOCAL	os_timer_t	post_timer;
		int			g_times = 0;
		char		g_sensor_id[ 16];
		char		json_data[ 256 ];
		char		buffer[ 1024 ];

void user_rf_pre_init(void)
{
}

void ICACHE_FLASH_ATTR show_sysinfo()
{
	uint32 chipid = system_get_chip_id();
	uint32 heepsize = system_get_free_heap_size();
	uint32 rtctime = system_get_rtc_time();
	uint32 systime = system_get_time();

	os_printf("\n\nSDK version: [%s]\n", system_get_sdk_version());

	os_printf("SYSTEM INIT OVER\n");
	os_printf("==========SYS INFO==========\n");
	system_print_meminfo();
	os_printf("CHIP   ID: [%d]\n", chipid);
	os_printf("HEAP SIZE: [%d]\n", heepsize);
	os_printf("RTC  TIME: [%d]\n", rtctime);
	os_printf("SYS  TIME: [%d]\n", systime);
	os_printf("==========SYS INFO==========\n");
}

void post_data_received( void *arg, char *pdata, unsigned short len )
{
	struct espconn *conn = arg;
	os_printf( "%s: %s\n", __FUNCTION__, pdata );
	espconn_disconnect( conn );
}

void data_received( void *arg, char *pdata, unsigned short len )
{
	struct espconn *conn = arg;
	os_printf( "%s: %s\n", __FUNCTION__, pdata );
	espconn_disconnect( conn );
}

void tcp_connected_post( void *arg )
{
	
	//os_printf( "%s\n", __FUNCTION__ );

	/*struct sensor_result sr;*/
	/*read_all(&sr);*/
	/*os_sprintf( dt, "Temperature: %s\n", f2s(sr.temperature,2));*/
	/*os_sprintf( dh, "Humidity: %s\n", f2s(sr.humidity,2));*/
	/*os_sprintf( dp, "Pressure: %s\n", f2s(sr.pressure,2));*/

	struct espconn *conn = arg;

	espconn_regist_recvcb( conn, post_data_received );

	char *remote_hostname;
#if USE_DNS
	remote_hostname = REMOTE_HOST;
#else
	remote_hostname = REMOTE_IP;
#endif
	os_sprintf( json_data, "{\"value\": \"%d\"}", times );
	os_sprintf( buffer, "POST /device/%s/sensor/%s/datapoints HTTP/1.1\r\nHost: %s\r\nConnection: close\r\nContent-Type: text/xml\r\nContent-Length: %d\r\n\r\n%s",
			NETVIOM_DEVICE_ID, g_sensor_id, remote_hostname, os_strlen( json_data ), json_data);

	//os_printf( " PING ===> Sending: [\n%s\n]", buffer );
	espconn_send( conn, buffer, os_strlen( buffer ) );

}

void tcp_connected_ping( void *arg )
{
	char buffer[ 1024 ];
	//os_printf( "%s\n", __FUNCTION__ );

	/*struct sensor_result sr;*/
	/*read_all(&sr);*/
	/*os_sprintf( dt, "Temperature: %s\n", f2s(sr.temperature,2));*/
	/*os_sprintf( dh, "Humidity: %s\n", f2s(sr.humidity,2));*/
	/*os_sprintf( dp, "Pressure: %s\n", f2s(sr.pressure,2));*/

	struct espconn *conn = arg;

	espconn_regist_recvcb( conn, data_received );

	char *remote_hostname;
#if USE_DNS
	remote_hostname = REMOTE_HOST;
#else
	remote_hostname = REMOTE_IP;
#endif
	
	os_sprintf( buffer, "POST /device/%s/sensor/000/datapoints HTTP/1.1\r\nHost: %s\r\nConnection: close\r\nContent-Type: text/xml\r\nContent-Length: 0\r\n\r\n",
			NETVIOM_DEVICE_ID, remote_hostname);

	//os_printf( " PING ===> Sending: [\n%s\n]", buffer );
	espconn_send( conn, buffer, os_strlen( buffer ) );

}

void tcp_error(void *arg, sint8 err) {
	os_printf("%s Error connecting: %d\n", __FUNCTION__, err);
	//wifi_station_disconnect();
}

void post_tcp_error(void *arg, sint8 err) {
	os_printf("%s Error connecting: %d\n", __FUNCTION__, err);
	//wifi_station_disconnect();
}

void tcp_disconnected( void *arg )
{
	struct espconn *conn = arg;
	os_printf( "%s\n", __FUNCTION__ );
	/*wifi_station_disconnect();*/
}

void post_tcp_disconnected( void *arg )
{
	struct espconn *conn = arg;
	os_printf( "%s\n", __FUNCTION__ );
	/*wifi_station_disconnect();*/
}

void netviom_ping()
{
	espconn_regist_connectcb( &server_conn, tcp_connected_ping);
	espconn_regist_disconcb( &server_conn, tcp_disconnected);
	espconn_regist_reconcb( &server_conn, tcp_error);

	espconn_connect( &server_conn);
}

void netviom_post( const char *sensor_id, int times)
{

	os_sprintf( g_sensor_id, "%s", sensor_id);
	g_times = times; 

	espconn_regist_connectcb( &server_post_conn, tcp_connected_post);
	espconn_regist_disconcb( &server_post_conn, post_tcp_disconnected);
	espconn_regist_reconcb( &server_post_conn, post_tcp_error);

	espconn_connect( &server_post_conn);

}

void dns_done( const char *name, ip_addr_t *ipaddr, void *arg )
{
	struct espconn *conn = &server_conn;
	struct espconn *post_conn = &server_post_conn;

	os_printf( "%s\n", __FUNCTION__ );

	if ( ipaddr == NULL)
	{
		os_printf("DNS lookup failed\n");
		wifi_station_disconnect();
	}
	else
	{
		os_printf("Connecting... %d.%d.%d.%d\n",
			*((uint8 *)&ipaddr->addr), *((uint8 *)&ipaddr->addr + 1),
			*((uint8 *)&ipaddr->addr + 2), *((uint8 *)&ipaddr->addr + 3));

		conn->type = ESPCONN_TCP;
		conn->state = ESPCONN_NONE;
		conn->proto.tcp=&server_tcp;
		conn->proto.tcp->local_port = espconn_port();
		conn->proto.tcp->remote_port = REMOTE_PORT;
		os_memcpy( conn->proto.tcp->remote_ip, &ipaddr->addr, 4 );

		post_conn->type = ESPCONN_TCP;
		post_conn->state = ESPCONN_NONE;
		post_conn->proto.tcp=&server_post_tcp;
		post_conn->proto.tcp->local_port = espconn_port();
		post_conn->proto.tcp->remote_port = REMOTE_PORT;
		os_memcpy( post_conn->proto.tcp->remote_ip, &ipaddr->addr, 4 );

		os_timer_disarm( &ping_timer);
		os_timer_setfn( &ping_timer, netviom_ping, NULL);
		os_timer_arm( &ping_timer, 10000, 1); // 10s
	}
}

/* send data to specified ip address */
void to_ip_address( )
{
	struct espconn *conn = &server_conn;
	ipaddr_aton(REMOTE_IP, &server_ip);
	ip_addr_t *target = &server_ip;

	os_printf( "%s\n", __FUNCTION__ );

	os_printf("Connecting to IP address %s...\n", REMOTE_IP );

	conn->type = ESPCONN_TCP;
	conn->state = ESPCONN_NONE;
	conn->proto.tcp=&server_tcp;
	conn->proto.tcp->local_port = espconn_port();
	conn->proto.tcp->remote_port = REMOTE_PORT;

	os_memcpy( conn->proto.tcp->remote_ip, &target->addr, 4 );

	post_conn->type = ESPCONN_TCP;
	post_conn->state = ESPCONN_NONE;
	post_conn->proto.tcp=&server_post_tcp;
	post_conn->proto.tcp->local_port = espconn_port();
	post_conn->proto.tcp->remote_port = REMOTE_PORT;

	os_memcpy( post_conn->proto.tcp->remote_ip, &ipaddr->addr, 4 );


	os_timer_disarm( &ping_timer);
	os_timer_setfn( &ping_timer, netviom_ping, NULL);
	os_timer_arm( &ping_timer, 10000, 1); // 10s

}


void wifi_handle_event_cb(System_Event_t *evt)
{
	os_printf("event %x\n", evt->event);
	switch (evt->event) {
		case EVENT_STAMODE_CONNECTED:
			os_printf("connect to ssid %s, channel %d\n",
					evt->event_info.connected.ssid,
					evt->event_info.connected.channel);
			break;
		case EVENT_STAMODE_DISCONNECTED:
			os_printf("disconnect from ssid %s, reason %d\n",
					evt->event_info.disconnected.ssid,
					evt->event_info.disconnected.reason);
			break;
		case EVENT_STAMODE_AUTHMODE_CHANGE:
			os_printf("mode: %d -> %d\n",
					evt->event_info.auth_change.old_mode,
					evt->event_info.auth_change.new_mode);
			break;
		case EVENT_STAMODE_GOT_IP:
			os_printf("ip:" IPSTR ",mask:" IPSTR ",gw:" IPSTR,
					IP2STR(&evt->event_info.got_ip.ip),
					IP2STR(&evt->event_info.got_ip.mask),
					IP2STR(&evt->event_info.got_ip.gw));
			os_printf("\n");
#if USE_DNS == true
			espconn_gethostbyname( &server_conn, REMOTE_HOST, &server_ip, dns_done );
#else
			to_ip_address();
#endif
			break;
		case EVENT_SOFTAPMODE_STACONNECTED:
			os_printf("station: " MACSTR "join, AID = %d\n",
					MAC2STR(evt->event_info.sta_connected.mac),
					evt->event_info.sta_connected.aid);
			break;
		case EVENT_SOFTAPMODE_STADISCONNECTED:
			os_printf("station: " MACSTR "leave, AID = %d\n",
					MAC2STR(evt->event_info.sta_disconnected.mac),
					evt->event_info.sta_disconnected.aid);
			break;
		default:
			break;
	}
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
    os_printf("LVZIANA ===> Start!\n");
		
	show_sysinfo();

#if ESP_PLATFORM
    /*Initialization of the peripheral drivers*/
    /*For light demo , it is user_light_init();*/
    /* Also check whether assigned ip addr by the router.If so, connect to ESP-server  */
    user_esp_platform_init();
#endif
    /*Establish a udp socket to receive local device detect info.*/
    /*Listen to the port 1025, as well as udp broadcast.*/
    /*If receive a string of device_find_request, it rely its IP address and MAC.*/
    user_devicefind_init();
	
	wifi_set_event_handler_cb( wifi_handle_event_cb );

#if 0
    /*Establish a TCP server for http(with JSON) POST or GET command to communicate with the device.*/
    /*You can find the command in "2B-SDK-Espressif IoT Demo.pdf" to see the details.*/
    /*the JSON command for curl is like:*/
    /*3 Channel mode: curl -X POST -H "Content-Type:application/json" -d "{\"period\":1000,\"rgb\":{\"red\":16000,\"green\":16000,\"blue\":16000}}" http://192.168.4.1/config?command=light      */
    /*5 Channel mode: curl -X POST -H "Content-Type:application/json" -d "{\"period\":1000,\"rgb\":{\"red\":16000,\"green\":16000,\"blue\":16000,\"cwhite\":3000,\"wwhite\",3000}}" http://192.168.4.1/config?command=light      */
#ifdef SERVER_SSL_ENABLE
    user_webserver_init(SERVER_SSL_PORT);
#else
    user_webserver_init(SERVER_PORT);
#endif
#endif
}

