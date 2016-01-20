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
#include "easygpio.h"
#include "user_sensor.h"
#include "user_json.h"

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
LOCAL	os_timer_t	ping_delay_timer;
LOCAL	os_timer_t	_ping_timer;

struct	espconn		server_post_conn;	// server connection
		esp_tcp		server_post_tcp;	// server tcp connection
LOCAL	os_timer_t	post_timer;
		int			g_times = 0;
		char		g_sensor_id[ 16];
		char		json_data[ 256 ];
		char		buffer[ 1024 ];
struct jsontree_context m_json;

extern uint8_t netviom_router_connected;
extern uint8_t netviom_server_connected;

LOCAL	os_timer_t	alert_timer;
LOCAL	os_timer_t	key_timer;
rw_info rw;
rw_info m_rw;

#if NETVIOM_USE_PIR
// GPIO_PIN_INTR_NEGEDGE - down
// GPIO_PIN_INTR_POSEDGE - up
// GPIO_PIN_INTR_ANYEDGE - both
// GPIO_PIN_INTR_LOLEVEL - low level
// GPIO_PIN_INTR_HILEVEL - high level
// GPIO_PIN_INTR_DISABLE - disable interrupt
const char *gpio_type_desc[] =
{
	"GPIO_PIN_INTR_DISABLE (DISABLE INTERRUPT)",
	"GPIO_PIN_INTR_POSEDGE (               UP)",
	"GPIO_PIN_INTR_NEGEDGE (             DOWN)",
	"GPIO_PIN_INTR_ANYEDGE (             BOTH)",
	"GPIO_PIN_INTR_LOLEVEL (        LOW LEVEL)",
	"GPIO_PIN_INTR_HILEVEL (       HIGH LEVEL)"
};

LOCAL os_timer_t pir_timer;

uint32 pir_time_intr_up;
uint32 pir_time_intr_down;
uint32 pir_time_intr_interval;
uint32 pir_time_intr_first = 1;
uint8_t g_pir_stop = 1;
uint8_t m_key_level = 0;
LOCAL os_timer_t pir_server_timeout_timer;
uint32_t pir_server_timeout_pass = -1;

LOCAL void ICACHE_FLASH_ATTR pir_register_intr_up();
LOCAL void ICACHE_FLASH_ATTR pir_register_intr_down();

LOCAL void ICACHE_FLASH_ATTR pir_handler_down();

void rwinfo_setup( rw_info *prw);

#endif

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

void netviom_alert()
{
	
	if( rw.gpio_id_pir == 1)
	{
		user_beep_timer_cb_long( rw.time);
		user_alert_led_timer_cb( rw.time);
	}
}

void netviom_timeout()
{
	ETS_GPIO_INTR_DISABLE();
	if( pir_server_timeout_pass != 2)
	{
		os_printf( " |LVZAINA| ===> Server Timeout! Load old config!\n");
		netviom_alert();	
			
		pir_server_timeout_pass = 1;
		netviom_server_connected = 0;
		user_quick_status( NETVIOM_STATUS_CONNECTED);
	}
	ETS_GPIO_INTR_ENABLE();
}

void netviom_recv()
{
	ETS_GPIO_INTR_DISABLE();
	if( pir_server_timeout_pass != 1)
	{
		if( rw.time != m_rw.time)
		{
			if( m_rw.time == 0)
				user_update_status( NETVIOM_STATUS_HIDE_MODE);
			else
			{
				if( rw.time == 0 )
					user_update_status( NETVIOM_STATUS_OPEN_MODE);
			}

		}
		rwinfo_setup( &m_rw);
			
		netviom_alert();	

		pir_server_timeout_pass = 2;
	}
	ETS_GPIO_INTR_ENABLE();
}

int get_time(struct jsontree_context *js_ctx, struct jsonparse_state *parser)
{
	int type;
	uint32_t tmp = -1;
	//os_printf("========== JSON get_time ==========\n");
	//os_printf(" %d :[%s]\n", parser->pos, parser->json);
	while ((type = jsonparse_next(parser)) != 0)
	{
		if (type == JSON_TYPE_PAIR_NAME)
		{
			if (jsonparse_strcmp_value(parser, SENSOR_TYPE_ID_TIME) == 0)
			{
				jsonparse_next(parser);
				jsonparse_next(parser);
				tmp = jsonparse_get_value_as_int(parser);
				os_printf(" |LVZAINA| ===> time : %d\n", tmp);
				m_rw.time = tmp;
			}
		}

	}

	os_timer_disarm( &alert_timer);
	os_timer_setfn( &alert_timer, netviom_recv, NULL);
	os_timer_arm( &alert_timer, 2, 0); // 10ms

	return 0;
}

int get_st(struct jsontree_context *js_ctx, struct jsonparse_state *parser)
{
	int type;
	uint32_t tmp = -1;

	//os_printf("========== JSON get_st ========== \n");
	//os_printf(" %d :[%s]\n", parser->pos, parser->json);
	while ((type = jsonparse_next(parser)) != 0)
	{
		tmp = -1;
		if (type == JSON_TYPE_PAIR_NAME)
		{

			if (jsonparse_strcmp_value(parser, SENSOR_TYPE_ID_IO_0) == 0)
			{
				jsonparse_next(parser);
				jsonparse_next(parser);
				tmp = jsonparse_get_value_as_int(parser);
				os_printf(" |LVZAINA| ===> GPIO_%s : %d\n", SENSOR_TYPE_ID_IO_0, tmp);
				m_rw.gpio_id_0 = tmp;
			}
			if (jsonparse_strcmp_value(parser, SENSOR_TYPE_ID_IO_1) == 0)
			{
				jsonparse_next(parser);
				jsonparse_next(parser);
				tmp = jsonparse_get_value_as_int(parser);
				os_printf(" |LVZAINA| ===> GPIO_%s : %d\n", SENSOR_TYPE_ID_IO_1, tmp);
				m_rw.gpio_id_1 = tmp;
			}
			if (jsonparse_strcmp_value(parser, SENSOR_TYPE_ID_IO_2) == 0)
			{
				jsonparse_next(parser);
				jsonparse_next(parser);
				tmp = jsonparse_get_value_as_int(parser);
				os_printf(" |LVZAINA| ===> GPIO_%s : %d\n", SENSOR_TYPE_ID_IO_2, tmp);
				m_rw.gpio_id_2 = tmp;
			}
			if (jsonparse_strcmp_value(parser, SENSOR_TYPE_ID_PIR) == 0)
			{
				jsonparse_next(parser);
				jsonparse_next(parser);
				tmp = jsonparse_get_value_as_int(parser);
				os_printf(" |LVZAINA| ===> GPIO_%s : %d\n", SENSOR_TYPE_ID_PIR, tmp);
				m_rw.gpio_id_pir = tmp;
			}
			if (jsonparse_strcmp_value(parser, SENSOR_TYPE_ID_ALARM) == 0)
			{
				jsonparse_next(parser);
				jsonparse_next(parser);
				tmp = jsonparse_get_value_as_int(parser);
				os_printf(" |LVZAINA| ===> GPIO_%s : %d\n", SENSOR_TYPE_ID_ALARM, tmp);
				m_rw.gpio_id_alarm = tmp;
			}
		}

	}
	return 0;
}

LOCAL struct jsontree_callback st_callback =
    JSONTREE_CALLBACK( NULL, get_st);

LOCAL struct jsontree_callback time_callback =
    JSONTREE_CALLBACK( NULL, get_time);

JSONTREE_OBJECT(st_tree,
                JSONTREE_PAIR(SENSOR_TYPE_ID_IO_0, &st_callback),
                JSONTREE_PAIR(SENSOR_TYPE_ID_IO_1, &st_callback),
                JSONTREE_PAIR(SENSOR_TYPE_ID_IO_2, &st_callback),
                JSONTREE_PAIR(SENSOR_TYPE_ID_PIR, &st_callback),
                JSONTREE_PAIR(SENSOR_TYPE_ID_ALARM, &st_callback));
JSONTREE_OBJECT(netviom_tree,
                JSONTREE_PAIR( SENSOR_TYPE_ID_ST, &st_tree),
                JSONTREE_PAIR( SENSOR_TYPE_ID_TIME, &time_callback));

void post_data_received( void *arg, char *pdata, unsigned short len )
{
	struct espconn *conn = arg;
	char *vdata = strstr( pdata, "\r\n\r\n") + 4;
	if( vdata != NULL)
		os_printf( "%s: %s\n", __FUNCTION__, vdata );
	espconn_disconnect( conn );
	jsontree_setup(&m_json, (struct jsontree_value *)&netviom_tree, json_putchar);
	json_parse( &m_json, vdata);
}

void data_received( void *arg, char *pdata, unsigned short len )
{
	struct espconn *conn = arg;
	os_printf( "PING Recv: [%s]\n", pdata );
	espconn_disconnect( conn );
}

void tcp_connected_post( void *arg )
{
	
	os_memset( &m_json, 0, sizeof( struct jsontree_context));	
	os_memcpy( &m_rw, &rw, sizeof(rw_info));
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
	os_sprintf( json_data, "{\"value\":%d}", g_times );
	os_sprintf( buffer, "POST /device/%s/sensor/%s/datapoints HTTP/1.1\r\nHost: %s\r\nConnection: close\r\nContent-Type: text/xml\r\nContent-Length: %d\r\n\r\n%s",
			NETVIOM_DEVICE_ID, g_sensor_id, remote_hostname, os_strlen( json_data ), json_data);

	os_printf( " POST ===> Sending: [%s]", buffer );
	espconn_send( conn, buffer, os_strlen( buffer ) );

}

void tcp_connected_ping( void *arg )
{
	netviom_server_connected = 1;
	user_quick_status( NETVIOM_STATUS_CONNECTED);
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

	os_printf( " PING ===> Sending: [%s]", buffer );
	espconn_send( conn, buffer, os_strlen( buffer ) );

}

void tcp_error(void *arg, sint8 err) {
	os_printf("%s Error connecting: %d\n", __FUNCTION__, err);
	//wifi_station_disconnect();
	netviom_server_connected = 0;
	user_quick_status( NETVIOM_STATUS_CONNECTED);
}

void post_tcp_error(void *arg, sint8 err) {
	os_printf("%s Error connecting: %d\n", __FUNCTION__, err);
	//wifi_station_disconnect();
	netviom_server_connected = 0;
	user_quick_status( NETVIOM_STATUS_CONNECTED);
}

void tcp_disconnected( void *arg )
{
	struct espconn *conn = arg;
	/*os_printf( "%s\n", __FUNCTION__ );*/
	/*wifi_station_disconnect();*/
}

void post_tcp_disconnected( void *arg )
{
	struct espconn *conn = arg;
	/*os_printf( "%s\n", __FUNCTION__ );*/
	/*wifi_station_disconnect();*/
}

void netviom_ping()
{
	/*
	stop_pir();
	os_timer_disarm( &ping_delay_timer);
	os_timer_setfn( &ping_delay_timer, start_pir, NULL);
	os_timer_arm( &ping_delay_timer, NETVIOM_PIR_PING_DELAY_TIME, 0); // 10ms
	*/

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
	
	pir_server_timeout_pass = 0;
	os_timer_disarm( &pir_server_timeout_timer);
	os_timer_setfn( &pir_server_timeout_timer, netviom_timeout, NULL);
	os_timer_arm( &pir_server_timeout_timer, PIR_TIMEOUT_INTERVAL, 0); // 10ms

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

		os_timer_disarm( &_ping_timer);
		os_timer_setfn( &_ping_timer, netviom_ping, NULL);
		os_timer_arm( &_ping_timer, 10, 0); // 10s

		os_timer_disarm( &ping_timer);
		os_timer_setfn( &ping_timer, netviom_ping, NULL);
		os_timer_arm( &ping_timer, NETVIOM_SERVER_PING_TIME, 1); // 10s

	}
}

uint8 ICACHE_FLASH_ATTR write_cfg_flash(rw_info* prw)
{
	//写入前，需要擦除
	if (spi_flash_erase_sector(FLASH_HEAD_ADDR / (4 * 1024))
			!= SPI_FLASH_RESULT_OK) {
		os_printf("SPI FLASH ERASE ERROR\n");
		return -1;
	}
	os_printf("SPI FLASH ERASE SUCCESS\n");

	//写入
	if (spi_flash_write(FLASH_HEAD_ADDR, (uint32*) prw, sizeof(rw_info))
			!= SPI_FLASH_RESULT_OK) {
		os_printf("SPI FLASH WRITE ERROR\n");
	}
	os_printf("SPI FLASH WRITE SUCCESS\n");

	return 0;
}

bool ICACHE_FLASH_ATTR read_cfg_flash(rw_info* prw)
{
	if (spi_flash_read(FLASH_HEAD_ADDR, (uint32*) prw, sizeof(rw_info))
			!= SPI_FLASH_RESULT_OK) {
		os_printf("FLASH READ ERROR\n");
		return false;
	}
	os_printf("FLASH READ SUCCESS\n");

	return true;
}

void rwinfo_load()
{
	if( read_cfg_flash( &rw) == false || rw.key != LVZAINA_FLASH_KEY)
	{
		os_memset(&rw, 0, sizeof(rw_info));
		rw.gpio_id_0 = 1; 
		rw.gpio_id_1 = 1; 
		rw.gpio_id_2 = 1; 
		rw.gpio_id_pir = 1; 
		rw.gpio_id_alarm = 1; 
		rw.time = 10;
		rw.key = LVZAINA_FLASH_KEY; 
	}
}

void rwinfo_init( rw_info *prw, uint16_t g0, uint16_t g1, uint16_t g2, uint16_t g_pir, uint16_t g_a, uint32_t time, uint32_t key)
{
		os_memset(prw, 0, sizeof(rw_info));
		prw->gpio_id_0 = g0; 
		prw->gpio_id_1 = g1; 
		prw->gpio_id_2 = g2; 
		prw->gpio_id_pir = g_pir; 
		prw->gpio_id_alarm = g_a; 
		prw->time = time;
		prw->key = key; 
}

void rwinfo_setup( rw_info *prw)
{
	if( prw->gpio_id_0 != rw.gpio_id_0 || 
		prw->gpio_id_1 != rw.gpio_id_1 || 
		prw->gpio_id_2 != rw.gpio_id_2 || 
		prw->gpio_id_pir != rw.gpio_id_pir || 
		prw->gpio_id_alarm != rw.gpio_id_alarm || 
		prw->time != rw.time ||
		prw->key != rw.key
		)	
	{
		os_memcpy( &rw, prw, sizeof(rw_info));
		write_cfg_flash( &rw);
	}
}

void ICACHE_FLASH_ATTR
init_netviom_device()
{
		ETS_GPIO_INTR_DISABLE(); // Disable gpio interrupts
#if NETVIOM_USE_PIR
	
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT( SENSOR_PIR_IO_NUM) );
		pir_register_intr_up();
		
#endif
		// key	
        PIN_FUNC_SELECT( SENSOR_KEY_IO_MUX, SENSOR_KEY_IO_FUNC);
        gpio_output_set(0, 0, 0, GPIO_ID_PIN( SENSOR_KEY_IO_NUM));
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT( SENSOR_KEY_IO_NUM) );

        gpio_pin_intr_state_set(GPIO_ID_PIN( SENSOR_KEY_IO_NUM), GPIO_PIN_INTR_NEGEDGE);
		
		// alarm
        PIN_FUNC_SELECT( SENSOR_ALARM_IO_MUX, SENSOR_ALARM_IO_FUNC);
		GPIO_DIS_OUTPUT( SENSOR_ALARM_IO_NUM);
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT( SENSOR_ALARM_IO_NUM) );

        gpio_pin_intr_state_set(GPIO_ID_PIN( SENSOR_ALARM_IO_NUM), GPIO_PIN_INTR_LOLEVEL);

		ETS_GPIO_INTR_ENABLE(); // Enable gpio interrupts
}

void ICACHE_FLASH_ATTR
close_netviom_device()
{
	ETS_GPIO_INTR_DISABLE(); // Disable gpio interrupts
#if NETVIOM_USE_PIR
	
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT( SENSOR_PIR_IO_NUM) );
		PIN_FUNC_SELECT( SENSOR_PIR_IO_MUX, SENSOR_PIR_IO_FUNC);
		GPIO_DIS_OUTPUT( SENSOR_PIR_IO_NUM);
		PIN_PULLUP_DIS( SENSOR_PIR_IO_MUX);
        gpio_pin_intr_state_set(GPIO_ID_PIN( SENSOR_PIR_IO_NUM), GPIO_PIN_INTR_DISABLE);
		
#endif
		PIN_FUNC_SELECT( SENSOR_KEY_IO_MUX, SENSOR_KEY_IO_FUNC);
        gpio_output_set(0, 0, 0, GPIO_ID_PIN( SENSOR_KEY_IO_NUM));
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT( SENSOR_KEY_IO_NUM) );
        gpio_pin_intr_state_set(GPIO_ID_PIN( SENSOR_KEY_IO_NUM), GPIO_PIN_INTR_DISABLE);

		PIN_FUNC_SELECT( SENSOR_ALARM_IO_MUX, SENSOR_ALARM_IO_FUNC);
		GPIO_DIS_OUTPUT( SENSOR_ALARM_IO_NUM);
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT( SENSOR_ALARM_IO_NUM) );

        gpio_pin_intr_state_set(GPIO_ID_PIN( SENSOR_ALARM_IO_NUM), GPIO_PIN_INTR_DISABLE);


	ETS_GPIO_INTR_ENABLE(); // Enable gpio interrupts
}

void ICACHE_FLASH_ATTR
start_pir()
{
#if NETVIOM_USE_PIR
		os_printf( " |LVZAINA| ===> PIR Start!\n");
		
		g_pir_stop = 0;
		
#endif
}

void ICACHE_FLASH_ATTR
stop_pir()
{
#if NETVIOM_USE_PIR
		os_printf( " |LVZAINA| ===> PIR Stop!\n");
		
		g_pir_stop = 1;
		
#endif
}

/* send data to specified ip address */
void to_ip_address( )
{
	struct espconn *conn = &server_conn;
	struct espconn *post_conn = &server_post_conn;
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

	os_memcpy( post_conn->proto.tcp->remote_ip, &target->addr, 4 );

	os_timer_disarm( &_ping_timer);
	os_timer_setfn( &_ping_timer, netviom_ping, NULL);
	os_timer_arm( &_ping_timer, 10, 0); // 10s

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
			/*os_printf("ip:" IPSTR ",mask:" IPSTR ",gw:" IPSTR,
					IP2STR(&evt->event_info.got_ip.ip),
					IP2STR(&evt->event_info.got_ip.mask),
					IP2STR(&evt->event_info.got_ip.gw));
			os_printf("\n");*/
			netviom_server_connected = 0;
			netviom_router_connected = 1;
			user_quick_status( NETVIOM_STATUS_CONNECTED);
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
			netviom_router_connected = 0;
			netviom_server_connected = 0;
			user_quick_status( NETVIOM_STATUS_UNCONNECT);
			break;
	}
}

#if NETVIOM_USE_PIR

void alerting()
{
	os_printf( " |LVZAINA| ==========> alerting ============\n" );
	netviom_post( SENSOR_TYPE_ID_PIR, 1);
	/*user_beep_timer_cb( 1);*/
}

LOCAL void ICACHE_FLASH_ATTR
key_hander()
{
	if( m_key_level == 0)
	{
		os_timer_disarm( &key_timer);
		os_timer_setfn( &key_timer, (os_timer_func_t *)user_sensor_long_press, NULL);
		os_timer_arm(&key_timer, 3000, 0);

		m_key_level = 1;
		
		stop_pir();

        gpio_pin_intr_state_set(GPIO_ID_PIN( SENSOR_KEY_IO_NUM), GPIO_PIN_INTR_POSEDGE);
	}
	else
	{
		os_timer_disarm( &key_timer);
		
		m_key_level = 0;
	
		start_pir();

        gpio_pin_intr_state_set(GPIO_ID_PIN( SENSOR_KEY_IO_NUM), GPIO_PIN_INTR_NEGEDGE);
	}
}



LOCAL void ICACHE_FLASH_ATTR
pir_handler_up()
{
	uint32 gpio_status;	
	gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	os_printf( " |LVZAINA| ===> intr_handler_up: %d,(pir_off:%d)\n", gpio_status, g_pir_stop);
	ETS_GPIO_INTR_DISABLE(); // Disable gpio interrupts
	
	if( gpio_status & BIT(SENSOR_PIR_IO_NUM))
	{
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT( SENSOR_PIR_IO_NUM) );
		
		pir_time_intr_up = system_get_time();
	
		//pir_register_intr_down();
		ETS_GPIO_INTR_ATTACH( pir_handler_down, NULL); // GPIO interrupt handler
		gpio_pin_intr_state_set( GPIO_ID_PIN( SENSOR_PIR_IO_NUM), GPIO_PIN_INTR_NEGEDGE); // Interrupt on NEGATIVE edge
		
	}
	if( gpio_status & BIT(SENSOR_KEY_IO_NUM))
	{
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT( SENSOR_KEY_IO_NUM) );

		key_hander();

	}
	if( gpio_status & BIT(SENSOR_ALARM_IO_NUM))
	{
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT( SENSOR_ALARM_IO_NUM) );

		os_printf( " |LVZAINA| ===> ALARM !");

	}
	ETS_GPIO_INTR_ENABLE(); // Enable gpio interrupts
}

LOCAL void ICACHE_FLASH_ATTR
pir_register_intr_up_ssl()
{
	ETS_GPIO_INTR_DISABLE(); // Disable gpio interrupts

	ETS_GPIO_INTR_ATTACH( pir_handler_up, NULL); // GPIO interrupt handler
	PIN_FUNC_SELECT( SENSOR_PIR_IO_MUX, SENSOR_PIR_IO_FUNC);
	GPIO_DIS_OUTPUT( SENSOR_PIR_IO_NUM);
	PIN_PULLUP_DIS( SENSOR_PIR_IO_MUX);
	//GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, BIT( SENSOR_PIR_IO_NUM)); // Clear GPIO status
	gpio_pin_intr_state_set( GPIO_ID_PIN( SENSOR_PIR_IO_NUM), GPIO_PIN_INTR_POSEDGE); // Interrupt on POSETIVE edge

	ETS_GPIO_INTR_ENABLE(); // Enable gpio interrupts
}

LOCAL void ICACHE_FLASH_ATTR
pir_handler_down()
{
	uint32 gpio_status;	
	gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	os_printf( " |LVZAINA| ===> intr_handler_up: %d,(pir_off:%d)\n", gpio_status, g_pir_stop);

	ETS_GPIO_INTR_DISABLE(); // Disable gpio interrupts

	if( gpio_status & BIT(SENSOR_PIR_IO_NUM))
	{
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT( SENSOR_PIR_IO_NUM) );
		
		pir_time_intr_down = system_get_time();


		pir_time_intr_interval = pir_time_intr_down - pir_time_intr_up;
#if 1
		if( pir_time_intr_interval > PIR_TIME_INTERVAL_40M)
		{
			os_printf( " |LVZAINA| ======> %d < %d\n", PIR_TIME_INTERVAL_40M, pir_time_intr_interval);
#else
		if( pir_time_intr_interval > PIR_TIME_INTERVAL_200M)
		{
			os_printf( " |LVZAINA| ======> %d < %d\n", PIR_TIME_INTERVAL_200M, pir_time_intr_interval);
#endif
			if( g_pir_stop != 1)
				alerting();
			os_timer_disarm(&pir_timer);
			os_timer_setfn(&pir_timer, (os_timer_func_t *)pir_register_intr_up_ssl, NULL);
			os_timer_arm(&pir_timer, (rw.time*1000+PIR_DELAY_TIME), 0);	
			gpio_pin_intr_state_set(GPIO_ID_PIN( SENSOR_PIR_IO_NUM), GPIO_PIN_INTR_DISABLE);
		}
		else
		{
			os_printf( " |LVZAINA| skip ================> %d\n", pir_time_intr_interval);
			pir_register_intr_up();
		}

		//os_printf( " |LVZAINA| ==> D | [%d]\n", pir_time_intr_down);
	}
	if( gpio_status & BIT(SENSOR_KEY_IO_NUM))
	{
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT( SENSOR_KEY_IO_NUM) );

		key_hander();

	}
	if( gpio_status & BIT(SENSOR_ALARM_IO_NUM))
	{
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT( SENSOR_ALARM_IO_NUM) );

		os_printf( " |LVZAINA| ===> ALARM !");

	}
	ETS_GPIO_INTR_ENABLE(); // Enable gpio interrupts
}

LOCAL void ICACHE_FLASH_ATTR
pir_register_intr_up()
{
	//ETS_GPIO_INTR_DISABLE(); // Disable gpio interrupts

	ETS_GPIO_INTR_ATTACH( pir_handler_up, NULL); // GPIO interrupt handler
	PIN_FUNC_SELECT( SENSOR_PIR_IO_MUX, SENSOR_PIR_IO_FUNC);
	GPIO_DIS_OUTPUT( SENSOR_PIR_IO_NUM);
	PIN_PULLUP_DIS( SENSOR_PIR_IO_MUX);
	//GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, BIT( SENSOR_PIR_IO_NUM)); // Clear GPIO status
	gpio_pin_intr_state_set( GPIO_ID_PIN( SENSOR_PIR_IO_NUM), GPIO_PIN_INTR_POSEDGE); // Interrupt on POSETIVE edge

	//ETS_GPIO_INTR_ENABLE(); // Enable gpio interrupts
}
 
LOCAL void ICACHE_FLASH_ATTR
pir_register_intr_down()
{
	//ETS_GPIO_INTR_DISABLE(); // Disable gpio interrupts

//	uint32 now_time = system_get_time();

	ETS_GPIO_INTR_ATTACH( pir_handler_down, NULL); // GPIO interrupt handler
	PIN_FUNC_SELECT( SENSOR_PIR_IO_MUX, SENSOR_PIR_IO_FUNC);
	GPIO_DIS_OUTPUT( SENSOR_PIR_IO_NUM);
	PIN_PULLUP_DIS( SENSOR_PIR_IO_MUX);
	//GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, BIT( SENSOR_PIR_IO_NUM)); // Clear GPIO status
	gpio_pin_intr_state_set( GPIO_ID_PIN( SENSOR_PIR_IO_NUM), GPIO_PIN_INTR_NEGEDGE); // Interrupt on NEGATIVE edge

	//ETS_GPIO_INTR_ENABLE(); // Enable gpio interrupts
}

#endif

#ifdef TEST_POLLING

#define SAMPLE_PERIOD 250 // 750 ms
static os_timer_t dht22_timer;
uint8_t pinsToTest[] = {5};
uint8_t pinsToTestLen = 1;

static void ICACHE_FLASH_ATTR
loop(void) {
  uint8_t i=0;
  for (i=0; i<pinsToTestLen; i++) {
    easygpio_outputDisable(pinsToTest[i]);
    os_printf("GPIO%d=>%d", pinsToTest[i], easygpio_inputGet(pinsToTest[i]));
    if(i<pinsToTestLen-1) {
      os_printf(", ");
    }
  }
  os_printf("\n");
}

void ICACHE_FLASH_ATTR
setup(void) {
  uint8_t i=0;
  for (i=0; i<pinsToTestLen; i++) {
    os_printf("Setting gpio%d as input\n", pinsToTest[i]);
    easygpio_pinMode(pinsToTest[i], EASYGPIO_NOPULL, EASYGPIO_INPUT);
  }
  os_printf("Starting to test the gpio pin values:\n");
  os_timer_disarm(&dht22_timer);
  os_timer_setfn(&dht22_timer, (os_timer_func_t *)loop, NULL);
  os_timer_arm(&dht22_timer, SAMPLE_PERIOD, true);
}

#endif

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
    os_printf("LVZIANA ===> Start! \n");
    os_printf("LVZIANA ========> Netviom ID:  |%s| \n", NETVIOM_DEVICE_ID);
		
	show_sysinfo();

	rwinfo_load();
	if( rw.time == 0)
		user_update_status( NETVIOM_STATUS_HIDE_MODE);

	gpio_init();

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

#ifdef TEST_POLLING
	os_timer_disarm(&dht22_timer);
	os_timer_setfn(&dht22_timer, (os_timer_func_t *)setup, NULL);
	os_timer_arm(&dht22_timer, 2000, false);
#endif

}

