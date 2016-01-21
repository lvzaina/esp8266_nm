#include "user_server.h"
#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include "espconn.h"
#include "user_netviom.h"
#include "mem.h"

#define packet_size   (2 * 1024)
static struct espconn tcp_server;
static esp_tcp tcp_config;

static void ICACHE_FLASH_ATTR
tcpserver_recv_callback(void *arg, char *pdata, uint16_t length)
{
	char * pos = NULL;
	os_printf( "|LVZAINA| ===> Config Client:[%s]", pdata);
	pos = strstr( pdata, NETVIOM_SETTINGS_CMD);
    if( index != NULL)
    {
		pos = pos+(os_strlen(NETVIOM_SETTINGS_CMD))+1;
		os_printf( "|LVZAINA| ===> %s:[%s]", NETVIOM_SETTINGS_CMD, pos);
		set_netviom_id( pos);
    }
}

static void ICACHE_FLASH_ATTR
tcpserver_disconnect_callback(void *arg)
{
    os_printf("Client disconnected from the server. link_cnt=%d.\r\n", tcp_server.link_cnt);
}

static void ICACHE_FLASH_ATTR
tcpserver_reconnect_callback(void *arg, int8_t err)
{ 
    os_printf("Connection terminated unexpectedly, error code: %d.\r\n", err);
}

LOCAL void ICACHE_FLASH_ATTR
user_sent_data(struct espconn *pespconn)
{
	char *pbuf = (char *)os_zalloc(packet_size);
	os_sprintf( pbuf, "Netviom!\nsystem_get_time: %d\nsystem_get_rtc_time: %d\nsystem_rtc_clock_cali_proc: %d\n", system_get_time(), system_get_rtc_time(), system_rtc_clock_cali_proc());
	
	espconn_send(pespconn, pbuf, os_strlen(pbuf));

	os_free(pbuf);

}

static void ICACHE_FLASH_ATTR
incoming_connection_callback(void *arg)
{
    struct espconn *pconnection = arg;
    remot_info *premot = NULL;
    uint8_t idx;
    int8_t value;

    espconn_regist_recvcb(pconnection, tcpserver_recv_callback);
    espconn_regist_reconcb(pconnection, tcpserver_reconnect_callback);
    espconn_regist_disconcb(pconnection, tcpserver_disconnect_callback);

    value = espconn_get_connection_info(&tcp_server, &premot, 0);
    os_printf("Incoming TCP connection. link_cnt=%d. Get connnection info result:[%d]\r\n", 
            tcp_server.link_cnt, value);
	user_sent_data( pconnection);
    if (value == ESPCONN_OK)
    {
        for (idx = 0; idx < tcp_server.link_cnt; idx++)
        {
            os_printf("TCP client %d: %d.%d.%d.%d:%d\r\n",
                    idx + 1,
                    premot[idx].remote_ip[0],
                    premot[idx].remote_ip[1],
                    premot[idx].remote_ip[2],
                    premot[idx].remote_ip[3],
                    premot[idx].remote_port);
        }
    }

    os_printf("\r\n");
}

void ICACHE_FLASH_ATTR
tcpserver_init(void)
{
    int8_t res;

    tcp_server.type = ESPCONN_TCP;
    tcp_server.state = ESPCONN_NONE;
    tcp_server.proto.tcp = &tcp_config;
    tcp_server.proto.tcp->local_port = NETVIOM_SETTINGS_PORT;

    espconn_regist_connectcb(&tcp_server, incoming_connection_callback);

    espconn_tcp_set_max_con_allow(&tcp_server, 1); //Allow 1 connection max

    res = espconn_accept(&tcp_server);

    if(res == 0)
        os_printf("Created TCP server on port %d, running on ROM %d, AP RSSI: %ddBm, WiFi mode: %d.\r\n", 
                tcp_server.proto.tcp->local_port,
                system_upgrade_userbin_check(),
                wifi_station_get_rssi(),
                wifi_get_phy_mode());
    else
        os_printf("Failed to create TCP server, error code: %d.\r\n", res);
}
