/*
 * dhcpc.h
 *
 * Created: 12/26/2011 11:19:21 AM
 *  Author: Administrator
 */ 
#ifndef DHCPC_H_
#define DHCPC_H_

#include "status_codes.h"
#include "w5200.h"
#include "rtc.h"
#include "io_stream.h"

#define DHCPC_STATE_INIT          1
#define DHCPC_STATE_SELECTING     2
#define DHCPC_STATE_REQUESTING    3
#define DHCPC_STATE_BOUND         4
#define DHCPC_STATE_RENEWING      5
#define DHCPC_STATE_REBINDING     6
#define DHCPC_STATE_INITREBOOT    7
#define DHCPC_STATE_REBOOTING     8


#define DHCPC_CLIENT_SRC_PORT_HIGH    0xe0 
#define DHCPC_SRC_PORT                68
#define DHCPC_DEST_PORT               67

#define DHCPC_TIMEOUT               10*SYSTEM_TICKS_PER_SEC



#define DHCP_BOOTREQUEST    1
#define DHCP_BOOTREPLY      2
#define DHCPDISCOVER        0x01
#define DHCPOFFER           0x02
#define DHCPREQUEST         0x03
#define DHCPACK             0x05
//#define DHCPNACK

#define DHCPC_OPTION_MSGTYPE          53
#define DHCPC_OPTION_CLIENT_ID        61
#define DHCPC_OPTION_HOST_NAME        12
#define DHCPC_OPTION_VENDOR_CLASS_ID  60
#define DHCPC_OPTION_REQUESTED_IP     50
#define DHCPC_OPTION_SERVER_IP        54
#define DHCPC_OPTION_PARAM_LIST       55
#define DHCPC_OPTION_SUBNET_MASK      1
#define DHCPC_OPTION_GATEWAY          3
#define DHCPC_OPTION_DNS_SERVER       6
#define DHCPC_OPTION_LEASE_TIME       51
#define DHCPC_OPTION_END              255



#ifdef __cplusplus
extern "C" { 
#endif

struct dhcp_data {
  // 0-3 op, htype, hlen, hops
  uint8_t op;
  uint8_t htype;
  uint8_t hlen;
  uint8_t hops;
  // 4-7 xid
  uint32_t xid;
  // 8-9 secs
  uint16_t secs;
  // 10-11 flags
  uint16_t flags;
  // 12-15 ciaddr
  uint8_t ciaddr[4];
  // 16-19 yiaddr
  uint8_t yiaddr[4];
  // 20-23 siaddr
  uint8_t siaddr[4];
  // 24-27 giaddr
  uint8_t giaddr[4];
  // 28-43 chaddr(16)
  uint8_t chaddr[16];
  // 44-107 sname (64)
  uint8_t sname[64];
  // 108-235 file(128)
  uint8_t file[128];
  // options (variable size)
  //uint8_t *options;
};


struct dhcp_config {
  uint8_t mac[6];
  uint8_t ip[4];
  uint8_t mask[4];
  uint8_t server[4];
  uint16_t server_port;
  uint8_t dns[4];
  uint8_t gateway[4];
  uint32_t lease_start;
  uint32_t lease_time;
};

int8_t dhcpc_run(SOCKET socket, struct dhcp_config *conf, uint8_t *buf, uint16_t buf_len);
int8_t dhcpc_renew(void);
  
#ifdef __cplusplus
}
#endif

#endif /* DHCPC_H_ */