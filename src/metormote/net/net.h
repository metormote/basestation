/*
 * net.h
 *
 * Created: 1/5/2012 10:18:19 PM
 *  Author: Administrator
 */ 
#ifndef NET_H_
#define NET_H_

#include <stdint.h>
#include <stddef.h>
#include "mem.h"
#include "w5200.h"
#include "atom.h"
#include "atomqueue.h"
#include "atommutex.h"
#include "atomport.h"
#include "net_core.h"
#include "dhcpc.h"

#define NET_CLIENT_SOCKET         0
#define NET_DHCPC_SOCKET          0
#define NET_DNS_SOCKET            0

#define NET_RECEIVE_TIMEOUT       10*SYSTEM_TICKS_PER_SEC

#ifdef __cplusplus
extern "C" { 
#endif


struct net_options_t {
  uint8_t mac[6];
  uint8_t ip[4];
  uint8_t mask[4];
  uint8_t gw[4];
  uint8_t dns[2][4];
  uint8_t dhcp_on;
  uint32_t dhcp_lease_end;
};

struct net_client_socket_options_t {
  uint8_t  ip[4];
  uint16_t port;
  uint16_t timeout;
};

struct net_server_socket_options_t {
  uint16_t port;
  void (*on_request)(uint8_t *msg, uint16_t len);
};


void net_init(void);
void net_reset(void);
uint8_t net_link_ok(void);
int8_t net_config(struct net_options_t *options);
int8_t net_start_dhcp(struct net_options_t *options);
int8_t net_connect(uint8_t socket, struct net_client_socket_options_t *client);
int8_t net_listen(uint8_t socket, struct net_server_socket_options_t *server);
int8_t net_close(uint8_t socket);
int8_t net_send(uint8_t socket, uint8_t *src, uint16_t len);
int8_t net_receive(uint8_t socket, uint8_t *dst, uint16_t len, int32_t timeout);
void net_clear_buffer(uint8_t s);

#ifdef __cplusplus
}
#endif

#endif /* NET_H_ */