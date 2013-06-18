/*
 * net.c
 *
 * Created: 3/2/2012 2:24:33 PM
 *  Author: Administrator
 */ 
#include "net.h"

void net_init() {
  w5200_init();
}

void net_reset() {
  w5200_reset();
}

int8_t net_config(struct net_options_t *options) {
  
  if(options->mac != NULL) {
    w5200_set_mac(options->mac);
  }
  
  if(options->ip != NULL) {
    w5200_set_source_ip(options->ip);
  }
  
  if(options->mask != NULL) {
    w5200_set_subnet(options->mask);
  }
  
  if(options->gw != NULL) {
    w5200_set_gateway(options->gw);
  }
  
  return STATUS_OK;
}


uint8_t net_link_ok() {
  return w5200_get_link_state();
}


int8_t net_start_dhcp(struct net_options_t *options) {
  uint8_t i;
  uint8_t *buf;
  int8_t status;
  struct dhcp_config dhcp;
  
  if(!w5200_get_link_state()) {
      return ERR_IO_ERROR;
  }
  
  memset(&dhcp, 0, sizeof(struct dhcp_config));
  for(i=0;i<4;i++) {
    dhcp.ip[i] = 0x00;
    dhcp.server[i] = 0xFF;
  }
  
  w5200_get_mac(dhcp.mac);
  
  buf = mem_safe_calloc(sizeof(uint8_t), 300);
  if(buf == NULL) return ERR_NO_MEMORY;
  
  status = dhcpc_run(NET_DHCPC_SOCKET, &dhcp, buf, 300);
  
  mem_safe_free(buf);
  
  if(status==STATUS_OK) {
    options->dhcp_on=TRUE;
    options->dhcp_lease_end=dhcp.lease_start+dhcp.lease_time;
    memcpy(options->ip, dhcp.ip, 4);
    memcpy(options->mask, dhcp.mask, 4);
    memcpy(options->gw, dhcp.gateway, 4);
    memcpy(options->dns[0], dhcp.dns, 4);
    
    w5200_set_gateway(dhcp.gateway);
    w5200_set_source_ip(dhcp.ip);
    w5200_set_subnet(dhcp.mask);
  }
  return status;
}


int8_t net_connect(uint8_t s, struct net_client_socket_options_t *client) {
  int8_t status;
  
  if(!w5200_get_link_state()) {
      return ERR_IO_ERROR;
  }
  
  if(w5200_get_socket_status(s)==W5200_SOCK_CLOSED) {
    w5200_sockets[s].protocol=W5200_Sn_MR_TCP;
    w5200_sockets[s].port=0;
    w5200_sockets[s].flag=0x00;
    if((status=w5200_socket(s))!=STATUS_OK) {
      return status;
    }
  }
  
  if((status=w5200_connect(s, client->ip, client->port))!=STATUS_OK) {
    return status;
  }
  
  return STATUS_OK;
}


int8_t net_listen(uint8_t s, struct net_server_socket_options_t *server) {
  int8_t status;
  
  if(!w5200_get_link_state()) {
      return ERR_IO_ERROR;
  }
  
  if(w5200_get_socket_status(s)==W5200_SOCK_CLOSED) {
    w5200_sockets[s].protocol=W5200_Sn_MR_TCP;
    w5200_sockets[s].port=server->port;
    w5200_sockets[s].flag=0x00;
    if((status=w5200_socket(s))!=STATUS_OK) {
      return status;
    }
  }
    
  if((status=w5200_listen(s))!=STATUS_OK) {
    return status;
  }
  return STATUS_OK;
}


int8_t net_close(uint8_t s) {
  w5200_close(s);
  return STATUS_OK;
}

void net_clear_buffer(uint8_t s) {
  uint16_t len;
  len=w5200_get_rx_buf_len(s);
  w5200_receive_tcp(s, NULL, len);
}

int8_t net_receive(uint8_t s, uint8_t *buf, uint16_t len, int32_t timeout) {
  int8_t status;
  io_output_stream_t dst;
  
  if(w5200_wait_for_data(s, len, timeout)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  
  dst=io_output_stream_from_buffer(buf, len);
  
  status=w5200_receive_tcp(s, &dst, len);
  
  return status;
}  


int8_t net_send(uint8_t s, uint8_t *buf, uint16_t len) {
  int8_t status;
  io_input_stream_t src;
  
  if(w5200_get_socket_status(s)==W5200_SOCK_CLOSED) {
    return ERR_IO_ERROR;
  }
  
  src=io_input_stream_from_buffer(buf, len);
  
  status=w5200_send_tcp(s, &src, len);
  
  return status;
}  
