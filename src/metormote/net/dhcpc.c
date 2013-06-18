/*
 * dhcpc.c
 *
 * Created: 12/26/2011 11:19:34 AM
 * Author: Administrator
 */ 
#include "dhcpc.h"

static uint8_t BROADCAST_ADDRESS[4]={0xFF, 0xFF, 0xFF, 0xFF};

//dhcp state
static uint8_t dhcpc_state;

//transaction id
static uint8_t dhcpc_xid=0;

static struct dhcp_config *config;
static char hostname[] = "metormote-AA";

static uint8_t have_answer=0;
static uint8_t dhcp_error=0;
static uint32_t current_xid = 0;
static uint16_t current_time = 0;

static uint8_t* bufPtr;

static int8_t dhcpc_start(SOCKET s, struct dhcp_config *conf, uint8_t *buf);
static int8_t dhcpc_send(SOCKET s, uint8_t *buf, uint8_t type );
static int8_t dhcpc_process(SOCKET s, uint8_t *buf, uint16_t plen);

static int8_t process_dhcpoffer (SOCKET s, uint8_t *buf, uint16_t plen);
static int8_t process_dhcpack (void);

static void append(uint8_t b) {
    *bufPtr++ = b;
}


int8_t dhcpc_renew(void)
{
  // Check lease and request renew if currently OK and time
  // lease_start - start time
  // lease_time - length of lease
  //
  if( dhcpc_state == DHCPC_STATE_BOUND && (config->lease_start + config->lease_time) <= rtc_get_time() ) {
    // Calling app needs to detect this and init renewal
    dhcpc_state = DHCPC_STATE_RENEWING;
  }
  return(dhcpc_state);
}

// Start request sequence, send DHCPDISCOVER
// Wait for DHCPOFFER
// Send DHCPREQUEST
// Wait for DHCPACK
// All configured
static int8_t dhcpc_start(SOCKET s, struct dhcp_config *conf, uint8_t *buf)
{
  config = conf;
  current_xid = 0x00654321 + random();
  current_time = 0;
  // Set a unique hostname, use 'metormote-' plus last octet of mac address
  hostname[10] = 'A' + (config->mac[5] >> 4);
  hostname[11] = 'A' + (config->mac[5] & 0x0F);
  
  dhcpc_state = DHCPC_STATE_INIT;
  return dhcpc_send(s, buf, DHCPDISCOVER);
}



// Main DHCP client message sending function, either DHCPDISCOVER or DHCPREQUEST
static int8_t dhcpc_send(SOCKET s, uint8_t *buf, uint8_t type ) {
  uint8_t i=0;
  uint16_t len;
  io_input_stream_t src;
  
  have_answer=0;
  dhcp_error=0;
  dhcpc_xid++; // increment for next request, finally wrap
  
  // Build DHCP Packet from buf[UDP_DATA_P]
  // Make dhcpPtr start of UDP data buffer
  struct dhcp_data *dhcpPtr = (struct dhcp_data *)buf;
  // 0-3 op, htype, hlen, hops
  dhcpPtr->op = DHCP_BOOTREQUEST;
  dhcpPtr->htype = 1;
  dhcpPtr->hlen = 6;
  dhcpPtr->hops = 0;
  // 4-7 xid
  dhcpPtr->xid = current_xid;
  // 8-9 secs
  dhcpPtr->secs = current_time;
  // 16-19 yiaddr
  memset(dhcpPtr->yiaddr, 0, 4);
  // 28-43 chaddr(16)
  memcpy(dhcpPtr->chaddr, config->mac, 6);

  // options defined as option, length, value
  bufPtr = buf + sizeof( struct dhcp_data );
  // Magic cookie 99, 130, 83 and 99
  append(99);
  append(130);
  append(83);
  append(99);
        
  // Set options
  // DHCP message type
  append(DHCPC_OPTION_MSGTYPE); // DHCPDISCOVER or DHCPREQUEST
  append(1);        // Length 
  append(type);     // Value

  // Client Identifier Option, this is the client mac address
  append(DHCPC_OPTION_CLIENT_ID);
  append(7);        // Length 
  append(0x01);     // Value
  for( i=0; i<6; i++)
    append(config->mac[i]);

  // Host name Option
  append(DHCPC_OPTION_HOST_NAME);
  append(12);      // Length 
  for( i=0; i<12; i++)
    append(hostname[i]);
  
  // Vendor class identifier
  append(DHCPC_OPTION_VENDOR_CLASS_ID);
  append(8);      // Length 
  for( i=0; i<8; i++)
    append(hostname[i]);
  
  if( type == DHCPREQUEST ) {
    // requested ip address
    append(DHCPC_OPTION_REQUESTED_IP);
    append(4);      // Length 
    for( i=0; i<4; i++)
      append(config->ip[i]);

    // using server ip address
    append(DHCPC_OPTION_SERVER_IP);
    append(4);      // Length 
    for( i=0; i<4; i++) 
      append(config->server[i]);
  }

  // request information in parameter list
  append(DHCPC_OPTION_PARAM_LIST);
  append(3);      // Length 
  append(DHCPC_OPTION_SUBNET_MASK);
  append(DHCPC_OPTION_GATEWAY);
  append(DHCPC_OPTION_DNS_SERVER);

  append(DHCPC_OPTION_END);
  
  len = bufPtr - buf;
  src=io_input_stream_from_buffer(buf, len);
  return w5200_send(s, &src, &len, BROADCAST_ADDRESS, DHCPC_DEST_PORT);
}


// process packet, if not dhcp then exit
static int8_t dhcpc_process(SOCKET s, uint8_t *buf, uint16_t plen) {
  int8_t status=STATUS_OK;
  
  // Map struct onto payload
  struct dhcp_data *dhcp = (struct dhcp_data *)buf;
  if (plen >= 70 && dhcp->op == DHCP_BOOTREPLY && dhcp->xid == current_xid ) {
    int optionIndex = sizeof( struct dhcp_data ) + 4;
    if( buf[optionIndex] == DHCPC_OPTION_MSGTYPE )
      switch( buf[optionIndex+2] ) {
          case DHCPOFFER: 
            status = process_dhcpoffer(s, buf, plen );
            if(status == STATUS_OK) {
              status = process_dhcpack();
            }
            break;
          case DHCPACK:   
            status = process_dhcpack();
            break;
      }
  }
  return status;
}


static int8_t process_dhcpoffer (SOCKET s, uint8_t *buf, uint16_t plen) {
  int8_t status;
  // Map struct onto payload
  struct dhcp_data *dhcpPtr = (struct dhcp_data *)buf;
  // Offered IP address is in yiaddr
  memcpy(config->ip, dhcpPtr->yiaddr, 4);
  // Scan through variable length option list identifying options we want
  uint8_t *ptr = (uint8_t*) (dhcpPtr + 1) + 4;
  do {
      uint8_t option = *ptr++;
      uint8_t optionLen = *ptr++;
      uint8_t i;
      switch (option) {
          case DHCPC_OPTION_SUBNET_MASK:
            memcpy(config->mask, ptr, 4);
            break;
          case DHCPC_OPTION_GATEWAY:
            memcpy(config->gateway, ptr, 4);
            break;
          case DHCPC_OPTION_DNS_SERVER:
            memcpy(config->dns, ptr, 4);
            break;
          case DHCPC_OPTION_LEASE_TIME:
            config->lease_time = 0;
            for (i = 0; i<4; i++)
              config->lease_time = (config->lease_time + ptr[i]) << 8;
            break;
          case DHCPC_OPTION_SERVER_IP:
            memcpy(config->server, ptr, 4);
            break;
      }
      ptr += optionLen;
  } while (ptr < buf + plen);
    
  status = dhcpc_send(s, buf, DHCPREQUEST );
  dhcpc_state = DHCPC_STATE_REQUESTING;
  return status;
}


static int8_t process_dhcpack () {
  dhcpc_state = DHCPC_STATE_BOUND;
  config->lease_start = rtc_get_time();
  return STATUS_OK;
}


// Perform all processing to get an IP address plus other addresses returned, e.g. gw, dns, dhcp server.
// Returns 1 for successful IP address allocation, 0 otherwise
int8_t dhcpc_run(SOCKET s, struct dhcp_config *conf, uint8_t *buf, uint16_t buf_len) {
  int8_t status;
  uint16_t plen = 0;
  uint32_t last = rtc_get_time();
  uint8_t state = 0;
  //bool ip_set = false;
  uint8_t retries = 10;  // After 10 attempts fail gracefully so other action can be carried out
  w5200_socket_t *sock;
  io_output_stream_t dst;
  
  sock=&w5200_sockets[s];
  sock->protocol=W5200_Sn_MR_UDP;
  sock->port=DHCPC_SRC_PORT;
  sock->flag=0x00;
  
  //open udp socket
  if(w5200_socket(s)!=STATUS_OK) {
    return ERR_IO_ERROR;
  }
  
  //send dhcp discovery
  if((status=dhcpc_start(s, conf, buf))!=STATUS_OK) {
    w5200_close(s);
    return status;
  }

  for(;;) {
    // wait for a tcp/udp packet
    if(w5200_wait_for_data(s, 0, DHCPC_TIMEOUT)!=STATUS_OK) {
      w5200_close(s);
      return ERR_TIMEOUT;
    }
    
    if((plen = w5200_get_rx_buf_len(s)) > 0) {
      /* if Rx data size is lager than buf_len */
      /* read the received data */
      if (plen > buf_len) plen = buf_len;
      
      dst=io_output_stream_from_buffer(buf, buf_len);
      if((status=w5200_receive(s, &dst, &plen, config->server, &(config->server_port)))!=STATUS_OK) {
        w5200_close(s);
        return status;
      }
      
      //process dhcp offer and send dhcp request
      if((status=dhcpc_process(s, buf, plen))!=STATUS_OK) {
        w5200_close(s);
        return status;
      }
      
      state = dhcpc_renew();
      
      if( state == DHCPC_STATE_BOUND ) {
        w5200_close(s);
        return STATUS_OK;
      } 
      else {
        if (rtc_get_time() > (last + 10) ){
          last = rtc_get_time();
          if( --retries <= 0 ) {
            //Failed to allocate address
            w5200_close(s);
            return ERR_TIMEOUT;    
          }        
          // send dhcp
          if((status=dhcpc_start(s, conf, buf))!=STATUS_OK) {
            w5200_close(s);
            return status;
          }
        }
        //continue
      }
    }
  }
}