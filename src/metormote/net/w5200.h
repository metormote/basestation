/*
 * w5200.h
 *
 * Created: 12/27/2011 6:05:28 PM
 *  Author: Administrator
 */ 
#ifndef W5200_H_
#define W5200_H_

#include <stdio.h>
#include <string.h>
#include <spi.h>
#include <spi_master.h>
#include <status_codes.h>
#include <interrupt.h>
#include "board.h"
#include "util/delay.h"
#include "gpio.h"
#include "atom.h"
#include "atomsem.h"
#include "atommutex.h"
#include "io_stream.h"

//! SPI master speed in Hz.
#define W5200_SPI_MASTER_SPEED    32000000

//! Number of bits in each SPI transfer.
#define W5200_SPI_BITS            8

#define W5200_MAX_SOCK_NUM  1

//RX MEM SIZE- SOCKET 0:16KB, SOCKET1-7:0KB
#define W5200_DEFAULT_RX_MEM_MAP  {16,0,0,0,0,0,0,0}

//TX MEM SIZE- SOCKET 0:16KB, SOCKET1-7:0KB
#define W5200_DEFAULT_TX_MEM_MAP  {16,0,0,0,0,0,0,0}
  
#define W5200_DEFAULT_NO_OF_RETRIES       3
#define W5200_DEFAULT_RETRY_TIMER         4*6000

#define W5200_SOCKET_TIMEOUT              10*SYSTEM_TICKS_PER_SEC


#define W5200_COMMON_BASE 0x0000
#define __DEF_IINCHIP_MAP_TXBUF__ (W5200_COMMON_BASE + 0x8000) /* Internal Tx buffer address of the chip */
#define __DEF_IINCHIP_MAP_RXBUF__ (W5200_COMMON_BASE + 0xC000) /* Internal Rx buffer address of the chip */

#define W5200_MR        (W5200_COMMON_BASE + 0x0000) 
/**
 @brief Gateway IP Register address
 */
#define W5200_GAR0              (W5200_COMMON_BASE + 0x0001)
/**
 @brief Subnet mask Register address
 */
#define W5200_SUBR0              (W5200_COMMON_BASE + 0x0005)
/**
 @brief Source MAC Register address
 */
#define W5200_SHAR0        (W5200_COMMON_BASE + 0x0009)
/**
 @brief Source IP Register address
 */
#define W5200_SIPR0        (W5200_COMMON_BASE + 0x000F)
/**
 @brief Interrupt Register
 */
#define W5200_IR        (W5200_COMMON_BASE + 0x0015)
/**
 @brief Socket Interrupt Register
 */
#define W5200_IR2        (W5200_COMMON_BASE + 0x0034) 
/**
 @brief PHY Status Register
 */
#define W5200_PHY        (W5200_COMMON_BASE + 0x0035)
/**
 @brief Interrupt mask register
 */
#define W5200_IMR        (W5200_COMMON_BASE + 0x0036)
/**
 @brief Socket Interrupt Mask Register
 */
#define W5200_IMR2        (W5200_COMMON_BASE + 0x0016)
/**
 @brief Timeout register address( 1 is 100us )
 */
#define W5200_RTR        (W5200_COMMON_BASE + 0x0017)
/**
 @brief Retry count register
 */
#define W5200_RCR        (W5200_COMMON_BASE + 0x0019)
/**
 @brief Authentication type register address in PPPoE mode
 */
#define W5200_PATR0              (W5200_COMMON_BASE + 0x001C)
#define W5200_PPPALGO           (W5200_COMMON_BASE + 0x001E)
/**
 @briefPPP LCP Request Timer register  in PPPoE mode
 */
#define W5200_PTIMER                     (W5200_COMMON_BASE + 0x0028)
/**
 @brief PPP LCP Magic number register  in PPPoE mode
 */
#define W5200_PMAGIC                     (W5200_COMMON_BASE + 0x0029)
/**
 @brief chip version register address
 */
#define W5200_VERSIONR      (W5200_COMMON_BASE + 0x001F)   
/**
 @brief Unreachable IP register address in UDP mode
 */
#define W5200_UIPR0        (W5200_COMMON_BASE + 0x002A)
/**
 @brief Unreachable Port register address in UDP mode
 */
#define W5200_UPORT0              (W5200_COMMON_BASE + 0x002E)
/**
 @brief set Interrupt low level timer register address
 */
#define W5200_INTLEVEL0      (W5200_COMMON_BASE + 0x0030)
#define W5200_INTLEVEL1      (W5200_COMMON_BASE + 0x0031)
/**
 @brief socket register
*/
#define W5200_CH_BASE       (W5200_COMMON_BASE + 0x4000)
/**
 @brief  size of each channel register map
 */
#define W5200_CH_SIZE        0x0100
/**
 @brief socket mode register
 */
#define W5200_Sn_MR(ch)            (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0000)
/**
 @brief socket command register
 */
#define W5200_Sn_CR(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0001)
/**
 @brief socket interrupt register
 */
#define W5200_Sn_IR(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0002)
/**
 @brief socket status register
 */
#define W5200_Sn_SR(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0003)
/**
 @brief socket source port register
 */
#define W5200_Sn_PORT0(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0004)
/**
 @brief Peer MAC register address
 */
#define W5200_Sn_DHAR0(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0006)
/**
 @brief Peer IP register address
 */
#define W5200_Sn_DIPR0(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x000C)
/**
 @brief Peer port register address
 */
#define W5200_Sn_DPORT0(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0010)
/**
 @brief Maximum Segment Size(Sn_MSSR0) register address
 */
#define W5200_Sn_MSSR0(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0012)
/**
 @brief Protocol of IP Header field register in IP raw mode
 */
#define W5200_Sn_PROTO(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0014)

/** 
 @brief IP Type of Service(TOS) Register 
 */
#define W5200_Sn_TOS(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0015)
/**
 @brief IP Time to live(TTL) Register 
 */
#define W5200_Sn_TTL(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0016)
/**
 @brief Receive memory size register
 */
#define W5200_Sn_RXMEM_SIZE(ch)          (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x001E)
/**
 @brief Transmit memory size register
 */
#define W5200_Sn_TXMEM_SIZE(ch)          (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x001F)
/**
 @brief Transmit free memory size register
 */
#define W5200_Sn_TX_FSR0(ch)            (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0020)
/**
 @brief Transmit memory read pointer register address
 */
#define W5200_Sn_TX_RD0(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0022)
/**
 @brief Transmit memory write pointer register address
 */
#define W5200_Sn_TX_WR0(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0024)
/**
 @brief Received data size register
 */
#define W5200_Sn_RX_RSR0(ch)            (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0026)
/**
 @brief Read point of Receive memory
 */
#define W5200_Sn_RX_RD0(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x0028)
/**
 @brief Write point of Receive memory
 */
#define W5200_Sn_RX_WR0(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x002A)
/**
 @brief socket interrupt mask register
 */
#define W5200_Sn_IMR(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x002C)
/**
 @brief frag field value in IP header register
 */
#define W5200_Sn_FRAG(ch)      (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x002D)
/**
 @brief Keep Timer register
 */
#define W5200_Sn_KEEP_TIMER(ch)    (W5200_CH_BASE + ch * W5200_CH_SIZE + 0x002F)

/* MODE register values */
#define W5200_MR_RST      0x80 /**< reset */
#define W5200_MR_WOL      0x20 /**< Wake on Lan */
#define W5200_MR_PB        0x10 /**< ping block */
#define W5200_MR_PPPOE    0x08 /**< enable pppoe */
#define W5200_MR_LB        0x04 /**< little or big endian selector in indirect mode */
#define W5200_MR_AI        0x02 /**< auto-increment in indirect mode */
#define W5200_MR_IND      0x01 /**< enable indirect mode */

/* IR register values */
#define W5200_IR_CONFLICT  0x80 /**< check ip confict */
#define W5200_IR_UNREACH  0x40 /**< get the destination unreachable message in UDP sending */
#define W5200_IR_PPPoE    0x20 /**< get the PPPoE close message */
#define W5200_IR_MAGIC    0x10 /**< get the magic packet interrupt */
#define W5200_IR_SOCK(ch)  (0x01 << ch) /**< check socket interrupt */

/* Sn_MR values */
#define W5200_Sn_MR_CLOSE    0x00    /**< unused socket */
#define W5200_Sn_MR_TCP      0x01    /**< TCP */
#define W5200_Sn_MR_UDP      0x02    /**< UDP */
#define W5200_Sn_MR_IPRAW    0x03    /**< IP LAYER RAW SOCK */
#define W5200_Sn_MR_MACRAW  0x04    /**< MAC LAYER RAW SOCK */
#define W5200_Sn_MR_PPPOE    0x05    /**< PPPoE */
#define W5200_Sn_MR_ND      0x20    /**< No Delayed Ack(TCP) flag */
#define W5200_Sn_MR_MULTI    0x80    /**< support multicating */

/* Sn_CR values */
#define W5200_Sn_CR_OPEN      0x01    /**< initialize or open socket */
#define W5200_Sn_CR_LISTEN    0x02    /**< wait connection request in tcp mode(Server mode) */
#define W5200_Sn_CR_CONNECT    0x04    /**< send connection request in tcp mode(Client mode) */
#define W5200_Sn_CR_DISCON    0x08    /**< send closing request in tcp mode */
#define W5200_Sn_CR_CLOSE      0x10    /**< close socket */
#define W5200_Sn_CR_SEND      0x20    /**< update txbuf pointer, send data */
#define W5200_Sn_CR_SEND_MAC  0x21    /**< send data with MAC address, so without ARP process */
#define W5200_Sn_CR_SEND_KEEP 0x22    /**<  send keep alive message */
#define W5200_Sn_CR_RECV      0x40    /**< update rxbuf pointer, recv data */

#ifdef __DEF_IINCHIP_PPP__
  #define W5200_Sn_CR_PCON      0x23     
  #define W5200_Sn_CR_PDISCON    0x24     
  #define W5200_Sn_CR_PCR    0x25     
  #define W5200_Sn_CR_PCN    0x26    
  #define W5200_Sn_CR_PCJ    0x27    
#endif

/* Sn_IR values */
#ifdef __DEF_IINCHIP_PPP__
  #define W5200_Sn_IR_PRECV    0x80    
  #define W5200_Sn_IR_PFAIL    0x40    
  #define W5200_Sn_IR_PNEXT    0x20    
#endif

#define W5200_Sn_IR_SEND_OK      0x10    /**< complete sending */
#define W5200_Sn_IR_TIMEOUT      0x08    /**< assert timeout */
#define W5200_Sn_IR_RECV        0x04    /**< receiving data */
#define W5200_Sn_IR_DISCON      0x02    /**< closed socket */
#define W5200_Sn_IR_CON          0x01    /**< established connection */

/* Sn_SR values */
#define W5200_SOCK_CLOSED        0x00    /**< closed */
#define W5200_SOCK_INIT         0x13    /**< init state */
#define W5200_SOCK_LISTEN        0x14    /**< listen state */
#define W5200_SOCK_SYNSENT      0x15    /**< connection state */
#define W5200_SOCK_SYNRECV      0x16    /**< connection state */
#define W5200_SOCK_ESTABLISHED  0x17    /**< success to connect */
#define W5200_SOCK_FIN_WAIT      0x18    /**< closing state */
#define W5200_SOCK_CLOSING      0x1A    /**< closing state */
#define W5200_SOCK_TIME_WAIT    0x1B    /**< closing state */
#define W5200_SOCK_CLOSE_WAIT    0x1C    /**< closing state */
#define W5200_SOCK_LAST_ACK      0x1D    /**< closing state */
#define W5200_SOCK_UDP          0x22    /**< udp socket */
#define W5200_SOCK_IPRAW        0x32    /**< ip raw mode socket */
#define W5200_SOCK_MACRAW        0x42    /**< mac raw mode socket */
#define W5200_SOCK_PPPOE        0x5F    /**< pppoe socket */

/* IP PROTOCOL */
#define W5200_IPPROTO_IP              0           /**< Dummy for IP */
#define W5200_IPPROTO_ICMP            1           /**< Control message protocol */
#define W5200_IPPROTO_IGMP            2           /**< Internet group management protocol */
#define W5200_IPPROTO_GGP             3           /**< Gateway^2 (deprecated) */
#define W5200_IPPROTO_TCP             6           /**< TCP */
#define W5200_IPPROTO_PUP             12          /**< PUP */
#define W5200_IPPROTO_UDP             17          /**< UDP */
#define W5200_IPPROTO_IDP             22          /**< XNS idp */
#define W5200_IPPROTO_ND              77          /**< UNOFFICIAL net disk protocol */
#define W5200_IPPROTO_RAW             255         /**< Raw IP packet */

#define W5200_COMMAND_READ            0
#define W5200_COMMAND_WRITE           1

#define W5200_TIMEOUT                 SYSTEM_TICKS_PER_SEC

#ifdef __cplusplus
extern "C" { 
#endif

typedef uint8_t SOCKET;

struct w5200_socket {
  uint8_t protocol;
  uint16_t port;
  uint8_t flag;
};

typedef struct w5200_socket w5200_socket_t;

struct w5200_frame {
  uint8_t address_high;
  uint8_t address_low;
  uint8_t len_high:7, command:1;
  uint8_t len_low;
  uint8_t data;
  union {
    io_input_stream_t *src;
    io_output_stream_t *dst;
  } stream;
};

extern w5200_socket_t w5200_sockets[W5200_MAX_SOCK_NUM];

/*********************************************************
* w5200 access function
*********************************************************/
void w5200_init(void);
void w5200_reset(void);

// setting tx/rx buf size
void w5200_config_mem(uint8_t *tx_size, uint8_t *rx_size);

uint8_t w5200_get_link_state(void);
void w5200_set_gateway(uint8_t *gw);
void w5200_get_gateway(uint8_t *gw);
void w5200_set_subnet(uint8_t *subnet);
void w5200_get_subnet(uint8_t *subnet);
void w5200_set_mac(uint8_t *mac);
void w5200_get_mac(uint8_t *mac);
void w5200_set_source_ip(uint8_t *ip);
void w5200_get_source_ip(uint8_t *ip);

uint8_t w5200_get_interrupt( void );
void w5200_set_interrupt_mask(uint8_t mask);
void w5200_set_timeout(uint16_t timeout);
void w5200_set_retries(uint8_t retry);

void w5200_set_max_segment_size(SOCKET s, uint16_t mss);
void w5200_set_ttl(SOCKET s, uint8_t ttl);
void w5200_set_protocol_field(SOCKET s, uint8_t proto);
uint8_t w5200_get_socket_interrupt(SOCKET s);
void w5200_clear_socket_interrupt(SOCKET s, uint8_t val);
uint8_t w5200_get_socket_status(SOCKET s);
uint8_t w5200_get_socket_command_reg(SOCKET s);

int8_t w5200_socket(SOCKET s);
int8_t w5200_close(SOCKET s);
int8_t w5200_listen(SOCKET s);
int8_t w5200_connect(SOCKET s, uint8_t *ip, uint16_t port);
int8_t w5200_disconnect(SOCKET s);
int8_t w5200_send_tcp(SOCKET s, io_input_stream_t *src, uint16_t len);
int8_t w5200_send(SOCKET s, io_input_stream_t *src, uint16_t *len, uint8_t *ip, uint16_t port);
int8_t w5200_receive_tcp(SOCKET s, io_output_stream_t *dst, uint16_t len);
int8_t w5200_receive(SOCKET s, io_output_stream_t *dst, uint16_t *len, uint8_t *ip, uint16_t *port);
uint16_t w5200_get_tx_buf_free(SOCKET s);
uint16_t w5200_get_rx_buf_len(SOCKET s);
int8_t w5200_wait_for_data(SOCKET s, uint16_t len, int32_t timeout);

#ifdef __DEF_IINCHIP_PPP__
uint8_t w5200_ppp_init(uint8_t *id, uint8_t idlen, uint8_t *passwd, uint8_t passwdlen);
uint8_t w5200_ppp_term(uint8_t *mac,uint8_t *sessionid);
#endif


#ifdef __cplusplus
}
#endif

#endif /* W5200_H_ */