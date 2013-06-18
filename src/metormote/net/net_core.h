/*
 * net_core.h
 *
 * Created: 12/26/2011 12:31:56 PM
 *  Author: Administrator
 */ 

#ifndef NET_CORE_H_
#define NET_CORE_H_

// ******* ETH *******
#define ETH_HEADER_LEN  14
// values of certain bytes:
#define ETHTYPE_ARP_H   0x08
#define ETHTYPE_ARP_L   0x06
#define ETHTYPE_IP_H    0x08
#define ETHTYPE_IP_L    0x00
// byte positions in the ethernet frame:
//
// Ethernet type field (2bytes):
#define ETH_TYPE_H_POS    12
#define ETH_TYPE_L_POS    13
//
#define ETH_DST_MAC     0
#define ETH_SRC_MAC     6


// ******* ARP *******
#define ETH_ARP_OPCODE_REPLY_H  0x0
#define ETH_ARP_OPCODE_REPLY_L  0x02
#define ETH_ARP_OPCODE_REQ_H    0x0
#define ETH_ARP_OPCODE_REQ_L    0x01
// start of arp header:
#define ETH_ARP_POS           0xe
//
#define ETHTYPE_ARP_L     0x06
// arp.dst.ip
#define ETH_ARP_DST_IP_POS    0x26
// arp.opcode
#define ETH_ARP_OPCODE_H_POS  0x14
#define ETH_ARP_OPCODE_L_POS  0x15
// arp.src.mac
#define ETH_ARP_SRC_MAC_POS   0x16
#define ETH_ARP_SRC_IP_POS    0x1c
#define ETH_ARP_DST_MAC_POS   0x20
#define ETH_ARP_DST_IP_POS    0x26

// ******* IP *******
#define IP_HEADER_LEN      20

#define IP_PROTO_ICMP      0x01
#define IP_PROTO_TCP      0x06
#define IP_PROTO_UDP      0x11
#define IP_V4              0x40
#define IP_HEADER_LENGTH  0x05

#define IP_POS                  0x0E
#define IP_HEADER_VER_LEN_POS    0x0E
#define IP_TOS_POS              0x0F
#define IP_TOTLEN_H_POS          0x10
#define IP_TOTLEN_L_POS          0x11
#define IP_ID_H_POS              0x12
#define IP_ID_L_POS              0x13
#define IP_FLAGS_POS             0x14     
#define IP_FLAGS_H_POS          0x14
#define IP_FLAGS_L_POS          0x15
#define IP_TTL_POS              0x16
#define IP_PROTO_POS            0x17
#define IP_CHECKSUM_POS         0x18 
#define IP_CHECKSUM_H_POS        0x18
#define IP_CHECKSUM_L_POS        0x19
#define IP_SRC_IP_POS            0x1A
#define IP_DST_IP_POS            0x1E

#define IP_SRC_POS               0x1a
#define IP_DST_POS               0x1e
#define IP_HEADER_LEN_VER_POS   0xe

// ******* ICMP *******
#define ICMP_TYPE_ECHOREPLY     0
#define ICMP_TYPE_ECHOREQUEST   8
//
#define ICMP_TYPE_POS         0x22
#define ICMP_CHECKSUM_POS     0x24
#define ICMP_CHECKSUM_H_POS   0x24
#define ICMP_CHECKSUM_L_POS   0x25
#define ICMP_IDENT_H_POS      0x26
#define ICMP_IDENT_L_POS      0x27
#define ICMP_DATA_POS         0x2a

// ******* UDP *******
#define UDP_HEADER_LEN      8
//
#define UDP_SRC_PORT_H_POS    0x22
#define UDP_SRC_PORT_L_POS    0x23
#define UDP_DST_PORT_H_POS    0x24
#define UDP_DST_PORT_L_POS    0x25
//
#define UDP_LEN_H_POS         0x26
#define UDP_LEN_L_POS         0x27
#define UDP_CHECKSUM_H_POS    0x28
#define UDP_CHECKSUM_L_POS    0x29
//#define UDP_DATA_POS          0x2a
#define UDP_DATA_POS          0x00

// ******* TCP *******
#define TCP_FLAGS_FIN         0x01
#define TCP_FLAGS_SYN         0x02
#define TCP_FLAGS_RST         0x04
#define TCP_FLAGS_PUSH        0x08
#define TCP_FLAGS_ACK         0x10
#define TCP_FLAGS_SYNACK      0x12
#define TCP_FLAGS_PSHACK      0x18

#define TCP_SRC_PORT_H_POS    0x22
#define TCP_SRC_PORT_L_POS    0x23
#define TCP_DST_PORT_H_POS    0x24
#define TCP_DST_PORT_L_POS    0x25
#define TCP_SEQ_POS           0x26   //4 bytes 0x26-0x29
#define TCP_SEQ_H_POS         0x26
#define TCP_SEQACK_POS        0x2a
#define TCP_SEQACK_H_POS      0x2a
// flags: SYN=2
#define TCP_FLAGS_POS         0x2f
//#define TCP_FLAG_POS        0x2f
#define TCP_WINDOWSIZE_H_POS  0x30
#define TCP_WINDOWSIZE_L_POS  0x31
#define TCP_CHECKSUM_H_POS     0x32
#define TCP_CHECKSUM_L_POS     0x33
#define TCP_URGENT_PTR_H_POS   0x34
#define TCP_URGENT_PTR_L_POS   0x35
#define TCP_OPTIONS_POS       0x36
#define TCP_DATA_POS          0x36
//  plain len without the options:
#define TCP_HEADER_LEN_PLAIN 20
#define TCP_HEADER_LEN_POS    0x2e
#define TCP_WIN_SIZE        0x30
#define TCP_CHECKSUM_H_POS    0x32
#define TCP_CHECKSUM_L_POS    0x33
#define TCP_OPTIONS_POS       0x36


#define IP_BROADCAST_ADDRESS    {0xFF, 0xFF, 0xFF, 0xFF}
  
#endif /* NET_CORE_H_ */