/*
 * dns_client.h
 *
 * Created: 1/10/2013 8:43:24 AM
 *  Author: Administrator
 */ 

#ifndef DNS_CLIENT_H_
#define DNS_CLIENT_H_

#include <string.h>
#include "status_codes.h"
#include "w5200.h"
#include "rtc.h"
#include "io_stream.h"
#include "mem.h"

#define DNS_SOCKET_NONE                255
#define DNS_SRC_PORT                  54
#define DNS_DEST_PORT                 53


#define DNS_TIMEOUT                   10*SYSTEM_TICKS_PER_SEC

// Various flags and header field values for a DNS message
#define DNS_UDP_HEADER_SIZE            8
#define DNS_HEADER_SIZE                12
#define DNS_TTL_SIZE                  4

#define DNS_QUERY_FLAG               (0)
#define DNS_RESPONSE_FLAG            (1<<15)
#define DNS_QUERY_RESPONSE_MASK      (1<<15)
#define DNS_OPCODE_STANDARD_QUERY    (0)
#define DNS_OPCODE_INVERSE_QUERY     (1<<11)
#define DNS_OPCODE_STATUS_REQUEST    (2<<11)
#define DNS_OPCODE_MASK              (15<<11)
#define DNS_AUTHORITATIVE_FLAG       (1<<10)
#define DNS_TRUNCATION_FLAG          (1<<9)
#define DNS_RECURSION_DESIRED_FLAG   (1<<8)
#define DNS_RECURSION_AVAILABLE_FLAG (1<<7)
#define DNS_RESP_NO_ERROR            (0)
#define DNS_RESP_FORMAT_ERROR        (1)
#define DNS_RESP_SERVER_FAILURE      (2)
#define DNS_RESP_NAME_ERROR          (3)
#define DNS_RESP_NOT_IMPLEMENTED     (4)
#define DNS_RESP_REFUSED             (5)
#define DNS_RESP_MASK                (15)
#define DNS_TYPE_A                   (0x0001)
#define DNS_CLASS_IN                 (0x0001)
#define DNS_LABEL_COMPRESSION_MASK   (0xC0)


#define htons(x) ( ((x)<<8) | (((x)>>8)&0xFF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)


#ifdef __cplusplus
extern "C" { 
#endif

struct dns_request_t {
 /*
    0  1  2  3  4  5  6  7  8  9  0  1  2  3  4  5
  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
  |                      ID                       |
  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
  |QR|   Opcode  |AA|TC|RD|RA|   Z    |   RCODE   |
  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
  |                    QDCOUNT                    |
  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
  |                    ANCOUNT                    |
  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
  |                    NSCOUNT                    |
  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
  |                    ARCOUNT                    |
  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
  
 */

  uint16_t id;          // Bits 0-15 are the query identifier
  uint16_t flags;       // 0      : Bit one Query or response
                        // 1 - 4  : OPcode, set 0 for standard
                        // 5      : AA, set to 1 for response 0 for query
                        // 6      : TC, truncated
                        // 7      : RD, Recursion desired 
                        // 8      : RA, recursion available
                        // 9-10    : set to zero
                        // 11-15    : Rcode, set 0's for client
  uint16_t qdcount; // how many queries
  uint16_t ancount;
  uint16_t nscount;
  uint16_t arcount;
/*
    0  1  2  3  4  5  6  7  8  9  0  1  2  3  4  5
  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
  |                                               |
  /                     QNAME                     /
  /                                               /
  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
  |                     QTYPE                     |
  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
  |                     QCLASS                    |
  +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
*/
  uint8_t query[128];
};


struct dns_answer_header_t {
  uint16_t type;
  uint16_t class;
  uint8_t  ttl[DNS_TTL_SIZE];
  uint16_t len;
};

int8_t dns_resolve(SOCKET s, uint8_t *dns, const char* host_name, uint8_t *ip);

#ifdef __cplusplus
}
#endif

#endif /* DNS_CLIENT_H_ */