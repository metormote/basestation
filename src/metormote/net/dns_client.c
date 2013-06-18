/*
 * dns_client.c
 *
 * Created: 1/10/2013 8:43:46 AM
 *  Author: Administrator
 */ 
#include "dns_client.h"

static uint16_t build_request(struct dns_request_t *request, const char* host_name);
static int8_t process_response(struct dns_request_t *response, uint8_t *ip);
static int8_t inet_aton(const char* ip_str, uint8_t *ip);

int8_t dns_resolve(SOCKET s, uint8_t *dns, const char *host_name, uint8_t *ip) {
  int8_t status;
  uint8_t retries = 3;
  uint16_t id, len, plen = 0;
  struct dns_request_t *request;
  w5200_socket_t *sock;
  io_input_stream_t src;
  io_output_stream_t dst;
  uint8_t server_ip[4];
  uint16_t server_port;

  //Check if we already have an ip address
  if (inet_aton(host_name, ip)==STATUS_OK) {
      return STATUS_OK;
  }
  
  sock=&w5200_sockets[s];
  sock->protocol=W5200_Sn_MR_UDP;
  sock->port=DNS_SRC_PORT;
  sock->flag=0x00;
  
  //open udp socket
  if(w5200_socket(s)!=STATUS_OK) {
    return ERR_IO_ERROR;
  }
  
  request=mem_safe_malloc(sizeof(struct dns_request_t));
  if(request==NULL) {
    return ERR_NO_MEMORY;
  }
  
  for(;retries!=0;retries--) {
    memset(request, 0, sizeof(struct dns_request_t));
    len=build_request(request, host_name);
  
    id=request->id;
  
    src=io_input_stream_from_buffer((uint8_t *)request, len);
    if((status=w5200_send(s, &src, &len, dns, DNS_DEST_PORT))!=STATUS_OK) {
      break;
    }

    // wait for a tcp/udp packet
    if((status=w5200_wait_for_data(s, 0, DNS_TIMEOUT))!=STATUS_OK) {
      break;
    }
    
    if((plen = w5200_get_rx_buf_len(s)) > 0) {
      if(plen < DNS_HEADER_SIZE || plen > sizeof(struct dns_request_t)) {
        status=ERR_BAD_DATA;
        break;
      }
      
      memset(request, 0, sizeof(struct dns_request_t));
      dst=io_output_stream_from_buffer((uint8_t *)request, len);
      if((status=w5200_receive(s, &dst, &plen, server_ip, &server_port))!=STATUS_OK) {
        break;
      }
      
      // Check that it's a response from the right server and the right port
      if ( (memcmp(dns, server_ip, 4) != 0) || 
        (server_port != DNS_DEST_PORT) ) {
        // It's not from who we expected
        status=ERR_BAD_ADDRESS;
        continue;
      }
      
      //Check id
      if(request->id!=id) {
        status=ERR_BAD_DATA;
        continue;
      }
      
      //process dns response
      if((status=process_response(request, ip))!=STATUS_OK) {
        break;
      }
      
      break;
    }
  }
  
  mem_safe_free(request);
  
  w5200_close(s);
  return status;
}  



static uint16_t build_request(struct dns_request_t *request, const char* host_name)
{
  uint8_t len;
  const char *start, *end;
  uint8_t *p;
  uint16_t buf;
  
  request->id=(uint16_t)rtc_get_time();
  request->flags = htons(DNS_QUERY_FLAG | DNS_OPCODE_STANDARD_QUERY | DNS_RECURSION_DESIRED_FLAG);
  request->qdcount = htons(1);
  request->ancount=0;
  request->nscount=0;
  request->arcount=0;
    
  // Build query
  start = host_name;
  end = start;
  p=request->query;
  // Run through the name being requested
  while (*end) {
    // Find out how long this section of the name is
    end = start;
    while(*end && (*end != '.')) {
      end++;
    }

    if(end-start > 0) {
      len = end-start;
      *p++=len;
      memcpy(p, (uint8_t*)start, len);
      p+=len;
    }
    start = end+1;
  }
    
  // Terminate it with a zero-length section
  len = 0;
  *p++=len;
  
  // Type of question
  buf = htons(DNS_TYPE_A);
  memcpy(p, (uint8_t*)&buf, 2);
  p+=2;
  
  // Internet class of question
  buf = htons(DNS_CLASS_IN);
  memcpy(p, (uint8_t*)&buf, 2);
  p+=2;
  
  return (uint16_t)(p-((uint8_t *)request));
}


static int8_t process_response(struct dns_request_t *response, uint8_t *ip)
{
  uint8_t i, len;
  uint16_t header_flags, answer_count;
  struct dns_answer_header_t *answer;
  uint8_t *p;
  
  header_flags = htons(response->flags);
  if ((header_flags & DNS_QUERY_RESPONSE_MASK) != (uint16_t)DNS_RESPONSE_FLAG) {
    return ERR_PROTOCOL;
  }
  
  // Check for any errors in the response (or in our request)
  if ( (header_flags & DNS_TRUNCATION_FLAG) || (header_flags & DNS_RESP_MASK) ) {
    return ERR_PROTOCOL;
  }

  // And make sure we have got (at least) one answer
  answer_count=htons(response->ancount);
  if (answer_count == 0 ) {
    return ERR_PROTOCOL;
  }

  p=response->query;
  
  // Skip over any questions
  for (uint16_t i =0; i < htons(response->qdcount); i++) {
    // Skip over the name
    do {
      len=*p++;
      p+=len;
    } while (len != 0);

    // Jump over the type and class
    p+=4;
  }

  // Now we're up to the bit we're interested in, the answer
  // There might be more than one answer (although we'll just use the first
  // type A answer) and some authority and additional resource records but
  // we're going to ignore all of them.

  for (i=0; i < answer_count; i++) {
    do {
      len=*p++;
      if ((len & DNS_LABEL_COMPRESSION_MASK) == 0) {
        p+=len;
      }
      else {
        // This is a pointer to a somewhere else in the message for the
        // rest of the name.  We don't care about the name, and RFC1035
        // says that a name is either a sequence of labels ended with a
        // 0 length octet or a pointer or a sequence of labels ending in
        // a pointer.  Either way, when we get here we're at the end of
        // the name
        // Skip over the pointer
        p++;
        // And set len so that we drop out of the name loop
        len = 0;
      }
    } while (len != 0);
    
    answer = (struct dns_answer_header_t *)p;
    p+=sizeof(struct dns_answer_header_t);
    
    if ((htons(answer->type) == DNS_TYPE_A) && (htons(answer->class) == DNS_CLASS_IN) ) {
      if (htons(answer->len) != 4) {
          // It's a weird size
          return ERR_BAD_DATA;//INVALID_RESPONSE;
      }
      //get the ip address
      memcpy(ip, p, 4);
      return STATUS_OK;
    }
    else {
      // This isn't an answer type we're after, move onto the next one
      p+=answer->len;
    }
  }
    
  // If we get here then we haven't found an answer
  return ERR_PROTOCOL;
}


static int8_t inet_aton(const char* ip_str, uint8_t *ip)
{
    // Check if we have been given a valid IP address
    const char* p=ip_str;
    while (*p &&
           ((*p == '.') || (*p >= '0') || (*p <= '9') )) {
        p++;
    }

    if (*p == '\0') {
        p = ip_str;
        int segment =0;
        int seg_val =0;
        while (*p && (segment < 4)) {
            if (*p == '.') {
                // We have reached the end of a segment
                if (seg_val > 255) {
                    // You can't have IP address segments that don't fit in a byte
                    return ERR_BAD_FORMAT;
                }
                else {
                    ip[segment] = (uint8_t)seg_val;
                    segment++;
                    seg_val = 0;
                }
            }
            else {
                // Next digit
                seg_val = (seg_val*10)+(*p - '0');
            }
            p++;
        }
        // We have reached the end of address, but there'll still be the last
        // segment to deal with
        if ((seg_val > 255) || (segment > 3)) {
            // You can't have IP address segments that don't fit in a byte,
            // or more than four segments
            return ERR_BAD_FORMAT;
        }
        else {
            ip[segment] = (uint8_t)seg_val;
            return STATUS_OK;
        }
    }
    else {
        return ERR_BAD_FORMAT;
    }
}