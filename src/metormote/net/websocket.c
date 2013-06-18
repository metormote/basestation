/*
 * websocket.c
 *
 * Created: 2/8/2012 11:09:45 AM
 *  Author: Administrator
 */ 

#include "websocket.h"

static uint16_t counter;
static uint32_t sum;

static int8_t websocket_send_handshake(struct websocket_t *websocket, const char *accept_key);
static int8_t websocket_read_until(struct websocket_t *websocket, char *wait_for_string);
static int8_t calc_response_key(char *response_key, const char *accept_key);


static int8_t websocket_send_handshake(struct websocket_t *websocket, const char *accept_key)
{
  int8_t status;
  char *buf, *p;
  
  buf=mem_safe_malloc(320);
  if(buf==NULL) {
    return ERR_NO_MEMORY;
  }
  
  status=STATUS_OK;
  p=buf;
  p+=sprintf_P(p,PSTR(WEBSOCKET_HANDSHAKE_GET),websocket->options->path);
  p+=sprintf_P(p,PSTR(WEBSOCKET_HANDSHAKE_HOST),websocket->options->host);
  p+=sprintf_P(p,PSTR(WEBSOCKET_HANDSHAKE_UPGRADE));
  p+=sprintf_P(p,PSTR(WEBSOCKET_HANDSHAKE_CONNECTION));
  p+=sprintf_P(p,PSTR(WEBSOCKET_HANDSHAKE_KEY),accept_key);
  p+=sprintf_P(p,PSTR(WEBSOCKET_HANDSHAKE_ORIGIN),websocket->options->origin);
  p+=sprintf_P(p,PSTR(WEBSOCKET_HANDSHAKE_PROTOCOL));
  p+=sprintf_P(p,PSTR(WEBSOCKET_HANDSHAKE_VERSION));
  p+=sprintf(p, (char *)"\r\n");
  
  if(websocket->write_socket(websocket->id, (uint8_t *)buf, p-buf)!=STATUS_OK) {
    status=ERR_TIMEOUT;
  }
  mem_safe_free(buf);
  
  return status;
}


static int8_t websocket_read_until(struct websocket_t *websocket, char *wait_for_string) {
  char c;
  char *p;
  
  if(wait_for_string==NULL) {
    return ERR_INVALID_ARG;
  }
  
  p=wait_for_string;
  for(;;) {
    if(websocket->read_socket(websocket->id, (uint8_t *)&c, 1, WEBSOCKET_READ_TIMEOUT)==STATUS_OK) {
      //check next char
      if(*p++==c) {
        //we have a match
        if(*p=='\0') break;
      }
      //check if the first char matches
      else if(wait_for_string[0]==c) {
        p=wait_for_string+1;
      }
      //restart match
      else {
        p=wait_for_string;
      }                
    }
    else {
      return ERR_TIMEOUT;
    }
  }
  return STATUS_OK;  
}


static int8_t calc_response_key(char *response_key, const char *accept_key) {
  SHA1Context *ctx;
  uint8_t key[SHA1_BYTES];
  int8_t status;
  
  ctx=(SHA1Context *)mem_safe_malloc(sizeof(SHA1Context));
  if(ctx==NULL) {
    return ERR_NO_MEMORY;
  }
  
  SHA1Reset(ctx);
  SHA1Input(ctx, (const uint8_t *)accept_key, strlen(accept_key));
  SHA1Input(ctx, (const uint8_t *)WEBSOCKET_GUID, strlen(WEBSOCKET_GUID));
  if((status=SHA1Result(ctx, (uint8_t *)key))!=STATUS_OK) {
    mem_safe_free(ctx);
    return status;
  }
  
  mem_safe_free(ctx);
  
  base64_encode(key, SHA1_BYTES, response_key);
  
  return STATUS_OK;
}

  
int8_t websocket_start(struct websocket_t *websocket) {
  int8_t status;
  uint8_t i;
  uint16_t k, l;
  uint32_t len;
  uint8_t key[SHA1_BYTES];
  uint8_t accept_key[32];
  uint8_t response_key[32];
  struct websocket_frame_t rx_frame, tx_frame;
  uint8_t *msg_buf;
  uint8_t mask[4];
  
  //init random 16 byte key value
  crypto_init_iv(key);
  
  //base64 encode key
  base64_encode(key, 16, (char *)accept_key);
  
  //calc the response key
  if((status=calc_response_key((char *)response_key, (const char *)accept_key))!=STATUS_OK) {
    return status;
  }
  
  //send handshake
  status=websocket_send_handshake(websocket, (const char *)accept_key);
  if(status!=STATUS_OK) {
    return status;
  }
  
  websocket->state=WEBSOCKET_STATE_CONNECTING;  
  
  //read response
  status=websocket_read_until(websocket, WEBSOCKET_HANDSHAKE_RESPONSE);
  if(status!=STATUS_OK) {
    return status;
  }
  
  //read until the accept header
  status=websocket_read_until(websocket, WEBSOCKET_HEADER_ACCEPT);
  if(status!=STATUS_OK) {
    return status;
  }
  
  //read the accept key
  status=websocket->read_socket(websocket->id, accept_key, WEBSOCKET_HEADER_ACCEPT_KEY_LEN, WEBSOCKET_READ_TIMEOUT);
  if(status!=STATUS_OK) {
    return status;
  }
  
  //read till end of reply
  status=websocket_read_until(websocket, "\r\n\r\n");
  if(status!=STATUS_OK) {
    return status;
  }
  
  //check the response key
  if(memcmp(accept_key, response_key, WEBSOCKET_HEADER_ACCEPT_KEY_LEN)!=0) {
    return ERR_PROTOCOL;
  }
  
  //set state to open
  websocket->state=WEBSOCKET_STATE_OPEN;
  websocket->ping_count=0;
  
  for(;;) {
    status=STATUS_OK;
    
    //start listening for a frame
    while(websocket->state==WEBSOCKET_STATE_OPEN && 
          websocket->read_socket(websocket->id, (uint8_t *)&rx_frame, 2, WEBSOCKET_KEEPALIVE_INTERVAL)==STATUS_OK) {
      
      //reset the transmit frame
      memset(&tx_frame, 0, 2);
      
      switch(rx_frame.payload_len) {
        case 126:
          //read 2 length bytes (big endian)
          status=websocket->read_socket(websocket->id, mask, 2, WEBSOCKET_READ_TIMEOUT);
          len=mask[0];
          len=(len << 8) | mask[1];
          break;
        case 127:
          //read 4 high length bytes
          status=websocket->read_socket(websocket->id, mask, 4, WEBSOCKET_READ_TIMEOUT);
          if(mask[0] | mask[1] | mask[2] | mask[3]) {
            status=ERR_NO_MEMORY;
            break;
          }
          //read 4 low length bytes (big endian)
          status=websocket->read_socket(websocket->id, mask, 4, WEBSOCKET_READ_TIMEOUT);
          len=mask[0];
          len=(len << 8) | mask[1];
          len=(len << 8) | mask[2];
          len=(len << 8) | mask[3];
          if(len>0x40000) {
            status=ERR_NO_MEMORY;
          }
          break;
        default:
          len=rx_frame.payload_len;
      }
      
      if(status!=STATUS_OK) {
        goto on_error;
      }
      
      //read the mask bytes
      if(rx_frame.mask) {
        status=websocket->read_socket(websocket->id, (uint8_t *)mask, 4, WEBSOCKET_READ_TIMEOUT);
        if(status!=STATUS_OK) {
          goto on_error;
        }
      }
      
      if(len>256 || rx_frame.opcode==WEBSOCKET_OPCODE_CONTINUATION) {
        //reset counter if it is the first segment of the message
        if(rx_frame.opcode==WEBSOCKET_OPCODE_BINARY || rx_frame.opcode==WEBSOCKET_OPCODE_TEXT) {
            counter=0;
        }
        
        //allocate memory for the data
        msg_buf=mem_safe_malloc(256);
        if(msg_buf==NULL) {
          status=ERR_NO_MEMORY;
          goto on_error;
        }
        
        for(k=(uint16_t)(len/256+1);k!=0;k--) {
          l=(k<=1 ? (uint16_t)(len%256) : 256);
          if(l==0) break;
          
          //read the data
          status=websocket->read_socket(websocket->id, (uint8_t *)msg_buf, l, WEBSOCKET_READ_TIMEOUT);
          if(status!=STATUS_OK) {
            mem_safe_free(msg_buf);
            goto on_error;
          }
        
          //apply the mask bytes
          if(rx_frame.mask) {
            for(i=0;i<l;i++) {
              msg_buf[i]^=mask[i%4];
            }
          }
          
          websocket->data_callback(counter++, msg_buf, l);
        }
      }
      else if(len>0) {
        //allocate memory for the data
        msg_buf=mem_safe_malloc((size_t)len);
        if(msg_buf==NULL) {
          status=ERR_NO_MEMORY;
          goto on_error;
        }
    
        //read the data
        status=websocket->read_socket(websocket->id, (uint8_t *)msg_buf, len, WEBSOCKET_READ_TIMEOUT);
        if(status!=STATUS_OK) {
          mem_safe_free(msg_buf);
          goto on_error;
        }
    
        //apply the mask bytes
        if(rx_frame.mask) {
          for(i=0;i<len;i++) {
            msg_buf[i]^=mask[i%4];
          }
        }
      }
      else {
        msg_buf=NULL;
      }

      //check the opcode of the frame
      switch(rx_frame.opcode) {
        case WEBSOCKET_OPCODE_CONTINUATION:
          sum+=len;
          if(rx_frame.fin) {
            websocket->on_message(msg_buf, sum);
          }
          break;
        case WEBSOCKET_OPCODE_TEXT:
        case WEBSOCKET_OPCODE_BINARY:
          sum=len;
          if(rx_frame.fin) {
            websocket->on_message(msg_buf, sum);
          }
          break;
        case WEBSOCKET_OPCODE_PING:
          //send pong
          tx_frame.fin=1;
          tx_frame.mask=0;
          tx_frame.opcode=WEBSOCKET_OPCODE_PONG;
          tx_frame.payload_len=0;
          status=websocket->write_socket(websocket->id, (uint8_t *)&tx_frame, 2);
          break;
        case WEBSOCKET_OPCODE_PONG:
          //reset ping counter
          websocket->ping_count=0;
          break;
        case WEBSOCKET_OPCODE_CLOSE:
          websocket_close(websocket);
          websocket->state=WEBSOCKET_STATE_CLOSED;
          break;
        default:
          //unknown opcode
          status=ERR_PROTOCOL;
      }
    
      //free memory
      mem_safe_free(msg_buf);
    
      //break on error
      if(status!=STATUS_OK) {
        goto on_error;
      }
    }
    
    on_error:
    
    //ping connection
    if(status==STATUS_OK && websocket->state==WEBSOCKET_STATE_OPEN
        && websocket->ping_count<WEBSOCKET_KEEPALIVE_GRACE_PERIOD) {
        status=websocket_send_ping(websocket);
        if(status==STATUS_OK) {
          websocket->ping_count++;
          continue;
        }
    }
    
    break;
  }
  
  //if we get here the socket has timed out
  websocket->state=WEBSOCKET_STATE_CLOSED;
  return status;
}


int8_t websocket_close(struct websocket_t *websocket) {
  struct websocket_frame_t tx_frame;
  
  //check that we are in the right state
  if(websocket->state!=WEBSOCKET_STATE_OPEN) {
    return ERR_PROTOCOL;
  }
  memset(&tx_frame, 0, 2);
  //send close
  tx_frame.fin=1;
  tx_frame.mask=0;
  tx_frame.opcode=WEBSOCKET_OPCODE_CLOSE;
  tx_frame.payload_len=0;
  websocket->write_socket(websocket->id, (uint8_t *)&tx_frame, 2);
  websocket->state=WEBSOCKET_STATE_CLOSING;
  
  return STATUS_OK;
}


int8_t websocket_send_ping(struct websocket_t *websocket) {
  struct websocket_frame_t tx_frame;
  int8_t status;
  
  //check that we are in the right state
  if(websocket->state!=WEBSOCKET_STATE_OPEN) {
    return ERR_PROTOCOL;
  }
  memset(&tx_frame, 0, 2);
  tx_frame.fin=1;
  tx_frame.mask=0;
  tx_frame.opcode=WEBSOCKET_OPCODE_PING;
  tx_frame.payload_len=0;
  status=websocket->write_socket(websocket->id, (uint8_t *)&tx_frame, 2);
  return status;
}

int8_t websocket_send_text(struct websocket_t *websocket, char *txt) {
  struct websocket_frame_t *tx_frame;
  uint8_t *buf, *mask, *p;
  uint16_t i, len;
  uint32_t r;
  int8_t status;
  
  //check that we are in the right state
  if(websocket->state!=WEBSOCKET_STATE_OPEN) {
    return ERR_PROTOCOL;
  }
  
  r=random();
  mask=(uint8_t *)&r;
  len=strlen(txt);
  
  buf=mem_safe_malloc(sizeof(struct websocket_frame_t)+len+8);
  if(buf==NULL) {
    return ERR_NO_MEMORY;
  }
  
  tx_frame=(struct websocket_frame_t *)buf;
  memset(tx_frame, 0, 2);
  
  tx_frame->fin=1;
  tx_frame->mask=1;
  tx_frame->opcode=WEBSOCKET_OPCODE_TEXT;
  if(len>125) {
    tx_frame->payload_len=126;
  }
  else {
    tx_frame->payload_len=len;
  }
  p=buf+2;
  if(len>125) {
    *p++=(uint8_t)((0xFF00 & len) >> 8);
    *p++=(uint8_t)(0x00FF & len);
  }
  for(i=0;i<4;i++) {
    *p++=mask[i];
  }
  for(i=0;i<len;i++) {
    *p++=(*txt++)^mask[i%4];
  }
  
  status=websocket->write_socket(websocket->id, buf, p-buf);
  
  mem_safe_free(buf);
  
  return status;
}

int8_t websocket_send_binary(struct websocket_t *websocket, io_input_stream_t *stream, uint16_t len) {
  struct websocket_frame_t *tx_frame;
  int8_t status;
  uint8_t *mask, *buf;
  uint16_t i;
  uint32_t r;
  uint8_t *p;
    
  //check that we are in the right state
  if(websocket->state!=WEBSOCKET_STATE_OPEN) {
    return ERR_PROTOCOL;
  }
  
  r=random();
  mask=(uint8_t *)&r;
  
  buf=mem_safe_malloc(sizeof(struct websocket_frame_t)+len+8);
  if(buf==NULL) {
    return ERR_NO_MEMORY;
  }
  
  tx_frame=(struct websocket_frame_t *)buf;
  memset(tx_frame, 0, 2);
  
  //TODO support message fragmenting
  tx_frame->fin=1;
  tx_frame->mask=1;
  tx_frame->opcode=WEBSOCKET_OPCODE_BINARY;
  if(len>125) {
    tx_frame->payload_len=126;
  }
  else {
    tx_frame->payload_len=len;
  }
  p=buf+2;
  if(len>125) {
    *p++=(uint8_t)((0xFF00 & len) >> 8);
    *p++=(uint8_t)(0x00FF & len);
  }
  if(tx_frame->mask) {
    for(i=0;i<4;i++) {
      *p++=mask[i];
    }
  }
  stream->read(stream, p, len);
  if(tx_frame->mask) {
    for(i=0;i<len;i++) {
      *p^=mask[i%4];
      p++;
    }
  }
  else {
    p+=len;
  }
  
  status=websocket->write_socket(websocket->id, buf, p-buf);
  
  mem_safe_free(buf);
  
  return status;
}

