/*
 * websocket.h
 *
 * Created: 2/8/2012 11:09:32 AM
 *  Author: Administrator
 */ 


#ifndef WEBSOCKET_H_
#define WEBSOCKET_H_

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "status_codes.h"
#include "crypto.h"
#include "base64.h"
#include "mem.h"
#include "io_stream.h"
#include "sha1.h"
#include "flash_fifo.h"

#define WEBSOCKET_HANDSHAKE_GET           "GET %s HTTP/1.1\r\n"
#define WEBSOCKET_HANDSHAKE_HOST          "Host: %s\r\n"
#define WEBSOCKET_HANDSHAKE_UPGRADE       "Upgrade: websocket\r\n"
#define WEBSOCKET_HANDSHAKE_CONNECTION    "Connection: Upgrade\r\n"
#define WEBSOCKET_HANDSHAKE_KEY           "Sec-WebSocket-Key: %s\r\n"
#define WEBSOCKET_HANDSHAKE_ORIGIN        "Origin: %s\r\n"
#define WEBSOCKET_HANDSHAKE_PROTOCOL      "Sec-WebSocket-Protocol: metormote\r\n"
#define WEBSOCKET_HANDSHAKE_VERSION       "Sec-WebSocket-Version: 13\r\n"

#define WEBSOCKET_HANDSHAKE_RESPONSE      "HTTP/1.1 101" // Switching Protocols\r\n"
#define WEBSOCKET_HEADER_ACCEPT           "Sec-WebSocket-Accept: "
#define WEBSOCKET_HEADER_ACCEPT_KEY_LEN   28

#define WEBSOCKET_GUID                    "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

#define WEBSOCKET_OPCODE_CONTINUATION     0x0
#define WEBSOCKET_OPCODE_TEXT             0x1
#define WEBSOCKET_OPCODE_BINARY           0x2
#define WEBSOCKET_OPCODE_CLOSE            0x8
#define WEBSOCKET_OPCODE_PING             0x9
#define WEBSOCKET_OPCODE_PONG             0xA


#define WEBSOCKET_STATE_CLOSED        0
#define WEBSOCKET_STATE_CONNECTING    1
#define WEBSOCKET_STATE_OPEN          2
#define WEBSOCKET_STATE_CLOSING       3

#define WEBSOCKET_READ_TIMEOUT            5*SYSTEM_TICKS_PER_SEC
#define WEBSOCKET_KEEPALIVE_INTERVAL      5*SYSTEM_TICKS_PER_SEC
#define WEBSOCKET_KEEPALIVE_GRACE_PERIOD  3

struct websocket_options_t {
  char *path;
  char *host;
  char *origin;
};

struct websocket_t {
  uint8_t id;
  uint8_t state;
  struct websocket_options_t *options;
  void (*on_message)(uint8_t *msg, uint32_t len) ;
  int8_t (*read_socket)(uint8_t socket_id, uint8_t *data, uint16_t len, int32_t timeout);
  int8_t (*write_socket)(uint8_t socket_id, uint8_t *data, uint16_t len);
  void (*data_callback)(uint16_t counter, uint8_t *data, uint16_t len);
  uint8_t ping_count;
};


struct websocket_frame_t {
  uint8_t opcode:4, rsv3:1, rsv2:1, rsv1:1, fin:1;
  uint8_t payload_len:7, mask:1;
};

int8_t websocket_start(struct websocket_t *websocket);
int8_t websocket_close(struct websocket_t *websocket);
int8_t websocket_send_text(struct websocket_t *websocket, char *txt);
int8_t websocket_send_binary(struct websocket_t *websocket, io_input_stream_t *stream, uint16_t len);
int8_t websocket_send_ping(struct websocket_t *websocket);

#endif /* WEBSOCKET_H_ */