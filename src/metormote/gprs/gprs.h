#ifndef GPRS_H
#define GPRS_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "gprs_core.h"
#include "atom.h"
#include "atomsem.h"

#define  GPRS_EVENT_NEW_SMS                     5
//#define  GPRS_EVENT_SMS_READ                    6
#define  GPRS_EVENT_SMS_SENT                    7


#define  GPRS_STATE_UNDEFINED                          0
#define  GPRS_STATE_INITIAL                            1
#define  GPRS_STATE_READY                              2
#define  GPRS_STATE_SOCKET_OPEN                        4

/*! \def HTTP_HEADER_CRYPTO_IV

    \brief The header name for AES128 initial vector
 */
#define HTTP_HEADER_CRYPTO_IV          "x-metormote-iv: "
//#define HTTP_HEADER_CRYPTO_IV          "x-ubisense-iv: "

/*! \def HTTP_HEADER_CONTENT_LENGTH

    \brief The header name content length
 */
#define HTTP_HEADER_CONTENT_LENGTH     "Content-Length: "


#ifdef __cplusplus
extern "C" { 
#endif

struct gprs_options {
  char *apn;
  char *username;
  char *password;
  char *host;
  char *path;
  uint16_t port;
};

struct sms_data {
  char origin[15];
  char text[160];
};


void gprs_set_event_listener(void (*callback)(uint8_t));
int8_t gprs_init(struct gprs_options *options);
int8_t gprs_reset(struct gprs_options *options);
int8_t gprs_check_network(void);

int8_t gprs_send_sms(char *dest, char *text);
int8_t gprs_read_sms(uint8_t index);
int8_t gprs_open_socket(struct gprs_options *options);
int8_t gprs_close_socket(void);
int8_t gprs_send_data(char *data);
int8_t gprs_send_http_post(char *url, char* mime, int32_t length, char* iv);
int8_t gprs_send_http_get(char *url);
int8_t gprs_read_data(char *buf, uint16_t length);
int8_t gprs_read_until(char *wait_for_string);
int8_t gprs_write_socket(uint8_t socket_id, uint8_t *data, uint16_t len);
int8_t gprs_read_socket(uint8_t socket_id, uint8_t *data, uint16_t len, int32_t timeout);
int8_t gprs_flush_socket(uint8_t socket_id);

int8_t gprs_battery(int16_t *voltage);
int8_t gprs_temperature(int16_t *temperature);

#ifdef __cplusplus
}
#endif


#endif /* GPRS_H */
