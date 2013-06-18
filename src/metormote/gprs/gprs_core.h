#ifndef GPRS_CORE_H
#define GPRS_CORE_H

#include <stdint.h>
#include <stddef.h>
#include <avr/interrupt.h>
#include <usart.h>
#include <status_codes.h>
#include <board.h>
#include <gpio.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include "atom.h"
#include "atomqueue.h"
#include "atomsem.h"

#define AT_TEMINATION_CHAR               '\r'
#define AT_NEW_LINE_CHAR                 '\n'
#define AT_SMS_FLUSH_CHAR                '\x1A'
#define AT_FLUSH_CHAR                    '\x1B'


//GENERAL
#define AT_CHECK_CONNECT                 "AT"
#define AT_FLOW_CTRL_OFF                 "AT&K3"
#define AT_ECHO_OFF                      "ATE0"
#define AT_PORT_SPEED_SETTING            "AT+IPR=0"
#define AT_DISABLE_VERBOSE_ERROR         "AT+CMEE=1"
#define AT_COMMAND_INTERFACE             "AT#SELINT=2"
#define AT_SET_REGISTRATION_MODE         "AT#REGMODE=1"
#define AT_ENABLE_QUERY_SIM_STATUS       "AT#QSS=2"
#define AT_CHECK_SIM_INSERTED            "AT+CPIN?"
#define AT_SIM_PIN_CODE                  "AT+CPIN=1234"
#define AT_ENABLE_NETWORK_REG_STATUS     "AT+CREG=1"
#define AT_QUERY_NETWORK_STATUS          "AT+CREG?"
#define AT_QUERY_SIGNAL_QUALITY          "AT+CSQ"
#define AT_QUERY_CELL_INFO               "AT#MONI=7"
#define AT_SET_DATETIME                  "AT#CCLK=\"02/09/07,22:30:00+04,1\""
#define AT_AUTO_UPDATE_DATETIME          "AT#NITZ=15,1"
#define AT_CHECK_BALANCE                 "AT+CUSD=1,\"*120#\""

//SMS
#define AT_SET_SMS_TEXT_MODE             "AT+CMGF=1"
#define AT_SET_SMS_COMMAND_MODE          "AT#SMSMODE=0"
#define AT_SET_SMS_STORE_MODE            "AT+CPMS=\"ME\""
#define AT_SMS_NEW_MSG_SETTING           "AT+CNMI=2,1,0,0,0"
#define AT_SMS_TEXT_MODE_PARAM           "AT+CSMP= 17,167,0,0"
#define AT_SEND_SMS                      "AT+CMGS=\"%s\",145"
#define AT_READ_SMS                      "AT+CMGR=%d"
#define AT_DELETE_SMS                    "AT+CMGD=1,4"
#define AT_LIST_SMS                      "AT+CMGL=0"


//RESPONSES
#define AT_RESP_NEW_SMS                  "+CMTI: \"ME\",1"
#define AT_RESP_SMS_SENT                 "+CMGS: <mr>"
#define AT_RESP_SMS_PROMPT               "> "

//GPRS
#define AT_GPRS_CONTEXT_SETTING          "AT+CGDCONT = 1,\"IP\",\"%s\",\"0.0.0.0\",0,0"
#define AT_GPRS_SOCKET_SETTING           "AT#SCFG=1,1,300,60,100,1"
#define AT_GPRS_SOCKET_ACTIVATE          "AT#SGACT=1,1,\"%s\",\"%s\""
#define AT_GPRS_SOCKET_DEACTIVATE        "AT#SGACT=1,0"
#define AT_GPRS_SOCKET_OPEN              "AT#SD=1,0,%d,\"%s\",0,0"
#define AT_GPRS_SOCKET_CLOSE             "AT#SH=1"
#define AT_GPRS_SOCKET_STATUS            "AT#SS=1"

//BATTERY & TEMPERATURE
#define AT_GPRS_BATTERY_STATUS           "AT#CBC"
#define AT_GPRS_TEMPERATURE              "AT#TEMPMON=1"

#define AT_ESCAPE_SEQUENCE               "+++"
#define AT_CRLN                          "\r\n"

//RESPONSES
#define AT_RESP_OK                       "\r\nOK\r\n"
#define AT_RESP_CONNECT                  "\r\nCONNECT\r\n"
#define AT_RESP_ERROR                    "\r\nERROR\r\n"
#define AT_RESP_NO_CARRIER               "\r\nNO CARRIER\r\n"
#define AT_RESP_SOCKET_CLOSED            "\r\n#SS: 1,0\r\n\r\nOK\r\n"

#define AT_RESP_CME_ERROR                "\r\n+CME ERROR: <err>\r\n"
#define AT_RESP_CMS_ERROR                "\r\n+CMS ERROR: <err>\r\n"



#define GPRS_CORE_RX_BUF_SIZE                 256
#define GPRS_CORE_TX_BUF_SIZE                 64
#define GPRS_CORE_RX_TIMEOUT                  30*SYSTEM_TICKS_PER_SEC
#define GPRS_CORE_TX_TIMEOUT                  30*SYSTEM_TICKS_PER_SEC
#define GPRS_CORE_COMMAND_TIMEOUT              30*SYSTEM_TICKS_PER_SEC
 
//#define  GPRS_CORE_EVENT_RESPONSE_OK                 1
#define  GPRS_CORE_EVENT_NEW_SMS                     2
//#define  GPRS_CORE_EVENT_TX_OK                       3
//#define  GPRS_CORE_EVENT_RESPONSE_ERROR              4
//#define  GPRS_CORE_EVENT_RESPONSE_TIMEOUT            5


#ifdef __cplusplus
extern "C" { 
#endif


extern ATOM_QUEUE gprs_rx_buf;

extern ATOM_QUEUE gprs_tx_buf;

//extern FIFO_BUFFER gprs_rx_buf;

//extern FIFO_BUFFER gprs_tx_buf;

void gprs_core_set_event_listener(void (*callback)(uint8_t));
int8_t gprs_core_init(void);
void gprs_core_reset(void);
int8_t gprs_core_on(void);
void gprs_core_off(void);
int8_t gprs_core_send_command(char *command, char* wait_for_response);
int8_t gprs_core_send_string(char *data);
int8_t gprs_core_send_char(char data);
int8_t gprs_core_flush_data(char* wait_for_response, uint32_t timeout);
int8_t gprs_core_read_char(char* data, uint32_t timeout);
void gprs_core_clear_rx_buffer(void);
void gprs_core_clear_tx_buffer(void);

#ifdef __cplusplus
}
#endif


#endif /* GPRS_CORE_H */
