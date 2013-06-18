#include "gprs.h"

//global data
struct sms_data sms;

//local data
static volatile uint8_t gprs_state;

//forward declarations
static void gprs_event_handler(uint8_t event);
static int8_t gprs_config(struct gprs_options *options);

//callback for gprs events
static void (*gprs_callback)(uint8_t event);

void gprs_set_event_listener(void (*callback)(uint8_t)) {
  gprs_callback=callback;
}

int8_t gprs_init(struct gprs_options *options) {
  int8_t status;
  
  if((status=gprs_core_init())!=STATUS_OK) {
    return status;
  }
  
  if((status=gprs_config(options))!=STATUS_OK) {
    return status;
  }
  
  return STATUS_OK;
}

int8_t gprs_reset(struct gprs_options *options) {
  int8_t status;
  
  gprs_core_reset();
  
  if((status=gprs_config(options))!=STATUS_OK) {
    return status;
  }
  
  return STATUS_OK;
}

static int8_t gprs_config(struct gprs_options *options) {
  char command[64];
  
  gprs_state=GPRS_STATE_INITIAL;
  
  strcpy(command, AT_CHECK_CONNECT);
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  
  strcpy_P(command, PSTR(AT_ECHO_OFF));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }

  strcpy_P(command, PSTR(AT_FLOW_CTRL_OFF));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  
  strcpy_P(command, PSTR(AT_PORT_SPEED_SETTING));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }

  strcpy_P(command, PSTR(AT_DISABLE_VERBOSE_ERROR));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }

  strcpy_P(command, PSTR(AT_COMMAND_INTERFACE));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }

  strcpy_P(command, PSTR(AT_SET_REGISTRATION_MODE));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }

  strcpy_P(command, PSTR(AT_CHECK_SIM_INSERTED));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }

  strcpy_P(command, PSTR(AT_SET_SMS_TEXT_MODE));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }

  strcpy_P(command, PSTR(AT_SET_SMS_COMMAND_MODE));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }

  strcpy_P(command, PSTR(AT_SET_SMS_STORE_MODE));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }

  strcpy_P(command, PSTR(AT_SMS_NEW_MSG_SETTING));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }

  strcpy_P(command, PSTR(AT_SMS_TEXT_MODE_PARAM));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  
  sprintf_P(command, PSTR(AT_GPRS_CONTEXT_SETTING), options->apn);
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }

  strcpy_P(command, PSTR(AT_GPRS_SOCKET_SETTING));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  /*
  sprintf_P(command, PSTR(AT_GPRS_SOCKET_ACTIVATE), options->username, options->password);
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  */
  gprs_core_set_event_listener(gprs_event_handler);
  
  gprs_state=GPRS_STATE_READY;
  return STATUS_OK;
}


static void gprs_event_handler(uint8_t event) {
  
  if(event==GPRS_CORE_EVENT_NEW_SMS) {
    gprs_read_sms(1);
    //todo read message number
  }
  
}


int8_t gprs_battery(int16_t *voltage) {
  char command[8];
  char res[16];
  char *c;
  
  strcpy_P(command, PSTR(AT_GPRS_BATTERY_STATUS));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  if(gprs_read_until(",")!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  
  c=res;
  while(gprs_core_read_char(c, GPRS_CORE_RX_TIMEOUT)==STATUS_OK) {
    if(*c=='\r') {
      *c='\0';
      *voltage=atoi(res);
      return STATUS_OK;
    }
    c++;    
  }
  *voltage=0;
  return ERR_TIMEOUT;
}


int8_t gprs_temperature(int16_t *temperature) {
  char command[16];
  char res[16];
  char *c;
  
  strcpy_P(command, PSTR(AT_GPRS_TEMPERATURE));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  if(gprs_read_until(",")!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  
  c=res;
  while(gprs_core_read_char(c, GPRS_CORE_RX_TIMEOUT)==STATUS_OK) {
    if(*c=='\r') {
      *c='\0';
      *temperature=atoi(res);
      return STATUS_OK;
    }
    c++;    
  }
  *temperature=0;
  return ERR_TIMEOUT;
}



int8_t gprs_check_network() {
  char command[16];
  strcpy_P(command, PSTR(AT_QUERY_NETWORK_STATUS));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  //todo read network status
  return STATUS_OK;
}


int8_t gprs_send_sms(char *dest, char *text) {
  char command[32];
  strcpy(sms.text, text);
  sprintf_P(command, PSTR(AT_SEND_SMS), dest);
  if(gprs_core_send_command(command, AT_RESP_SMS_PROMPT)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  
  gprs_core_send_string(sms.text);
  gprs_core_send_char(AT_SMS_FLUSH_CHAR);
  if(gprs_core_flush_data(AT_RESP_OK, GPRS_CORE_COMMAND_TIMEOUT)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  return STATUS_OK;
}


int8_t gprs_read_sms(uint8_t index) {
  int i;
  char c;
  char command[20];
  sprintf_P(command, PSTR(AT_READ_SMS), index);
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  i=-1;
  while(gprs_core_read_char(&c, GPRS_CORE_RX_TIMEOUT)==STATUS_OK) {
    //c=FIFO_Get(&gprs_rx_buf);
    if(i==-1 && c=='\n') i=0;
    else if(c=='\n') break;
    else if(i>=0) sms.text[i++]=c;
  }
  if(i>0) sms.text[i]='\0';
  else sms.text[0]='\0';
  return STATUS_OK;
}


int8_t gprs_open_socket(struct gprs_options *options) {
  char command[64];
  
  strcpy_P(command, PSTR(AT_QUERY_NETWORK_STATUS));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  
  
  sprintf_P(command, PSTR(AT_GPRS_SOCKET_ACTIVATE), options->username, options->password);
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  
  
  sprintf_P(command, PSTR(AT_GPRS_SOCKET_OPEN), options->port, options->host);
  if(gprs_core_send_command(command, AT_RESP_CONNECT)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  
  gprs_state=GPRS_STATE_SOCKET_OPEN;
  return STATUS_OK;
}


int8_t gprs_close_socket() {
  char command[16];
  
  atomTimerDelay(2*SYSTEM_TICKS_PER_SEC);
  gprs_core_clear_tx_buffer();
  gprs_core_send_string(AT_ESCAPE_SEQUENCE);
  if(gprs_core_flush_data(AT_RESP_OK, 3*SYSTEM_TICKS_PER_SEC)!=STATUS_OK) {
    if(gprs_core_send_command(AT_CHECK_CONNECT, AT_RESP_OK)!=STATUS_OK) {
      return ERR_IO_ERROR;
    }
  }
  
  strcpy_P(command, PSTR(AT_GPRS_SOCKET_CLOSE));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    //return ERR_TIMEOUT;
  }
  
  //wait for close
  strcpy_P(command, PSTR(AT_GPRS_SOCKET_STATUS));
  if(gprs_core_send_command(command, AT_RESP_SOCKET_CLOSED)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  
  
  strcpy_P(command, PSTR(AT_GPRS_SOCKET_DEACTIVATE));
  if(gprs_core_send_command(command, AT_RESP_OK)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  
  
  gprs_state=GPRS_STATE_READY;
  return STATUS_OK;
}



int8_t gprs_send_data(char *data) {
  if(gprs_state!=GPRS_STATE_SOCKET_OPEN) {
    return ERR_PROTOCOL;
  }
  gprs_core_send_string(data);
  if(gprs_core_flush_data(NULL, GPRS_CORE_TX_TIMEOUT)!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  return STATUS_OK;
}


int8_t gprs_read_data(char *buf, uint16_t length) {
  uint16_t len;
  char c;
  for(len=0;len<length;len++) {
    if(gprs_core_read_char(&c, GPRS_CORE_RX_TIMEOUT)==STATUS_OK) {
      buf[len]=c;
    }
    else {
      return ERR_TIMEOUT;
    }
  }
  
  return STATUS_OK;
}

int8_t gprs_write_socket(uint8_t socket_id, uint8_t *data, uint16_t len) {
  if(gprs_state!=GPRS_STATE_SOCKET_OPEN) {
    return ERR_PROTOCOL;
  }
  
  for(;len!=0;len--) {
    if(gprs_core_send_char((char)*data++)!=STATUS_OK) {
      return ERR_TIMEOUT;
    }
    
    //start transfer
    usart_set_dre_interrupt_level(GPRS_USART, USART_DREINTLVL_LO_gc);
  }
  
  return STATUS_OK;
}


int8_t gprs_read_socket(uint8_t socket_id, uint8_t *data, uint16_t len, int32_t timeout) {
  
  for(;len!=0;len--) {
    if(gprs_core_read_char((char *)data++, timeout)!=STATUS_OK) {
      return ERR_TIMEOUT;    
    }
  }  
  return STATUS_OK;
}

int8_t gprs_flush_socket(uint8_t socket_id) {
  return STATUS_OK;
}



int8_t gprs_send_http_post(char *url, char* mime, int32_t length, char* iv)
{
  char buf[64];
  if(gprs_state!=GPRS_STATE_SOCKET_OPEN) {
    return ERR_PROTOCOL;
  }
  sprintf_P(buf,PSTR("POST %s HTTP/1.0%c%c"),url,'\r','\n');
  gprs_send_data(buf);
  sprintf_P(buf,PSTR("Content-Type: %s%c%c"),mime,'\r','\n');
  gprs_send_data(buf);
  sprintf_P(buf,PSTR("Content-Length: %ld%c%c"),length,'\r','\n');
  gprs_send_data(buf);
  sprintf_P(buf,PSTR("%s%s%c%c"),HTTP_HEADER_CRYPTO_IV,iv,'\r','\n');
  gprs_send_data(buf);
  gprs_send_data("\r\n");
  return STATUS_OK;
}


int8_t gprs_send_http_get(char *url) {
  char buf[64];
  sprintf_P(buf,PSTR("GET %s HTTP/1.0%c%c"),url,'\r','\n');
  gprs_send_data(buf);
  gprs_send_data("\r\n");
  return STATUS_OK;
}


int8_t gprs_read_until(char *wait_for_string) {
  char c;
  char *p;
  
  if(wait_for_string==NULL) {
    return ERR_INVALID_ARG;
  }
  
  p=wait_for_string;
  for(;;) {
    if(gprs_core_read_char(&c, GPRS_CORE_RX_TIMEOUT)==STATUS_OK) {
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






