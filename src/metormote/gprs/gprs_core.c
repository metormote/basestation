#include "gprs_core.h"

// GPRS USART options.
static usart_rs232_options_t GPRS_USART_OPTIONS = {
    .baudrate = 115200,
    .charlength = USART_CHSIZE_8BIT_gc,
    .paritytype = USART_PMODE_DISABLED_gc,
    .stopbits = false
};

ATOM_QUEUE gprs_rx_buf;
static uint8_t rx_data_store[GPRS_CORE_RX_BUF_SIZE];
//static volatile uint8_t gprs_rx_tmp[32];
//static volatile uint8_t gprs_rx_ix;

ATOM_QUEUE gprs_tx_buf;
static uint8_t tx_data_store[GPRS_CORE_TX_BUF_SIZE];

static ATOM_SEM gprs_sem;
static char wait_for_string[32];
static char *wait_index;

static volatile uint8_t new_sms_index;

//forward declarations
static inline void gprs_tx_interrupt_handler(void);
static inline void gprs_dre_interrupt_handler(void);
static inline void gprs_rx_interrupt_handler(void);


//interrupt handler for gprs receive complete
ISR(USARTC0_RXC_vect)
{
  atomIntEnter();
  gprs_rx_interrupt_handler();
  atomIntExit(FALSE, TRUE);
}

//interrupt handler for ant gprs register empty
ISR(USARTC0_DRE_vect)
{
  atomIntEnter();
  gprs_dre_interrupt_handler();
  atomIntExit(FALSE, TRUE);
}

//interrupt handler for gprs transmit complete
ISR(USARTC0_TXC_vect)
{
  atomIntEnter();
  gprs_tx_interrupt_handler();
  atomIntExit(FALSE, TRUE);
}

//interrupt handler for gprs status
ISR(GPRS_STATUS_INT_VECTOR)
{
  ioport_toggle_pin_level(GPIO_LED_YELLOW);
}

//callback for unsolicited notifications (sms receive etc)
static void (*gprs_core_callback)(uint8_t event);

void gprs_core_set_event_listener(void (*callback)(uint8_t)) {
  gprs_core_callback=callback;
}


int8_t gprs_core_init() {
  wait_for_string[0]='\0';
  wait_index=wait_for_string;
  
  if (atomSemCreate (&gprs_sem, 0) != ATOM_OK)
  {
    return ERR_INVALID_ARG;
  }
  
  //initialize ant usart driver in RS232 mode
  usart_init_rs232(GPRS_USART, &GPRS_USART_OPTIONS);
  
  //set GPRS_STATUS pin 0 as source of interrupts 
  GPRS_STATUS_PORT->INT1MASK = GPRS_STATUS_PORT->INT1MASK | ioport_pin_to_mask(GPRS_STATUS);
  //set interrupt level
  GPRS_STATUS_PORT->INTCTRL = (GPRS_STATUS_PORT->INTCTRL & ~PORT_INT1LVL_gm) | PORT_INT1LVL_LO_gc;
  //sense both edges
  GPRS_STATUS_PORT->PIN0CTRL = IOPORT_SENSE_RISING;
  
  ioport_set_pin_level(GPRS_ON_OFF, false);
  ioport_set_pin_level(GPRS_RESET, false);
  
  
  if (atomQueueCreate (&gprs_rx_buf, rx_data_store, sizeof(uint8_t), GPRS_CORE_RX_BUF_SIZE) != ATOM_OK)
  {
    return ERR_INVALID_ARG;
  }

  if (atomQueueCreate (&gprs_tx_buf, tx_data_store, sizeof(uint8_t), GPRS_CORE_TX_BUF_SIZE) != ATOM_OK)
  {
    return ERR_INVALID_ARG;
  }
  
  usart_set_rx_interrupt_level(GPRS_USART, USART_RXCINTLVL_LO_gc);
  usart_set_tx_interrupt_level(GPRS_USART, USART_TXCINTLVL_LO_gc);
    
  return gprs_core_on();
}

void gprs_core_reset()
{
  ioport_set_pin_level(GPRS_RESET, true);
  atomTimerDelay(SYSTEM_TICKS_PER_SEC/4);
  ioport_set_pin_level(GPRS_RESET, false);
  atomTimerDelay(16*SYSTEM_TICKS_PER_SEC);
  gprs_core_clear_rx_buffer();
  gprs_core_clear_tx_buffer();
  return;
}

int8_t gprs_core_on(void)
{
  if(!ioport_get_pin_level(GPRS_POWER_MON)) {
    ioport_set_pin_level(GPRS_ON_OFF, true);
    atomTimerDelay(SYSTEM_TICKS_PER_SEC);
    ioport_set_pin_level(GPRS_ON_OFF, false);
    atomTimerDelay(SYSTEM_TICKS_PER_SEC);
    //if power monitor is still low try longer signal period
    if(!ioport_get_pin_level(GPRS_POWER_MON)) {
        ioport_set_pin_level(GPRS_ON_OFF, true);
        atomTimerDelay(4*SYSTEM_TICKS_PER_SEC);
        ioport_set_pin_level(GPRS_ON_OFF, false);
        atomTimerDelay(SYSTEM_TICKS_PER_SEC);
    }      
    //if power monitor pin is still low we have no gprs module
    if(!ioport_get_pin_level(GPRS_POWER_MON)) {
        return ERR_UNSUPPORTED_DEV;
    }      
    //wait for sim-card initialization
    atomTimerDelay(16*SYSTEM_TICKS_PER_SEC);
    gprs_core_clear_tx_buffer();
  }      
  else {
    //if device is already on, just reset it
    gprs_core_reset();
  }
  return STATUS_OK;
}

void gprs_core_off(void)
{
  ioport_set_pin_level(GPRS_ON_OFF, true);
  atomTimerDelay(2*SYSTEM_TICKS_PER_SEC);
  ioport_set_pin_level(GPRS_ON_OFF, false);
  atomTimerDelay(4*SYSTEM_TICKS_PER_SEC);
  return;
}



static inline void gprs_tx_interrupt_handler() {
  //buffer transferred
  /*if(wait_for_string[0]=='\0') {
    //if we are not waiting for a specific string notify caller
    //that we are ready
    atomSemPut(&gprs_sem);
  }*/
  //gprs_rx_ix=0;
}


static inline void gprs_dre_interrupt_handler() {
  uint8_t tx_byte;
  
  //read one byte from the queue and check that the queue is not empty
  if(atomQueueGet(&gprs_tx_buf, -1, &tx_byte)!=ATOM_WOULDBLOCK) {
    usart_putchar(GPRS_USART, tx_byte);
  }
  else {
    usart_set_dre_interrupt_level(GPRS_USART, USART_DREINTLVL_OFF_gc);
  }
  
  return;
}


static inline void gprs_rx_interrupt_handler() {
  uint8_t gprs_rx_byte, gprs_dummy_byte;
  
  gprs_rx_byte = usart_getchar(GPRS_USART);
  //gprs_rx_tmp[gprs_rx_ix++]=gprs_rx_byte;
  
  if(atomQueuePut(&gprs_rx_buf, -1, &gprs_rx_byte)==ATOM_WOULDBLOCK) {
    //if buffer is full, remove one element
    atomQueueGet(&gprs_rx_buf, -1, &gprs_dummy_byte);
    atomQueuePut(&gprs_rx_buf, -1, &gprs_rx_byte);
  }
  
  if(wait_for_string[0]!='\0') {
    if(gprs_rx_byte==*wait_index) {
      wait_index++;
      if(*wait_index=='\0') {
        wait_for_string[0]='\0';
        atomSemPut(&gprs_sem);
      }
    }
    else if(gprs_rx_byte==wait_for_string[0]) {
      wait_index=wait_for_string;
      wait_index++;
    }
    else {
      wait_index=wait_for_string;
    }
  }
  
  if(gprs_rx_byte==AT_RESP_NEW_SMS[new_sms_index]) {
    new_sms_index++;
    if(AT_RESP_NEW_SMS[new_sms_index]=='\0') {
      new_sms_index=0;
      gprs_core_callback(GPRS_CORE_EVENT_NEW_SMS);  
    }
  }
  else if(gprs_rx_byte==AT_RESP_NEW_SMS[0]) {
    new_sms_index=1;
  }
  else {
    new_sms_index=0;
  }
  
  return;
}


void gprs_core_clear_rx_buffer() {
  uint8_t rx_byte;
  while(atomQueueGet(&gprs_rx_buf, -1, &rx_byte)!=ATOM_WOULDBLOCK) ;
}

void gprs_core_clear_tx_buffer() {
  uint8_t tx_byte;
  while(atomQueueGet(&gprs_tx_buf, -1, &tx_byte)!=ATOM_WOULDBLOCK) ;
}


int8_t gprs_core_send_command(char *command, char* wait_for_response) {
  int8_t status;
  
  //add command to transmit buffer
  gprs_core_send_string(command);
  gprs_core_send_char(AT_TEMINATION_CHAR);
  //gprs_core_send_char(AT_NEW_LINE_CHAR);
  
  //start transfer
  status=gprs_core_flush_data(wait_for_response, GPRS_CORE_COMMAND_TIMEOUT);
  
  return status;
}


int8_t gprs_core_send_string(char *data) {
  //add data to transmit buffer
  while(*data) {
    if(atomQueuePut(&gprs_tx_buf, GPRS_CORE_TX_TIMEOUT, (uint8_t*)data++)!=ATOM_OK) {
      return ERR_TIMEOUT;
    }      
  }
  return STATUS_OK;
}

int8_t gprs_core_send_char(char data) {
  //add data to transmit buffer
  if(atomQueuePut(&gprs_tx_buf, GPRS_CORE_TX_TIMEOUT, (uint8_t*)&data)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  return STATUS_OK;
}

int8_t gprs_core_read_char(char* data, uint32_t timeout) {
  if(atomQueueGet(&gprs_rx_buf, timeout, (uint8_t*)data)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  return STATUS_OK;
}

int8_t gprs_core_flush_data(char* wait_for_response, uint32_t timeout) {
  int i;
  //reset match variables
  wait_index=wait_for_string;
  if(wait_for_response!=NULL) {
    //copy response string
    i=0;
    while(*wait_for_response) {
      wait_for_string[i++]=*wait_for_response++;
    }
    wait_for_string[i]='\0';
  }
  else {
    wait_for_string[0]='\0';
  }
  
  //clear rx buffer
  gprs_core_clear_rx_buffer();
  
  //start transfer
  usart_set_dre_interrupt_level(GPRS_USART, USART_DREINTLVL_LO_gc);
  
  //wait for response
  if(wait_for_response!=NULL) {
    if(atomSemGet(&gprs_sem, timeout)!=ATOM_OK) {
      gprs_core_clear_tx_buffer();
      wait_for_string[0]='\0';
      atomSemResetCount (&gprs_sem, 0);
      return ERR_TIMEOUT;
    }
  }  
  return STATUS_OK;
}

