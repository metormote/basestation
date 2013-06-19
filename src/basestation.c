/*
 * basestation.c
 *
 * Created: 10/4/2011 10:10:41 AM
 *  Author: Administrator
 */ 
#include "basestation.h"

/* Local data */
#define STATE_EEPROM_PAGE_ADDRESS   0


/* Idle thread's stack area */
static uint8_t *idle_thread_stack=(uint8_t *)RAMEND;

/* ANT thread's stack area */
static uint8_t *ant_thread_stack=(uint8_t *)(RAMEND-IDLE_THREAD_STACK_SIZE_BYTES);

/* Process thread's stack area */
static uint8_t *process_thread_stack=(uint8_t *)(RAMEND-IDLE_THREAD_STACK_SIZE_BYTES
                                                 -ANT_THREAD_STACK_SIZE_BYTES);

/* GPRS thread */
static uint8_t *gprs_thread_stack=(uint8_t *)(RAMEND-IDLE_THREAD_STACK_SIZE_BYTES
                                                -ANT_THREAD_STACK_SIZE_BYTES
                                                -PROCESS_THREAD_STACK_SIZE_BYTES);
                                                
/* Websocket thread */
static uint8_t *websocket_thread_stack=(uint8_t *)(RAMEND-IDLE_THREAD_STACK_SIZE_BYTES
                                                  -ANT_THREAD_STACK_SIZE_BYTES
                                                  -PROCESS_THREAD_STACK_SIZE_BYTES
                                                  -GPRS_THREAD_STACK_SIZE_BYTES);


/* Application threads' TCBs */
static ATOM_TCB ant_tcb;
static ATOM_TCB gprs_tcb;
static ATOM_TCB process_tcb;
static ATOM_TCB websocket_tcb;

static ATOM_SEM tx_sem;
static ATOM_SEM websocket_sem;
static ATOM_MUTEX websocket_mutex;

/* Forward declarations */
static void ant_thread_func (uint32_t data);
static void ant_callback(ant_event_t evt, uint8_t *data, uint16_t len);
static void process_callback_data(uint8_t *data, uint16_t len);
static void gprs_thread_func (uint32_t data);
static int8_t gprs_websocket_handler (void);
static void websocket_thread_func(uint32_t data);
static void process_thread_func(uint32_t data);
static int8_t net_websocket_handler (void);
static void idle_thread_callback (void);
static void ui_listener(uint8_t event);
static bool write_flash_fifo_cb(pb_ostream_t *stream, const uint8_t *buf, size_t count);
static int8_t read_flash_fifo_cb(io_input_stream_t *stream, uint8_t *buf, uint16_t count);
static void on_message(uint8_t *buf, uint32_t len);
static int8_t process_rx_data(uint8_t *buf, uint16_t len);
static int8_t process_fw_data(uint32_t len);
static bool decode_hex_callback(pb_istream_t *stream, const pb_field_t *field, void *arg);
static void message_data_callback(uint16_t counter, uint8_t *buf, uint16_t len);
static int8_t snapshot_state(void);
static bool encode_topology_callback(pb_ostream_t *stream, const pb_field_t *field, const void *arg);
static bool encode_node_callback(pb_ostream_t *stream, const pb_field_t *field, const void *arg);
static int8_t confirm_fw_upload(int8_t status);
static int8_t send(pb_bytes_array_t *env_bytes, uint64_t msg_code, uint8_t *msg_buf, uint16_t len);
static void read_mac(uint8_t *mac);
static void read_state(void);
static void save_state(void);


static uint32_t keydown;
static uint32_t fw_upload_len;
static uint32_t next_snapshot;

static struct websocket_t websocket;

static struct basestation_signature_t signature = {
  .device_id=0x00,
  .key={0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F},
  .temp_offset=0,
};

static struct basestation_state_t state = {
  .network_id=0x0,
  .transfer_interval=10,
  .snapshot_interval=60,
  .send_immediate=FALSE,
  .rx_nonce=0,
  .tx_nonce=0
};


FLASH_FIFO FIFO = {
    .memidx = 0,
    .offset = 0,
    .head = 0,
    .tail = 0,
    .buffer_len = FIFO_BUFFER_LENGTH,
    .buffer_no = FIFO_BUFFER_NO
};

FLASH_FIFO WEBSOCKET_FIFO = {
  .memidx = 0,
  .offset = WEBSOCKET_FIFO_BUFFER_OFFSET,
  .head = 0,
  .tail = 0,
  .buffer_len = WEBSOCKET_FIFO_BUFFER_LENGTH,
  .buffer_no = WEBSOCKET_FIFO_BUFFER_NO
};

struct net_options_t NET_OPTIONS = {
  .mac = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  .ip = {0, 0, 0, 0},
  .mask = {0, 0, 0, 0},
  .gw = {0, 0, 0, 0},
  .dns = {{8, 8, 8, 8}, {8, 8, 4, 4}},
  .dhcp_on = FALSE,
};

struct net_client_socket_options_t NET_CLIENT_OPTIONS = {
  .ip = {79, 125, 105, 194},
  //.ip = {0, 0, 0, 0},
  .port = 80,
  //.ip = {192, 168, 0, 172},
  //.port = 3001,
  .timeout = 20*SYSTEM_TICKS_PER_SEC,
};

static struct gprs_options GPRS_OPTIONS = {
    .apn = "internet.telia.se",
    .username = "",
    .password = "",
    //.host = "79.125.105.194",
    .host = "gw.iop.io",
    //.port = 3001
    .port = 80
};

static struct websocket_options_t WEBSOCKET_OPTIONS = {
    .path = "/",
    .host = "gw.iop.io",
    //.host = "192.168.0.172",
    .origin = "0000000000000000"
};

static void init(void) {
    //initialize interrupt controller
    pmic_init();
    
    //init io ports
    ioport_init();
    
    //initialize board
    board_init();
    
    //initialize system clock
    sysclk_init();
    ccp_write_io((uint8_t *)&OSC_DFLLCTRL, 0x01);
    //ccp_write_io((uint8_t *)&OSC.XOSCFAIL, 0x01);
    
    //enable the AES clock
    sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_AES);
    
    //initialize sleep manager
    sleepmgr_init();
    
    //initialize real time counter
    osc_enable(OSC_ID_RC32KHZ);
    rtc_init();
    
    //initialize data flash fifo
    flash_fifo_init();
    
    atomSemCreate(&websocket_sem, 0);
    atomSemCreate(&tx_sem, 0);
    atomMutexCreate(&websocket_mutex);
    
    nvm_user_sig_read_buffer(0, &signature, sizeof(struct basestation_signature_t));
    
    sprintf(WEBSOCKET_OPTIONS.origin, "%lu", (uint32_t)signature.device_id);
    
    //seed random generator
    seed();
    
    read_state();
    
    //initialize crypto module
    crypto_init(signature.key);
    
    //init the leds and button
    ui_init();
    ui_set_ui_listener(ui_listener);
    
    power_init();
    
    read_mac(NET_OPTIONS.mac);
}

int basestation_start (void)
{
  int8_t status, ant_status, gprs_status, process_status, websocket_status;
  uint16_t sp;
 /**
  * Set stack pointer to a temporary stack position required
  * during this startup function. Note that all the stacks must reside
  * in the top of the memory to make malloc() work. malloc() will not
  * allocate any memory above the current stack pointer.
  */
  sp=RAMEND-IDLE_THREAD_STACK_SIZE_BYTES
    -ANT_THREAD_STACK_SIZE_BYTES
    -PROCESS_THREAD_STACK_SIZE_BYTES
    -WEBSOCKET_THREAD_STACK_SIZE_BYTES
    -GPRS_THREAD_STACK_SIZE_BYTES;
  
  //we assign the stack pointer in two steps because of
  //http://savannah.nongnu.org/bugs/?25778
  SPH=(uint8_t)(sp>>8);
  SPL=(uint8_t)sp;
  
  //initialize all modules
  init();
  
  
   /**
    * Note: to protect OS structures and data during initialization,
    * interrupts must remain disabled until the first thread
    * has been restored. They are re-enabled at the very end of
    * the first thread restore, at which point it is safe for a
    * reschedule to take place.
    */

   /**
    * Initialize the OS before creating our threads.
    *
    */
  status = atomOSInit(idle_thread_stack, 
            IDLE_THREAD_STACK_SIZE_BYTES, idle_thread_callback);
    
  if (status == ATOM_OK)
  {
      /* Enable the system tick timer */
      avrInitSystemTickTimer();

      /* Create ANT thread */
      ant_status = atomThreadCreate(&ant_tcb,
                    ANT_THREAD_PRIO, ant_thread_func, 0,
                    ant_thread_stack,
                    ANT_THREAD_STACK_SIZE_BYTES);
      
      //ant_status = 0;
       
      /* Create process rx thread */
      process_status = atomThreadCreate(&process_tcb,
                    PROCESS_THREAD_PRIO, process_thread_func, 0,
                    process_thread_stack,
                    PROCESS_THREAD_STACK_SIZE_BYTES);
      
      //process_status = 0;

      /* Create GPRS thread */
      gprs_status = atomThreadCreate(&gprs_tcb,
                    GPRS_THREAD_PRIO, gprs_thread_func, 0,
                    gprs_thread_stack,
                    GPRS_THREAD_STACK_SIZE_BYTES);
      
      //gprs_status = 0;
      
      /* Create websocket thread */
      websocket_status = atomThreadCreate(&websocket_tcb,
                    WEBSOCKET_THREAD_PRIO, websocket_thread_func, 0,
                    websocket_thread_stack,
                    WEBSOCKET_THREAD_STACK_SIZE_BYTES);
      
      //websocket_status = 0;
      
      
      if (ant_status == ATOM_OK && gprs_status == ATOM_OK && process_status == ATOM_OK && websocket_status == ATOM_OK)
      {
         /**
          * Application threads successfully created. It is
          * now possible to start the OS. Execution will not return
          * from atomOSStart(), which will restore the context of
          * our application thread and start executing it.
          *
          * Note that interrupts are still disabled at this point.
          * They will be enabled as we restore and execute our first
          * thread in archFirstThreadRestore().
          */
          atomOSStart();
      }
  }
    
  while (1)
      ;

  /* There was an error starting the OS if we reach here */
  return (0);
    
}

/**
 * \b ant_idle_thread_callback
 *
 * Callback for the idle thread.
 *
 * @return None
 */
static void idle_thread_callback (void)
{
  for(;;) {
    sleepmgr_enter_sleep();
  }
}


static void websocket_thread_func (uint32_t data)
{
  net_init();
  
  for(;;) {
    ui_led_blink(UI_GREEN_LED, 5, 5, 30);
    
    while(!net_link_ok())
      atomTimerDelay(SYSTEM_TICKS_PER_SEC);
    
    net_config(&NET_OPTIONS);
    
    ui_led_off(UI_RED_LED);
    ui_led_on(UI_GREEN_LED);
    
    if(net_websocket_handler()!=STATUS_OK) {
      ui_led_on(UI_RED_LED);
    }
    
    net_reset();
    
    atomTimerDelay(SYSTEM_TICKS_PER_SEC);
    
  }
}

static int8_t net_websocket_handler () {
    int8_t status;
    uint8_t retry_count;
    
    for(retry_count=0;retry_count<3;retry_count++) {
      atomTimerDelay((0x01 << retry_count)*SYSTEM_TICKS_PER_SEC);
    
      if(net_start_dhcp(&NET_OPTIONS)==STATUS_OK) {
        break;
      }
    }
    
    for(retry_count=0;retry_count<3;retry_count++) {
      atomTimerDelay((0x01 << retry_count)*SYSTEM_TICKS_PER_SEC);
    
      if(dns_resolve(NET_DNS_SOCKET, NET_OPTIONS.dns[retry_count % 2], WEBSOCKET_OPTIONS.host, NET_CLIENT_OPTIONS.ip)==STATUS_OK) {
        break;
      }
    }
    
    //init websocket
    memset(&websocket, 0, sizeof(struct websocket_t));
    websocket.id=NET_CLIENT_SOCKET;
    websocket.read_socket=net_receive;
    websocket.write_socket=net_send;
    websocket.on_message=on_message;
    websocket.data_callback=message_data_callback;
    websocket.options=&WEBSOCKET_OPTIONS;
    
    atomMutexGet(&websocket_mutex, 10*SYSTEM_TICKS_PER_SEC);
    
    for(retry_count=0;retry_count<2;retry_count++) {
      //open tcp socket
      status=net_connect(NET_CLIENT_SOCKET, &NET_CLIENT_OPTIONS);
      if(status!=STATUS_OK) {
        switch(retry_count) {
          case 0:
            net_close(websocket.id);
            atomTimerDelay(SYSTEM_TICKS_PER_SEC);
            continue;
          case 1:
            net_reset();
            atomTimerDelay(SYSTEM_TICKS_PER_SEC);
            while(!net_link_ok())
              atomTimerDelay(SYSTEM_TICKS_PER_SEC);
            net_config(&NET_OPTIONS);
            continue;
        }
      }
      
      atomSemPut(&websocket_sem);
      status=websocket_start(&websocket);
      atomSemGet(&websocket_sem, -1);
      net_close(websocket.id);
      net_clear_buffer(websocket.id);
    }
    
    atomMutexPut(&websocket_mutex);
    
    return status;
}  
  
  
/**
 * \b ant_thread_func
 *
 * Entry point for ANT application thread.
 *
 * @param[in] data Unused (optional thread entry parameter)
 *
 * @return None
 */
static void ant_thread_func (uint32_t data)
{ 
    // initialize rf circuits...
    ant_init(ANT_DEVICE_TYPE_BASE, (uint32_t)signature.device_id, (uint32_t)state.network_id);
  
    ui_led_on(UI_GREEN_LED);
  
    for(;;) {
        ant_start(ant_callback);
    }    
}    



static void ant_callback(ant_event_t evt, uint8_t *data, uint16_t len) {
  
  switch(evt) {
    case ANT_EVENT_SCAN_START:
    case ANT_EVENT_CONNECTED:
    case ANT_EVENT_POLL_BEGIN:
    case ANT_EVENT_POLL_END:
      break;
    case ANT_EVENT_MASTER_DATA:
    case ANT_EVENT_NETWORK_DATA:
    case ANT_EVENT_SLAVE_DATA:
    default:
      process_callback_data(data, len);
  }
}  


static void process_callback_data(uint8_t *data, uint16_t len) {
  iop_Envelope env;
  pb_istream_t msg_stream;
  pb_istream_t istream;
  pb_ostream_t ostream;
    
  memset(&env, 0, sizeof(iop_Envelope));
  memset(&istream, 0, sizeof(pb_istream_t));
  memset(&ostream, 0, sizeof(pb_ostream_t));
  
  istream=pb_istream_from_buffer(data, len);
    
  //try to write it directly to the websocket
  if(websocket.state==WEBSOCKET_STATE_OPEN) {
      if(websocket_send_binary(&websocket, (io_input_stream_t *)&istream, len)==STATUS_OK) {
          ui_led_blink(UI_GREEN_LED, 2, 4, 8);
          return;
      }
  }
    
  //else write the message to flash
  msg_stream=pb_istream_from_buffer(NULL, 0);
  if(!decode_envelope(&env, &istream, &msg_stream)) {
    return;
  }
  
  ostream.max_size=SIZE_MAX;
  
  //set timestamp if missing
  if(!env.has_timestamp) {
      env.has_timestamp=true;
      env.timestamp=(uint64_t)rtc_get_time();
      env.timestamp*=1000;
      //find length of modified message
      ostream.callback=NULL;
      encode_envelope(&ostream, &env, &msg_stream);
      len=ostream.bytes_written;
  }
    
  //write length
  flash_fifo_write(&FIFO, (uint8_t *)&len, 2);
  
  //serialize the envelope
  ostream.state=&FIFO;
  ostream.callback=write_flash_fifo_cb;
  encode_envelope(&ostream, &env, &msg_stream);
  
  atomSemPut(&tx_sem);
  
  ui_led_blink(UI_GREEN_LED, 2, 4, 8);
}  



static void process_thread_func (uint32_t data)
{
  int8_t status;
  uint16_t len;
  uint32_t count;
  io_input_stream_t stream;
  
  stream.read=read_flash_fifo_cb;
  next_snapshot=rtc_get_time();
  
  for(;;) {
    atomSemGet(&websocket_sem, 0);
    atomSemPut(&websocket_sem);
    
    count=flash_fifo_count(&FIFO);
    
    while(count>0) {
      flash_fifo_set_mark(&FIFO);
      status=flash_fifo_read(&FIFO, (uint8_t *)&len, 2);
      stream.state=&FIFO;
      stream.bytes_left=count;
      status=websocket_send_binary(&websocket, &stream, len);
      switch(status) {
        case STATUS_OK:
          count=flash_fifo_count(&FIFO);
          break;
        case ERR_NO_MEMORY:
          flash_fifo_clear(&FIFO);
          count=0;
          break;
        case ERR_PROTOCOL:
        case ERR_IO_ERROR:
          atomTimerDelay(5*SYSTEM_TICKS_PER_SEC);
        default:
          flash_fifo_rollback_to_mark(&FIFO);
          count=0;
      }
      
      atomTimerDelay(10);
    }
    
    atomSemGet(&tx_sem, 5*SYSTEM_TICKS_PER_SEC);
    
    if(fw_upload_len) {
      status=process_fw_data(fw_upload_len);
      atomTimerDelay(SYSTEM_TICKS_PER_SEC);
      confirm_fw_upload(status);
      fw_upload_len=0;
    }
    else if(rtc_get_time() > next_snapshot) {
      snapshot_state();
      next_snapshot=rtc_get_time()+state.snapshot_interval;
    }
  }
}

/**
 * \b gprs_thread_func
 *
 * Entry point for GPRS application thread.
 *
 * @param[in] data Unused (optional thread entry parameter)
 *
 * @return None
 */
static void gprs_thread_func (uint32_t data)
{
  uint8_t err_count=0;
  
  //init gprs
  if(gprs_init(&GPRS_OPTIONS)!=STATUS_OK) {
      atomTimerDelay((0x01 << err_count)*SYSTEM_TICKS_PER_SEC);
      if(err_count<12) err_count++;
  }
  
  atomTimerDelay(SYSTEM_TICKS_PER_SEC);
  
  for(;;) {
    err_count=0;
    
    while(err_count<5) {
        
        if(gprs_websocket_handler()!=STATUS_OK) {
      
          //make sure to release the crypto mutex
          crypto_final();
      
          //try to close the socket
          gprs_close_socket();
          
          atomTimerDelay((0x01 << err_count)*SYSTEM_TICKS_PER_SEC);
      
          err_count++;
        }
        else {
          err_count=0;
        }
    }
    
    gprs_reset(&GPRS_OPTIONS);
    
    //delay
    atomTimerDelay(state.transfer_interval*SYSTEM_TICKS_PER_SEC);
  }    
}


/**
 * \b gprs websocket
 *
 * Send and receive data via gprs websocket.
 *
 * @return None
 */
static int8_t gprs_websocket_handler (void)
{
    int8_t status;
    
    atomMutexGet(&websocket_mutex, 0);
    
    //open tcp socket
    if(gprs_open_socket(&GPRS_OPTIONS)!=STATUS_OK) {
        atomMutexPut(&websocket_mutex);
        return ERR_TIMEOUT;
    }  
    
    memset(&websocket, 0, sizeof(struct websocket_t));
    websocket.id=0;
    websocket.read_socket=&gprs_read_socket;
    websocket.write_socket=&gprs_write_socket;
    websocket.on_message=&on_message;
    websocket.data_callback=message_data_callback;
    websocket.options=&WEBSOCKET_OPTIONS;
  
    atomSemPut(&websocket_sem);
    status=websocket_start(&websocket);
    atomSemGet(&websocket_sem, -1);
  
    atomMutexPut(&websocket_mutex);
    
    if(status!=STATUS_OK) {
      return status;
    }
    
    gprs_close_socket();
  
    return STATUS_OK;
}

static void message_data_callback(uint16_t counter, uint8_t *buf, uint16_t len) {
  
  if(counter==0) {
    flash_fifo_clear(&WEBSOCKET_FIFO);
  }
  flash_fifo_write(&WEBSOCKET_FIFO, buf, len);
}

static void on_message(uint8_t *buf, uint32_t len) {
    
    ui_led_blink(UI_GREEN_LED, 2, 4, 8);
    
    if(len>256) {
      if(!fw_upload_len) {
        fw_upload_len=len;
        atomSemPut(&tx_sem);
      }
    }
    else {
      process_rx_data(buf, (uint16_t)len);
    }
}


static int8_t process_fw_data(uint32_t len) {
  int8_t status;
  uint8_t hmac[SHA1_BYTES];
  uint8_t *buf, *p;
  uint8_t bc, bl;
  uint16_t l;
  iop_Envelope env;
  firmware_Firmware msg;
  pb_istream_t istream;
  pb_istream_t msg_stream;
  io_input_stream_t src;
  
  memset(&env, 0, sizeof(iop_Envelope));
  
  buf=mem_safe_malloc(256);
  if(buf==NULL) {
    return ERR_NO_MEMORY;
  }
  
  flash_fifo_read(&WEBSOCKET_FIFO, buf, 256);
  
  //decode envelope
  istream=pb_istream_from_buffer(buf, len);
  msg_stream=pb_istream_from_buffer(NULL, 0);
  if (!decode_envelope(&env, &istream, &msg_stream)) {
    mem_safe_free(buf);
    return ERR_BAD_FORMAT;
  }
  
  WEBSOCKET_FIFO.tail=(WEBSOCKET_FIFO.tail-256+(len-msg_stream.bytes_left)) % WEBSOCKET_FIFO.buffer_len;
  flash_fifo_set_mark(&WEBSOCKET_FIFO);
  src.bytes_left=flash_fifo_count(&WEBSOCKET_FIFO);
  src.state=&WEBSOCKET_FIFO;
  src.read=read_flash_fifo_cb;
  
  p=(uint8_t *)mem_safe_calloc(SHA1_BLOCK_BYTES, sizeof(uint8_t));
  if(p==NULL) {
    mem_safe_free(buf);
    return ERR_NO_MEMORY;
  }
  calc_hash(hmac, p, signature.key, &env, &src);
  mem_safe_free(p);
  flash_fifo_rollback_to_mark(&WEBSOCKET_FIFO);
  src.bytes_left=flash_fifo_count(&WEBSOCKET_FIFO);
  
  //if hash do not match, discard message
  if(memcmp(hmac, env.hash.bytes, SHA1_BYTES)!=0) {
    mem_safe_free(buf);
    return ERR_BAD_DATA;
  }
  
  //check that the received nonce is greater than the last received
  if(state.rx_nonce>=env.nonce) {
    //mem_safe_free(buf);
    //return ERR_BAD_DATA;
  }
  
  state.rx_nonce=env.nonce;
  save_state();
  
  crypto_start_decrypt();
  
  while(src.bytes_left>0) {
    l=src.bytes_left>256 ? 256 : (uint16_t)src.bytes_left;
    src.read(&src, buf, l);
    p=buf;
    bc=(uint8_t)(l/AES_DATA_SIZE);
    l=0;
    for(;bc!=0;bc--) {
      //decrypt block
      if(crypto_decrypt_block(p, env.iv.bytes, &bl, (bool)(src.bytes_left==0 && bc==1))!=STATUS_OK) {
        mem_safe_free(buf);
        return ERR_TIMEOUT;
      }
      l+=bl;
      p+=AES_DATA_SIZE;
    }
    flash_fifo_write(&WEBSOCKET_FIFO, buf, l);
  }
  
  crypto_final();
  
  len=flash_fifo_count(&WEBSOCKET_FIFO);
  flash_fifo_read(&WEBSOCKET_FIFO, buf, 128);
  
  //decode message
  istream=pb_istream_from_buffer(buf, len);
  msg_stream=pb_istream_from_buffer(NULL, 0);
  msg.hex.funcs.decode=decode_hex_callback;
  msg.hex.arg=&msg_stream;
  if (!pb_decode(&istream, firmware_Firmware_fields, &msg)) {
    mem_safe_free(buf);
    return ERR_BAD_FORMAT;
  }
  
  mem_safe_free(buf);
  
  WEBSOCKET_FIFO.tail=(WEBSOCKET_FIFO.tail-128+(len-msg_stream.bytes_left)) % WEBSOCKET_FIFO.buffer_len;
  
  src.state=&WEBSOCKET_FIFO;
  src.bytes_left=flash_fifo_count(&WEBSOCKET_FIFO);
  status=ant_tunnel_open_channel(ANT_TUNNEL_CHANNEL_NO, (uint32_t)msg.crc, TRUE);
  if(status==STATUS_OK) {
    status=ant_tunnel_transfer(ANT_TUNNEL_CHANNEL_NO, src);
  }
  ant_tunnel_close_channel(ANT_TUNNEL_CHANNEL_NO);
  
  return status;
}

static bool decode_hex_callback(pb_istream_t *stream, const pb_field_t *field, void *arg) {
  pb_istream_t *msg_stream=(pb_istream_t *)arg;
  msg_stream->bytes_left=stream->bytes_left;
  msg_stream->state=stream->state;
  
  stream->bytes_left=0;
  
  return true;
}

static int8_t process_rx_data(uint8_t *buf, uint16_t len) {
  iop_Envelope env;
  basestation_In msg;
  uint8_t hmac[SHA1_BYTES];
  uint8_t *key_buffer;
  pb_istream_t istream;
  pb_istream_t msg_stream;
  io_input_stream_t src;
  
  memset(&env, 0, sizeof(iop_Envelope));
  
  istream=pb_istream_from_buffer(buf, (size_t)len);
  msg_stream=pb_istream_from_buffer(NULL, 0);
  
  //decode envelope
  if (!decode_envelope(&env, &istream, &msg_stream)) {
    return ERR_BAD_FORMAT;
  }
  
  //check if the message is for this node
  if(env.dst!=signature.device_id) {
    //deliver the message to the recipient
    return ant_send_to_device(env.dst, buf, (uint16_t)len);
  }
  else if(env.msgCode!=BASESTATION_IN_MSG_CODE) {
    return ERR_UNSUPPORTED_DEV;
  }
  
  memset(&msg, 0, sizeof(basestation_In));
  
  src=io_input_stream_from_buffer((uint8_t *)msg_stream.state, msg_stream.bytes_left);
  key_buffer=(uint8_t *)mem_safe_calloc(SHA1_BLOCK_BYTES, sizeof(uint8_t));
  if(key_buffer==NULL) {
    return ERR_NO_MEMORY;
  }
  calc_hash(hmac, key_buffer, signature.key, &env, &src);
  mem_safe_free(key_buffer);
  
  //if hash do not match, discard message
  if(memcmp(hmac, env.hash.bytes, SHA1_BYTES)!=0) {
    return ERR_BAD_DATA;
  }
  
  //check that the received nonce is greater than the last received
  if(state.rx_nonce>=env.nonce) {
    //return ERR_BAD_DATA;
  }
  
  state.rx_nonce=env.nonce;
  save_state();
  len=(uint16_t)msg_stream.bytes_left;
  
  crypto_start_decrypt();
  crypto_decrypt(msg_stream.state, env.iv.bytes, &len);
  crypto_final();
  
  msg_stream.bytes_left=len;
  
  //decode message
  if (!pb_decode(&msg_stream, basestation_In_fields, &msg)) {
     return ERR_BAD_FORMAT;
  }
  
  if(msg.has_networkId) {
    state.network_id=msg.networkId;
    ant_set_network_id((uint32_t)msg.networkId);
  }
      
  if(msg.has_time) {
    rtc_set_time((uint32_t)(msg.time/1000));
  }
  
  if(msg.has_transferInterval) {
    state.snapshot_interval=(uint16_t)msg.transferInterval;
    next_snapshot=rtc_get_time();
  }
    
  atomSemPut(&tx_sem);
    
  return STATUS_OK;
}

  //read the serialized data from the flash memory
static int8_t read_flash_fifo_cb(io_input_stream_t *stream, uint8_t *buf, uint16_t count) {
  flash_fifo_read((FLASH_FIFO *)stream->state, buf, count);
  stream->bytes_left-=count;
  return STATUS_OK;
}

  //store the serialized data in the flash memory
static bool write_flash_fifo_cb(pb_ostream_t *stream, const uint8_t *buf, size_t count) {
  flash_fifo_write((FLASH_FIFO *)stream->state, buf, count);
  stream->bytes_written+=count;
  return true;
}

static int8_t snapshot_state() {
    pb_bytes_array_t *env_bytes;
    uint8_t *msg_buf;
    basestation_Out msg;
    pb_ostream_t stream;
    
    if(!state.network_id)
    {
      return ERR_BAD_ADDRESS;
    }
  
    memset((void *)&msg, 0, sizeof(basestation_Out));
  
    //allocate memory for the envelope
    env_bytes=(pb_bytes_array_t *)mem_safe_malloc(ENV_BUF_SIZE);
    if(env_bytes==NULL) {
      return ERR_NO_MEMORY;
    }
    
    msg_buf=(uint8_t *)env_bytes+(ENV_BUF_SIZE-MSG_BUF_SIZE);
    
    if(power_read_temperature(&msg.temperature)==STATUS_OK) {
      msg.temperature+=signature.temp_offset;
      msg.has_temperature=true;
    }
  
    if(power_read_batt_lvl(&msg.batteryLevel)==STATUS_OK) {
      msg.has_batteryLevel=true;
    }
  
    // Get the topology of the network
    msg.topology.funcs.encode=encode_topology_callback;
    msg.node.funcs.encode=encode_node_callback;
  
    //serialize the message
    stream=pb_ostream_from_buffer(msg_buf, MSG_BUF_SIZE-sizeof(size_t));
    if (!pb_encode(&stream, basestation_Out_fields, &msg)) {
        mem_safe_free(env_bytes);
        return ERR_BAD_FORMAT;
    }
    
    send(env_bytes, BASESTATION_OUT_MSG_CODE, msg_buf, stream.bytes_written);
    
    mem_safe_free(env_bytes);
  
    return STATUS_OK;
}

static bool encode_topology_callback(pb_ostream_t *stream, const pb_field_t *field, const void *arg) {
  uint8_t i;
  basestation_Out_Topology topology;
  
  for(i=0;i<OLSR_MAX_TOPOLOGY;i++) {
    if(olsr_topology_set[i].dest) {
      topology.destinationId=olsr_topology_set[i].dest;
      topology.previousId=olsr_topology_set[i].last;
      
      if (!pb_encode_tag_for_field(stream, field))
        return false;
    
      if (!pb_encode_submessage(stream, basestation_Out_Topology_fields, &topology))
        return false;
    }
  }
  
  return true;
}

static bool encode_node_callback(pb_ostream_t *stream, const pb_field_t *field, const void *arg) {
  uint8_t i;
  basestation_Out_Node node;
  
  for(i=0;i<OLSR_MAX_MID;i++) {
    if(olsr_mid_set[i].origin) {
      node.nodeId=olsr_mid_set[i].origin;
      node.deviceId=olsr_mid_set[i].device_id;
      node.has_deviceType=true;
      node.deviceType=olsr_mid_set[i].device_type;
      
      if (!pb_encode_tag_for_field(stream, field))
        return false;
  
      if (!pb_enc_submessage(stream, field, &node))
        return false;
    }
  }
  
  return true;
}


static int8_t confirm_fw_upload(int8_t status) {
  pb_bytes_array_t *env_bytes;
  uint8_t *msg_buf;
  firmware_Response msg;
  pb_ostream_t stream;
  
  if(!state.network_id)
  {
    return ERR_BAD_ADDRESS;
  }
  
  //allocate memory for the envelope
  env_bytes=(pb_bytes_array_t *)mem_safe_malloc(ENV_BUF_SIZE);
  if(env_bytes==NULL) {
    return ERR_NO_MEMORY;
  }
  
  msg_buf=(uint8_t *)env_bytes+(ENV_BUF_SIZE-MSG_BUF_SIZE);
  
  memset((void *)&msg, 0, sizeof(firmware_Response));
  
  switch(status) {
    case STATUS_OK:
      msg.status=firmware_Response_Status_SUCCESS;
      break;
    case ANT_EVENT_TRANSFER_TX_FAILED:
      msg.status=firmware_Response_Status_TRANSFER_FAILED;
      break;
    default:
      msg.status=firmware_Response_Status_NOT_IN_RANGE;
  }
  msg.has_errorCode=1;
  msg.errorCode=status;
  
  //serialize the message
  stream=pb_ostream_from_buffer(msg_buf, MSG_BUF_SIZE-sizeof(size_t));
  if (!pb_encode(&stream, firmware_Response_fields, &msg)) {
    mem_safe_free(env_bytes);
    return ERR_IO_ERROR;
  }
  
  send(env_bytes, FIRMWARE_RESPONSE_MSG_CODE, msg_buf, stream.bytes_written);
  
  mem_safe_free(env_bytes);
  
  return STATUS_OK;
}


static int8_t send(pb_bytes_array_t *env_bytes, uint64_t msg_code, uint8_t *msg_buf, uint16_t len) {
  int8_t status=STATUS_OK;
  uint8_t *tmp;
  iop_Envelope env;
  pb_ostream_t output_stream;
  pb_istream_t msg_input_stream;
  io_input_stream_t istream;
  
  memset((void *)&env, 0, sizeof(iop_Envelope));
  
  env.msgCode=msg_code;
  env.has_dst=false;
  env.has_src=true;
  env.src=signature.device_id;
  env.has_iv=true;
  env.iv.size=AES_KEY_SIZE;
  env.has_hash=true;
  env.hash.size=SHA1_BYTES;
  env.has_nonce=true;
  env.nonce = ++state.tx_nonce;
  if((state.tx_nonce % 100)==0) {
    save_state();
  }
  
  crypto_init_iv(env.iv.bytes);
  crypto_start_encrypt(env.iv.bytes);
  crypto_encrypt(msg_buf, &len);
  crypto_final();
  
  //calculate hash
  tmp=(uint8_t *)env_bytes;
  istream=io_input_stream_from_buffer(msg_buf, len);
  calc_hash(env.hash.bytes, tmp, signature.key, &env, &istream);
  
  output_stream=pb_ostream_from_buffer(env_bytes->bytes, ENV_BUF_SIZE-sizeof(size_t));
  msg_input_stream=pb_istream_from_buffer(msg_buf, len);
  encode_envelope(&output_stream, &env, &msg_input_stream);
  env_bytes->size=output_stream.bytes_written;
  
  istream=io_input_stream_from_buffer(env_bytes->bytes, env_bytes->size);
  websocket_send_binary(&websocket, &istream, env_bytes->size);
  
  return status;
}


static void ui_listener(uint8_t event) {
  switch(event) {
    case UI_EVENT_BUTTON_DOWN:
      ui_led_on(UI_RED_LED);
      keydown=rtc_get_time();
      break;
    case UI_EVENT_BUTTON_UP:
      if((rtc_get_time()-keydown)>3) {
        ui_led_blink(UI_RED_LED, 1, 1, 10);
        ui_led_blink(UI_GREEN_LED, 1, 1, 10);
        _delay_ms(1000);
        return;
      }
      else {
        _delay_ms(100);
        reset_do_soft_reset();
      }
      
      ui_led_off(UI_RED_LED);
      break;
  }
}  

static void read_mac(uint8_t *mac) {
  twi_options_t opt;
  twi_package_t rx_packet;
  
  opt.speed=100000;
  twi_master_setup(W5200_TWI, &opt);
  
  rx_packet.chip=0x50;
  rx_packet.addr_length=1;
  rx_packet.addr[0]=0xFA;
  rx_packet.buffer=mac;
  rx_packet.length=6;
  rx_packet.no_wait=false;
  twi_master_read(W5200_TWI, &rx_packet);
  
  //disable twi
  //sysclk_disable_peripheral_clock(W5200_TWI);
}


static void read_state() {
  nvm_eeprom_read_buffer(STATE_EEPROM_PAGE_ADDRESS*EEPROM_PAGE_SIZE, &state, sizeof(struct basestation_state_t));
  if(state.tx_nonce==0xFFFFFFFFFFFFFFFF) {
    state.network_id=0x0,
    state.transfer_interval=10,
    state.snapshot_interval=60,
    state.send_immediate=FALSE,
    state.rx_nonce=0;
    state.tx_nonce=0;
    save_state();
  }
  else {
    state.rx_nonce+=100;
    state.tx_nonce+=100;
  }
}

static void save_state() {
  uint8_t i;
  uint8_t *p;
  
  p=(uint8_t *)&state;
  nvm_eeprom_flush_buffer();
  for(i=0;i<sizeof(struct basestation_state_t);i++) {
    nvm_eeprom_load_byte_to_buffer(i, *p++);
  }
  nvm_eeprom_atomic_write_page(STATE_EEPROM_PAGE_ADDRESS);
}

