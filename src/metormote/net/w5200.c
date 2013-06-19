/*
 * w5200.c
 *
 * Created: 12/27/2011 6:50:36 PM
 *  Author: Administrator
 */ 
#include "w5200.h"


static uint16_t W5200_SMASK[W5200_MAX_SOCK_NUM]; /**< Variable for Tx buffer MASK in each channel */
static uint16_t W5200_RMASK[W5200_MAX_SOCK_NUM]; /**< Variable for Rx buffer MASK in each channel */
static uint16_t W5200_SSIZE[W5200_MAX_SOCK_NUM]; /**< Max Tx buffer size by each channel */
static uint16_t W5200_RSIZE[W5200_MAX_SOCK_NUM]; /**< Max Rx buffer size by each channel */
static uint16_t W5200_SBUFBASEADDRESS[W5200_MAX_SOCK_NUM]; /**< Tx buffer base address by each channel */
static uint16_t W5200_RBUFBASEADDRESS[W5200_MAX_SOCK_NUM]; /**< Rx buffer base address by each channel */


struct spi_device W5200_DEVICE = {
  .id = W5200_nSCS
};

static ATOM_MUTEX w5200_mutex;
static ATOM_SEM w5200_ir_sem;
static ATOM_SEM w5200_spi_sem;

w5200_socket_t w5200_sockets[W5200_MAX_SOCK_NUM];

static volatile struct w5200_frame current_frame;
static volatile uint8_t *frame_ptr;
static volatile uint8_t frame_pos;
static volatile uint16_t data_bytes_left;

static uint16_t local_port;

static uint8_t w5200_spi_interrupt_handler(void);
static uint8_t w5200_read(uint16_t addr);
static void w5200_write(uint16_t addr, uint8_t data);
static uint16_t w5200_write_buf(uint16_t addr, io_input_stream_t *src, uint16_t len);
static uint16_t w5200_read_buf(uint16_t addr, io_output_stream_t *dst, uint16_t len);
static void w5200_write_data(SOCKET s, io_input_stream_t *src, uint8_t *dst, uint16_t len);
static void w5200_read_data(SOCKET s, uint8_t *src, io_output_stream_t *dst, uint16_t len);
static void w5200_send_data_processing(SOCKET s, io_input_stream_t *src, uint16_t len);


ISR(W5200_nINT_VECTOR) {
  atomIntEnter();
  atomSemPut(&w5200_ir_sem);
  atomIntExit(FALSE, TRUE);
}

ISR(W5200_nLINKLED_VECTOR) {

}

ISR(W5200_SPI_INT_VECTOR) {
  atomIntEnter();
  atomIntExit(FALSE, w5200_spi_interrupt_handler());
}



static uint8_t w5200_spi_interrupt_handler() {
  uint8_t data;
  
  if(frame_pos==0) {
    frame_ptr=(uint8_t *)&current_frame;
  }
  
  if(frame_pos<4) {
    spi_write_single(W5200_SPI, *frame_ptr++);
    frame_pos++;
  }
  else if(frame_pos==4 && current_frame.command==W5200_COMMAND_READ) {
    spi_write_single(W5200_SPI, CONFIG_SPI_MASTER_DUMMY);
    frame_pos++;
  }
  else if(data_bytes_left>0) {
    if(current_frame.command==W5200_COMMAND_WRITE) {
      if(current_frame.stream.src!=NULL) {
        current_frame.stream.src->read(current_frame.stream.src, &data, 1);
      }
      else {
        data=current_frame.data;
      }
      spi_write_single(W5200_SPI, data);
      data_bytes_left--;
    }
    else {
      spi_read_single(W5200_SPI, &data);
      if(current_frame.stream.dst!=NULL) {
        current_frame.stream.dst->write(current_frame.stream.dst, &data, 1);
      }
      else {
        current_frame.data=data;
      }
      
      if(--data_bytes_left>0) {
        spi_write_single(W5200_SPI, CONFIG_SPI_MASTER_DUMMY);
      }
      else {
        atomSemPut(&w5200_spi_sem);
        return TRUE;
      }
    }
  }
  else {
    atomSemPut(&w5200_spi_sem);
    return TRUE;
  }
  return FALSE;
 }


/**
@brief  This function reads the value from W5200 registers.
*/
static uint8_t w5200_read(uint16_t addr)
{
  uint8_t data;
  
  if(atomMutexGet(&w5200_mutex, W5200_TIMEOUT)!=ATOM_OK) {
    return 0;
  }
    
  current_frame.address_high=(addr & 0xFF00) >> 8;
  current_frame.address_low=addr & 0x00FF;
  current_frame.command=W5200_COMMAND_READ;
  current_frame.len_high=0;
  current_frame.len_low=1;
  current_frame.data=0;
  current_frame.stream.dst=NULL;
  
  frame_pos=0;
  data_bytes_left=1;
  spi_select_device(W5200_SPI, &W5200_DEVICE);
  w5200_spi_interrupt_handler();
  if(atomSemGet(&w5200_spi_sem, W5200_TIMEOUT)!=ATOM_OK) {
    frame_pos=0;
  }
  spi_deselect_device(W5200_SPI, &W5200_DEVICE);
  
  data=current_frame.data;
  
  atomMutexPut(&w5200_mutex);
  
  return data;
}

/**
@brief  This function writes the data into W5200 registers.
*/
static void w5200_write(uint16_t addr, uint8_t data)
{
  if(atomMutexGet(&w5200_mutex, W5200_TIMEOUT)!=STATUS_OK) {
    return;
  }
  
  current_frame.address_high=(addr & 0xFF00) >> 8;
  current_frame.address_low=addr & 0x00FF;
  current_frame.command=W5200_COMMAND_WRITE;
  current_frame.len_high=0;
  current_frame.len_low=1;
  current_frame.data=data;
  current_frame.stream.src=NULL;
  
  frame_pos=0;
  data_bytes_left=1;
  spi_select_device(W5200_SPI, &W5200_DEVICE);
  w5200_spi_interrupt_handler();
  if(atomSemGet(&w5200_spi_sem, W5200_TIMEOUT)!=ATOM_OK) {
    frame_pos=0;
  }
  spi_deselect_device(W5200_SPI, &W5200_DEVICE);
   
  atomMutexPut(&w5200_mutex);
}


/**
@brief  This function reads from W5200 rx memory
*/ 
static uint16_t w5200_read_buf(uint16_t addr, io_output_stream_t *dst, uint16_t len)
{
  if(atomMutexGet(&w5200_mutex, W5200_TIMEOUT)!=STATUS_OK) {
    return 0;
  }
  
  current_frame.address_high=(addr & 0xFF00) >> 8;
  current_frame.address_low=addr & 0x00FF;
  current_frame.command=W5200_COMMAND_READ;
  current_frame.len_high=(len & 0x7F00) >> 8;
  current_frame.len_low=len & 0x00FF;
  current_frame.stream.dst=dst;
   
  frame_pos=0;
  data_bytes_left=len;
  spi_select_device(W5200_SPI, &W5200_DEVICE);
  w5200_spi_interrupt_handler();
  if(atomSemGet(&w5200_spi_sem, W5200_TIMEOUT)!=ATOM_OK) {
    frame_pos=0;
  }
  spi_deselect_device(W5200_SPI, &W5200_DEVICE);
  
  len-=data_bytes_left;
  current_frame.stream.dst=NULL;
  atomMutexPut(&w5200_mutex);
  
  return len;
}

/**
@brief  This function writes into W5200 tx memory
*/ 
static uint16_t w5200_write_buf(uint16_t addr, io_input_stream_t *src, uint16_t len)
{
  if(atomMutexGet(&w5200_mutex, W5200_TIMEOUT)!=STATUS_OK) {
    return 0;
  }
  
  current_frame.address_high=(addr & 0xFF00) >> 8;
  current_frame.address_low=addr & 0x00FF;
  current_frame.command=W5200_COMMAND_WRITE;
  current_frame.len_high=(len & 0x7F00) >> 8;
  current_frame.len_low=len & 0x00FF;
  current_frame.stream.src=src;
  
  frame_pos=0;
  data_bytes_left=len;
  spi_select_device(W5200_SPI, &W5200_DEVICE);
  w5200_spi_interrupt_handler();
  if(atomSemGet(&w5200_spi_sem, W5200_TIMEOUT)!=ATOM_OK) {
    frame_pos=0;
  }
  spi_deselect_device(W5200_SPI, &W5200_DEVICE);
  
  len-=data_bytes_left;
  
  current_frame.stream.src=NULL;
  atomMutexPut(&w5200_mutex);
  
  return len;
}



/**
@brief  This function is for init the w5200 spi. 
*/ 
void w5200_init(void)
{
  atomMutexCreate(&w5200_mutex);
  atomSemCreate(&w5200_ir_sem, 0);
  atomSemCreate(&w5200_spi_sem, 0);
  
  spi_master_init(W5200_SPI);
  spi_master_setup_device(W5200_SPI, &W5200_DEVICE, SPI_MODE_0, W5200_SPI_MASTER_SPEED, 0);
  spi_enable(W5200_SPI);
  
  (W5200_SPI)->INTCTRL=SPI_INTLVL_LO_gc;
  
  //set W5200_INT_PORT pin 5 as source of interrupts 
  W5200_INT_PORT->INT0MASK = W5200_INT_PORT->INT0MASK | ioport_pin_to_mask(W5200_nINT);
  //set interrupt level to low
  W5200_INT_PORT->INTCTRL = (W5200_INT_PORT->INTCTRL & ~ PORT_INT0LVL_gm) | PORT_INT0LVL_LO_gc;
  //sense falling edge
  W5200_INT_PORT->PIN5CTRL = IOPORT_SENSE_FALLING;
  
  
  //set W5200_nLINKLED pin 2 as source of interrupts 
  //W5200_LINKLED_PORT->INT1MASK = W5200_LINKLED_PORT->INT1MASK | ioport_pin_to_mask(W5200_nLINKLED);
  //set interrupt level to low
  //W5200_LINKLED_PORT->INTCTRL = (W5200_LINKLED_PORT->INTCTRL & ~ PORT_INT1LVL_gm) | PORT_INT1LVL_LO_gc;
  //sense falling edge
  //W5200_LINKLED_PORT->PIN2CTRL = IOPORT_SENSE_FALLING;
  
  local_port=0;
  
  w5200_reset();
}


/**
@brief  This function performs a hard reset of the w5200. 
*/ 
void w5200_reset(void)
{  
  uint8_t i;
  uint8_t rx_mem_map[8]=W5200_DEFAULT_RX_MEM_MAP;
  uint8_t tx_mem_map[8]=W5200_DEFAULT_TX_MEM_MAP;
  
  atomMutexGet(&w5200_mutex, W5200_TIMEOUT);
    
  ioport_set_pin_level(W5200_PWDN, false);
  ioport_set_pin_level(W5200_nRST, false);
  _delay_us(10);
  ioport_set_pin_level(W5200_nRST, true);
  atomTimerDelay(SYSTEM_TICKS_PER_SEC/8);
  
  w5200_config_mem(tx_mem_map, rx_mem_map);
  
  //Set PTR and RCR register  
  //w5200_set_timeout(W5200_DEFAULT_RETRY_TIMER);
  //w5200_set_retries(W5200_DEFAULT_NO_OF_RETRIES);
  //w5200_write(W5200_INTLEVEL0, 0x08);
  
  w5200_write(W5200_IMR, 0x00);
  w5200_write(W5200_IMR2, (1 << W5200_MAX_SOCK_NUM)-1);
  
  atomSemResetCount(&(w5200_ir_sem), 0);
  atomSemResetCount(&(w5200_spi_sem), 0);
  for(i=0;i<W5200_MAX_SOCK_NUM;i++) {
    w5200_write(W5200_Sn_IMR(i), 0xFF);
    w5200_write(W5200_Sn_IR(i), 0xFF);
  }
  
  atomMutexPut(&w5200_mutex);
}



/**
@brief  This function set the transmit & receive buffer size as per the channels is used
Note for TMSR and RMSR bits are as follows\n
bit 1-0 : memory size of channel #0 \n
bit 3-2 : memory size of channel #1 \n
bit 5-4 : memory size of channel #2 \n
bit 7-6 : memory size of channel #3 \n
bit 9-8 : memory size of channel #4 \n
bit 11-10 : memory size of channel #5 \n
bit 12-12 : memory size of channel #6 \n
bit 15-14 : memory size of channel #7 \n
Maximum memory size for Tx, Rx in the W5200 is 16K Bytes,\n
In the range of 16KBytes, the memory size could be allocated dynamically by each channel.\n
Be attentive to sum of memory size shouldn't exceed 8Kbytes\n
and to data transmission and reception from non-allocated channel may cause some problems.\n
If the 16KBytes memory is already  assigned to certain channel, \n
other 3 channels couldn't be used, for there's no available memory.\n
If two 4KBytes memory are assigned to two each channels, \n
other 2 channels couldn't be used, for there's no available memory.\n
*/ 
void w5200_config_mem( uint8_t *tx_size, uint8_t *rx_size  )
{
  uint16_t i;
  uint16_t ssum,rsum;

  ssum = 0;
  rsum = 0;
  
  W5200_SBUFBASEADDRESS[0] = (uint16_t)(__DEF_IINCHIP_MAP_TXBUF__);    /* Set base address of Tx memory for channel #0 */
  W5200_RBUFBASEADDRESS[0] = (uint16_t)(__DEF_IINCHIP_MAP_RXBUF__);    /* Set base address of Rx memory for channel #0 */

  for (i = 0 ; i < W5200_MAX_SOCK_NUM; i++)       // Set the size, masking and base address of Tx & Rx memory by each channel
  {
    w5200_write((W5200_Sn_TXMEM_SIZE(i)),tx_size[i]);
    w5200_write((W5200_Sn_RXMEM_SIZE(i)),rx_size[i]);

    W5200_SSIZE[i] = (uint16_t)(0);
    W5200_RSIZE[i] = (uint16_t)(0);

    if (ssum <= 16384)
    {
      switch( tx_size[i] )
      {
      case 1:
        W5200_SSIZE[i] = (uint16_t)(1024);
        W5200_SMASK[i] = (uint16_t)(0x03FF);
        break;
      case 2:
        W5200_SSIZE[i] = (uint16_t)(2048);
        W5200_SMASK[i] = (uint16_t)(0x07FF);
        break;
      case 4:
        W5200_SSIZE[i] = (uint16_t)(4096);
        W5200_SMASK[i] = (uint16_t)(0x0FFF);
        break;
      case 8:
        W5200_SSIZE[i] = (uint16_t)(8192);
        W5200_SMASK[i] = (uint16_t)(0x1FFF);
        break;
      case 16:
        W5200_SSIZE[i] = (uint16_t)(16384);
        W5200_SMASK[i] = (uint16_t)(0x3FFF);
      break;
      }
    }

    if (rsum <= 16384)
    {
      switch( rx_size[i] )
      {
      case 1:
        W5200_RSIZE[i] = (uint16_t)(1024);
        W5200_RMASK[i] = (uint16_t)(0x03FF);
        break;
      case 2:
        W5200_RSIZE[i] = (uint16_t)(2048);
        W5200_RMASK[i] = (uint16_t)(0x07FF);
        break;
      case 4:
        W5200_RSIZE[i] = (uint16_t)(4096);
        W5200_RMASK[i] = (uint16_t)(0x0FFF);
        break;
      case 8:
        W5200_RSIZE[i] = (uint16_t)(8192);
        W5200_RMASK[i] = (uint16_t)(0x1FFF);
        break;
      case 16:
        W5200_RSIZE[i] = (uint16_t)(16384);
        W5200_RMASK[i] = (uint16_t)(0x3FFF);
        break;
      }
    }
    ssum += W5200_SSIZE[i];
    rsum += W5200_RSIZE[i];

    if (i != 0) // Sets base address of Tx and Rx memory for channel #1,#2,#3
    {
      W5200_SBUFBASEADDRESS[i] = W5200_SBUFBASEADDRESS[i-1] + W5200_SSIZE[i-1];
      W5200_RBUFBASEADDRESS[i] = W5200_RBUFBASEADDRESS[i-1] + W5200_RSIZE[i-1];
    }
  }
}



uint8_t w5200_get_link_state() {
    return w5200_read(W5200_PHY) & 0x20;
}

/**
@brief  This function sets up gateway IP address.
@arg Pointer to a 4 -byte array responsible to set the Gateway IP address.
*/ 
void w5200_set_gateway(uint8_t *gw)
{
  w5200_write((W5200_GAR0 + 0),gw[0]);
  w5200_write((W5200_GAR0 + 1),gw[1]);
  w5200_write((W5200_GAR0 + 2),gw[2]);
  w5200_write((W5200_GAR0 + 3),gw[3]);
}

/**
@brief  This function gets the gateway IP address.
*/
void w5200_get_gateway(uint8_t *gw)
{
  gw[0] = w5200_read(W5200_GAR0);
  gw[1] = w5200_read(W5200_GAR0+1);
  gw[2] = w5200_read(W5200_GAR0+2);
  gw[3] = w5200_read(W5200_GAR0+3);
}

/**
@brief  It sets up SubnetMask address
@arg Pointer to a 4 -byte array responsible to set the SubnetMask address
*/ 
void w5200_set_subnet(uint8_t *subnet)
{
  w5200_write((W5200_SUBR0 + 0),subnet[0]);
  w5200_write((W5200_SUBR0 + 1),subnet[1]);
  w5200_write((W5200_SUBR0 + 2),subnet[2]);
  w5200_write((W5200_SUBR0 + 3),subnet[3]);
}

/**
@brief  It reads the SubnetMask address
*/ 
void w5200_get_subnet(uint8_t *subnet)
{
  subnet[0] = w5200_read(W5200_SUBR0);
  subnet[1] = w5200_read(W5200_SUBR0+1);
  subnet[2] = w5200_read(W5200_SUBR0+2);
  subnet[3] = w5200_read(W5200_SUBR0+3);
}

/**
@brief  This function sets up MAC address.
*/ 
void w5200_set_mac (
  uint8_t *mac  /**< a pointer to a 6 -byte array responsible to set the MAC address. */
  )
{
  //w5200_write_burst(W5200_SHAR0, mac, 6);
  
  w5200_write((W5200_SHAR0 + 0),mac[0]);
  w5200_write((W5200_SHAR0 + 1),mac[1]);
  w5200_write((W5200_SHAR0 + 2),mac[2]);
  w5200_write((W5200_SHAR0 + 3),mac[3]);
  w5200_write((W5200_SHAR0 + 4),mac[4]);
  w5200_write((W5200_SHAR0 + 5),mac[5]);
  
}

/**
@brief  This function gets the MAC address.
*/ 
void w5200_get_mac(uint8_t *mac)
{
  mac[0] = w5200_read(W5200_SHAR0);
  mac[1] = w5200_read(W5200_SHAR0+1);
  mac[2] = w5200_read(W5200_SHAR0+2);
  mac[3] = w5200_read(W5200_SHAR0+3);
  mac[4] = w5200_read(W5200_SHAR0+4);
  mac[5] = w5200_read(W5200_SHAR0+5);
}


/**
@brief  This function sets up Source IP address.
*/
void w5200_set_source_ip(
  uint8_t *ip  /**< a pointer to a 4 -byte array responsible to set the Source IP address. */
  )
{
  w5200_write((W5200_SIPR0 + 0),ip[0]);
  w5200_write((W5200_SIPR0 + 1),ip[1]);
  w5200_write((W5200_SIPR0 + 2),ip[2]);
  w5200_write((W5200_SIPR0 + 3),ip[3]);
}

/**
@brief  This function gets up Source IP address.
*/
void w5200_get_source_ip(uint8_t *ip)
{
  ip[0] = w5200_read(W5200_SIPR0);
  ip[1] = w5200_read(W5200_SIPR0+1);
  ip[2] = w5200_read(W5200_SIPR0+2);
  ip[3] = w5200_read(W5200_SIPR0+3);
}



/**
@brief  This function gets Interrupt register in common register.
 */
uint8_t w5200_get_interrupt( void )
{
   return w5200_read(W5200_IR);
}

/**
@brief  This function set the interrupt mask Enable/Disable appropriate Interrupt. 
('1' : interrupt enable)

If any bit in IMR is set as '0' then there is not interrupt signal though the bit is
set in IR register.
*/
void w5200_set_interrupt_mask(uint8_t mask)
{
  w5200_write(W5200_IMR, mask);
}


/**
@brief  This function sets up Retransmission time.

If there is no response from the peer or delay in response then retransmission 
will be there as per RTR (Retry Time-value Register)setting
*/
void w5200_set_timeout(uint16_t timeout)
{
  w5200_write(W5200_RTR,(uint8_t)((timeout & 0xFF00) >> 8));
  w5200_write((W5200_RTR + 1),(uint8_t)(timeout & 0x00FF));
}

/**
@brief  This function set the number of Retransmission.

If there is no response from the peer or delay in response then recorded time 
as per RTR & RCR register setting then time out will occur.
*/
void w5200_set_retries(uint8_t retry)
{
  w5200_write(W5200_RCR, retry);
}




/**
@brief  This sets the maximum segment size of TCP in Active Mode), while in Passive Mode this is set by peer
*/
void w5200_set_max_segment_size(SOCKET s, uint16_t mss)
{
  w5200_write(W5200_Sn_MSSR0(s),(uint8_t)((mss & 0xFF00) >> 8));
  w5200_write((W5200_Sn_MSSR0(s) + 1),(uint8_t)(mss & 0x00FF));
}


void w5200_set_ttl(SOCKET s, uint8_t ttl)
{
   w5200_write(W5200_Sn_TTL(s), ttl);
}


/**
@brief  These below function is used to setup the Protocol Field of IP Header when
    executing the IP Layer RAW mode.
*/
void w5200_set_protocol_field(SOCKET s, uint8_t proto)
{
  w5200_write(W5200_Sn_PROTO(s), proto);
}


/**
@brief  get socket interrupt status

These below functions are used to read the Interrupt & Socket Status register
*/
uint8_t w5200_get_socket_interrupt(SOCKET s)
{
   return w5200_read(W5200_Sn_IR(s));
}

/**
@brief set socket interrupt status

These below functions are used to read the Interrupt & Socket Status register
*/
void w5200_clear_socket_interrupt(SOCKET s, uint8_t val)
{
   w5200_write(W5200_Sn_IR(s), val);
}



/**
@brief   get socket status
*/
uint8_t w5200_get_socket_status(SOCKET s)
{
   return w5200_read(W5200_Sn_SR(s));
}

/**
@brief   get socket status
*/
uint8_t w5200_get_socket_command_reg(SOCKET s)
{
   return w5200_read(W5200_Sn_CR(s));
}


/**
@brief  get socket TX free buf size

This gives free buffer size of transmit buffer. This is the data size that user can transmit.
User should check this value first and control the size of transmitting data
*/
uint16_t w5200_get_tx_buf_free(SOCKET s)
{
  uint16_t len=0;
  len = w5200_read(W5200_Sn_TX_FSR0(s));
  len = (len << 8) + w5200_read(W5200_Sn_TX_FSR0(s) + 1);
  return len;
}


/**
@brief   get socket RX receive buf size

This gives size of received data in receive buffer. 
*/
uint16_t w5200_get_rx_buf_len(SOCKET s)
{
  uint16_t len1=1, len2=2;
  while(len1!=len2) {
    len1 = w5200_read(W5200_Sn_RX_RSR0(s));
    len1 = (len1 << 8) + w5200_read(W5200_Sn_RX_RSR0(s) + 1);
    atomTimerDelay(1);
    len2 = w5200_read(W5200_Sn_RX_RSR0(s));
    len2 = (len2 << 8) + w5200_read(W5200_Sn_RX_RSR0(s) + 1);
  }
  return len1;
}


/**
@brief  This Socket function initialize the channel in particular mode and set the port.

@return   1 for success else 0.
*/  
int8_t w5200_socket(SOCKET s)
{
  uint8_t p;
  w5200_socket_t *socket = &w5200_sockets[s];
  
  if(socket->port == 0) socket->port = ++local_port;
  p=socket->protocol;
  if ((p == W5200_Sn_MR_TCP) 
      || (p == W5200_Sn_MR_UDP) 
      || (p == W5200_Sn_MR_IPRAW) 
      || (p == W5200_Sn_MR_MACRAW) 
      || (p == W5200_Sn_MR_PPPOE))
  {
    if(atomMutexGet(&w5200_mutex, W5200_TIMEOUT)!=ATOM_OK) {
      return ERR_TIMEOUT;
    }
  
    w5200_write(W5200_Sn_MR(s), socket->protocol | socket->flag);
    w5200_write(W5200_Sn_PORT0(s), (uint8_t)((socket->port & 0xFF00) >> 8));
    w5200_write((W5200_Sn_PORT0(s) + 1), (uint8_t)(socket->port & 0x00FF));
    w5200_write(W5200_Sn_CR(s), W5200_Sn_CR_OPEN);
    
    while( w5200_read(W5200_Sn_CR(s)));
    
    atomMutexPut(&w5200_mutex);

    return STATUS_OK;
  }
  else
  {
    return ERR_INVALID_ARG;
  }
}


/**
@brief  This function close the socket
*/ 
int8_t w5200_close(SOCKET s)
{
  if(atomMutexGet(&w5200_mutex, W5200_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  w5200_write(W5200_Sn_CR(s), W5200_Sn_CR_CLOSE);
  
  /* wait to process the command... */
  while( w5200_read(W5200_Sn_CR(s)) );
  
  atomMutexPut(&w5200_mutex);
  
  return STATUS_OK;
}


/**
@brief  This function establishes the connection for the channel in passive (server) mode. 
This function waits for the request from the peer.

@return  1 for success else 0.
*/ 
int8_t w5200_listen(SOCKET s)
{
  if(atomMutexGet(&w5200_mutex, W5200_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
    
  if (w5200_read(W5200_Sn_SR(s)) == W5200_SOCK_INIT)
  {
    w5200_write(W5200_Sn_CR(s),W5200_Sn_CR_LISTEN);
    /* wait to process the command... */
    while( w5200_read(W5200_Sn_CR(s)) ) ;
  }
  else
  {
    atomMutexPut(&w5200_mutex);
    return ERR_IO_ERROR;
  }
  
  atomMutexPut(&w5200_mutex);
  return STATUS_OK;
}


/**
@brief  This function establish the connection for the socket in active (client) mode. 
This function waits for the until the connection is established.
    
@return  1 for success else 0.
*/ 
int8_t w5200_connect(SOCKET s, uint8_t *addr, uint16_t port)
{
  int8_t status;
  
  if(((addr[0] == 0xFF) && (addr[1] == 0xFF) && (addr[2] == 0xFF) && (addr[3] == 0xFF)) ||
     ((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
     (port == 0x00)) 
  {
    return ERR_INVALID_ARG;
  }
  else
  {
    if(atomMutexGet(&w5200_mutex, W5200_TIMEOUT)!=ATOM_OK) {
      return ERR_TIMEOUT;
    }
    
    w5200_clear_socket_interrupt(s, 0xFF);
    
    // set destination IP
    w5200_write(W5200_Sn_DIPR0(s), addr[0]);
    w5200_write((W5200_Sn_DIPR0(s) + 1), addr[1]);
    w5200_write((W5200_Sn_DIPR0(s) + 2), addr[2]);
    w5200_write((W5200_Sn_DIPR0(s) + 3), addr[3]);
    w5200_write(W5200_Sn_DPORT0(s), (uint8_t)((port & 0xFF00) >> 8));
    w5200_write((W5200_Sn_DPORT0(s) + 1),(uint8_t)(port & 0x00FF));
    w5200_write(W5200_Sn_CR(s), W5200_Sn_CR_CONNECT);
    
    /* wait to process the command... */
    while( w5200_read(W5200_Sn_CR(s)) );
    
    /* wait for connect interrupt*/
    status=ERR_TIMEOUT;
    while(atomSemGet(&w5200_ir_sem, 5*W5200_TIMEOUT)==ATOM_OK) {
      if(w5200_get_socket_interrupt(s) & W5200_Sn_IR_CON) {
        w5200_clear_socket_interrupt(s, 0xFF);
        status=STATUS_OK;
        break;
      }
    }
    
    atomMutexPut(&w5200_mutex);
  }
  
  return status;
}



/**
@brief  This function used for disconnect the socket and parameter is "s" which represent the socket number
@return  1 for success else 0.
*/ 
int8_t w5200_disconnect(SOCKET s)
{
  int8_t status;
  
  w5200_clear_socket_interrupt(s, 0xFF);
  
  w5200_write(W5200_Sn_CR(s),W5200_Sn_CR_DISCON);
  
  /* wait to process the command... */
  while( w5200_read(W5200_Sn_CR(s)) );
  
  /* wait for connect interrupt*/
  status=ERR_TIMEOUT;
  while(atomSemGet(&w5200_ir_sem, 5*W5200_TIMEOUT)==ATOM_OK) {
    if(w5200_get_socket_interrupt(s) & W5200_Sn_IR_DISCON) {
      w5200_clear_socket_interrupt(s, 0xFF);
      status=STATUS_OK;
      break;
    }
  }
    
  return status;
}


/**
@brief  This function used to send the data in TCP mode
@return  1 for success else 0.
*/
int8_t w5200_send_tcp(SOCKET s, io_input_stream_t *src, uint16_t len)
{
  uint8_t status=0;  
  uint16_t freesize=0;
  
  if(atomMutexGet(&w5200_mutex, W5200_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  //check size not to exceed MAX size.
  if (len > W5200_SSIZE[s]) {
    atomMutexPut(&w5200_mutex);
    return ERR_INVALID_ARG;
  }
  
  // if free buffer space is available then start.
  do 
  {
    freesize = w5200_get_tx_buf_free(s);
    status = w5200_read(W5200_Sn_SR(s));
    if ((status != W5200_SOCK_ESTABLISHED) && (status != W5200_SOCK_CLOSE_WAIT))
    {
      atomMutexPut(&w5200_mutex);
      return ERR_IO_ERROR;;
    }

  } while (freesize < len);
  
  // copy data
  w5200_send_data_processing(s, src, len);
  
  w5200_write(W5200_Sn_CR(s), W5200_Sn_CR_SEND);
  
  /* wait to process the command... */
  while( w5200_read(W5200_Sn_CR(s)) );
  
  atomMutexPut(&w5200_mutex);
  
  return STATUS_OK;
}



/**
@brief  This function is an application I/F function which is used to send the data for other then TCP mode. 
Unlike TCP transmission, The peer's destination address and the port is needed.

@return  This function return send data size for success else -1.
*/ 
int8_t w5200_send(SOCKET s, io_input_stream_t *src, uint16_t *len, uint8_t *ip, uint16_t port)
{
  if (*len > W5200_SSIZE[s]) *len = W5200_SSIZE[s]; // check size not to exceed MAX size.
  
  if(((ip[0] == 0x00) && (ip[1] == 0x00) && (ip[2] == 0x00) && (ip[3] == 0x00)) ||
      ((port == 0x00)) || (*len == 0)) 
  {
     /* added return value */
     return ERR_INVALID_ARG;
  }
  else
  {
    if(atomMutexGet(&w5200_mutex, W5200_TIMEOUT)!=ATOM_OK) {
      return ERR_TIMEOUT;
    }
    
    w5200_write(W5200_Sn_DIPR0(s), ip[0]);
    w5200_write((W5200_Sn_DIPR0(s) + 1), ip[1]);
    w5200_write((W5200_Sn_DIPR0(s) + 2), ip[2]);
    w5200_write((W5200_Sn_DIPR0(s) + 3), ip[3]);
    w5200_write(W5200_Sn_DPORT0(s), (uint8_t)((port & 0xFF00) >> 8));
    w5200_write((W5200_Sn_DPORT0(s) + 1),(uint8_t)(port & 0x00FF));
    //copy data
    w5200_send_data_processing(s, src, *len);
     //send the copied data
    w5200_write(W5200_Sn_CR(s),W5200_Sn_CR_SEND);
    
    /* wait to process the command... */
    while( w5200_read(W5200_Sn_CR(s)) );
    
    atomMutexPut(&w5200_mutex);

  }
  return STATUS_OK;
}


int8_t w5200_wait_for_data(SOCKET s, uint16_t len, int32_t timeout) {
  
  while(w5200_get_rx_buf_len(s)<len) {
    
    atomSemResetCount(&w5200_ir_sem, 0);
    
    //clear socket interrupt register
    w5200_write(W5200_Sn_IR(s), 0xFF);
    
    //wait for interrupt
    if(atomSemGet(&w5200_ir_sem, timeout)!=STATUS_OK) {
      if(w5200_get_rx_buf_len(s)<len) {
        return ERR_TIMEOUT;
      }
    }
  }
  return STATUS_OK;
}



/**
@brief  This function is an application I/F function which is used to receive the data in other then
TCP mode. This function is used to receive UDP, IP_RAW and MAC_RAW mode, and handle the header as well. 
  
@return  This function return received data size for success else -1.
*/ 
int8_t w5200_receive(SOCKET s, io_output_stream_t *dst, uint16_t *len, uint8_t *ip, uint16_t *port)
{
  uint8_t head[8];
  uint16_t data_len=0;
  uint16_t ptr=0;
  io_output_stream_t head_stream;
  
  head_stream=io_output_stream_from_buffer(head, 8);
  
  if(atomMutexGet(&w5200_mutex, W5200_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
    
  //clear RECV interrupt
  w5200_write(W5200_Sn_IR(s), W5200_Sn_IR_RECV);
  
  //read rx address
  ptr = w5200_read(W5200_Sn_RX_RD0(s));
  ptr = ((ptr & 0x00FF) << 8) + w5200_read(W5200_Sn_RX_RD0(s) + 1);

  switch (w5200_read(W5200_Sn_MR(s)) & 0x07)
  {
    case W5200_Sn_MR_TCP :
      
      w5200_read_data(s, (uint8_t *)ptr, dst, *len); // data copy.
      ptr += *len;

      w5200_write(W5200_Sn_RX_RD0(s),(uint8_t)((ptr & 0xFF00) >> 8));
      w5200_write((W5200_Sn_RX_RD0(s) + 1),(uint8_t)(ptr & 0x00FF));
      break;
       
    case W5200_Sn_MR_UDP :
      w5200_read_data(s, (uint8_t *)ptr, &head_stream, 0x08);
      ptr += 8;
      // read peer's IP address, port number.
      ip[0] = head[0];
      ip[1] = head[1];
      ip[2] = head[2];
      ip[3] = head[3];
      *port = head[4];
      *port = (*port << 8) + head[5];
      data_len = head[6];
      data_len = (data_len << 8) + head[7];
        
      w5200_read_data(s, (uint8_t *)ptr, dst, data_len); // data copy.
      ptr += data_len;

      w5200_write(W5200_Sn_RX_RD0(s),(uint8_t)((ptr & 0xFF00) >> 8));
      w5200_write((W5200_Sn_RX_RD0(s) + 1),(uint8_t)(ptr & 0x00FF));
      break;
    
    case W5200_Sn_MR_IPRAW :
      w5200_read_data(s, (uint8_t *)ptr, &head_stream, 0x06);
      ptr += 6;
   
      ip[0] = head[0];
      ip[1] = head[1];
      ip[2] = head[2];
      ip[3] = head[3];
      data_len = head[4];
      data_len = (data_len << 8) + head[5];
    
      w5200_read_data(s, (uint8_t *)ptr, dst, data_len); // data copy.
      ptr += data_len;

      w5200_write(W5200_Sn_RX_RD0(s), (uint8_t)((ptr & 0xFF00) >> 8));
      w5200_write((W5200_Sn_RX_RD0(s) + 1),(uint8_t)(ptr & 0x00FF));
      break;
       
    case W5200_Sn_MR_MACRAW:
      w5200_read_data(s, (uint8_t *)ptr, &head_stream, 2);
      ptr+=2;
      data_len = head[0];
      data_len = (data_len<<8) + head[1] - 2;
      if(data_len > 1514) 
      {
        atomMutexPut(&w5200_mutex);
        return ERR_IO_ERROR;
      }

      w5200_read_data(s,(uint8_t *)ptr, dst, data_len);
      ptr += data_len;
      w5200_write(W5200_Sn_RX_RD0(s), (uint8_t)((ptr & 0xFF00) >> 8));
      w5200_write((W5200_Sn_RX_RD0(s) + 1), (uint8_t)(ptr & 0x00FF));
            
      break;

    default :
      break;
  }
    
  w5200_write(W5200_Sn_CR(s), W5200_Sn_CR_RECV);
  
  /* wait to process the command... */
  while( w5200_read(W5200_Sn_CR(s)) ) ;
  
  atomMutexPut(&w5200_mutex);
  
  return STATUS_OK;
}



/**
@brief  This function is an application I/F function which is used to receive the data in TCP mode.
  
@return  This function return status code.
*/ 
int8_t w5200_receive_tcp(SOCKET s, io_output_stream_t *dst, uint16_t len)
{
  uint16_t ptr=0;
  uint16_t l, left;
  
  left=len;
  
  while(left>0) {
    
    if(atomMutexGet(&w5200_mutex, W5200_TIMEOUT)!=ATOM_OK) {
      return ERR_TIMEOUT;
    }
  
    l=w5200_get_rx_buf_len(s);
    if(l > left) {
      l=left;
    }
    
    //read rx address
    ptr = w5200_read(W5200_Sn_RX_RD0(s));
    ptr = ((ptr & 0x00FF) << 8) + w5200_read(W5200_Sn_RX_RD0(s) + 1);
  
    w5200_read_data(s, (uint8_t *)ptr, dst, l); // data copy.
    ptr += l;
  
    w5200_write(W5200_Sn_RX_RD0(s),(uint8_t)((ptr & 0xFF00) >> 8));
    w5200_write((W5200_Sn_RX_RD0(s) + 1),(uint8_t)(ptr & 0x00FF));
    w5200_write(W5200_Sn_CR(s), W5200_Sn_CR_RECV);
  
    /* wait to process the command... */
    while( w5200_read(W5200_Sn_CR(s)) ) ;
    
    atomMutexPut(&w5200_mutex);
  
    left-=l;
  }
  
  return STATUS_OK;
}



/**
@brief  for copy the data form application buffer to transmit buffer of the chip.

This function is being used for copy the data form application buffer to Transmit
buffer of the chip. It calculate the actual physical address where one has to write
the data in transmit buffer. It also takes care of the condition while it exceed
the tx memory upper bound of socket.
*/
static void w5200_write_data(SOCKET s, io_input_stream_t *src, uint8_t *dst, uint16_t len)
{
  uint16_t size;
  uint16_t dst_mask;
  uint8_t *dst_ptr;

  dst_mask = (uint16_t)dst & W5200_SMASK[s];
  dst_ptr = (uint8_t *)(W5200_SBUFBASEADDRESS[s] + dst_mask);
  
  if (dst_mask + len > W5200_SSIZE[s]) 
  {
    size = W5200_SSIZE[s] - dst_mask;
    w5200_write_buf((uint16_t)dst_ptr, src, size);
    size = len - size;
    dst_ptr = (uint8_t *)(W5200_SBUFBASEADDRESS[s]);
    w5200_write_buf((uint16_t)dst_ptr, src, size);
  } 
  else
  {
    w5200_write_buf((uint16_t)dst_ptr, src, len);
  }
}


/**
@brief  This function is being used for copy the data form Receive buffer of the chip to application buffer.

It calculate the actual physical address where one has to read
the data from Receive buffer. It also takes care of the condition if it exceeds
the Rx memory upper-bound of socket.
*/
static void w5200_read_data(SOCKET s, uint8_t *src, io_output_stream_t *dst, uint16_t len)
{
  uint16_t size;
  uint16_t src_mask;
  uint16_t src_ptr;

  src_mask = (uint16_t)src & W5200_RMASK[s];
  src_ptr = W5200_RBUFBASEADDRESS[s] + src_mask;
  
  if( (src_mask + len) > W5200_RSIZE[s] ) 
  {
    size = W5200_RSIZE[s] - src_mask;
    w5200_read_buf(src_ptr, dst, size);
    size = len - size;
    src_ptr = W5200_RBUFBASEADDRESS[s];
    w5200_read_buf(src_ptr, dst, size);
  } 
  else
  {
    w5200_read_buf(src_ptr, dst, len);
  }
}


/**
@brief   This function is being called by send() and sendto() function also. 

This function read the Tx write pointer register and after copy the data in 
buffer update the Tx write pointer
register. User should read upper byte first and lower byte later to get proper value.
*/
static void w5200_send_data_processing(SOCKET s, io_input_stream_t *src, uint16_t len)
{
  
  uint16_t ptr;
  ptr = w5200_read(W5200_Sn_TX_WR0(s));
  ptr = (ptr << 8) + w5200_read(W5200_Sn_TX_WR0(s) + 1);
  w5200_write_data(s, src, (uint8_t *)(ptr), len);
  ptr += len;

  w5200_write(W5200_Sn_TX_WR0(s),(uint8_t)((ptr & 0xFF00) >> 8));
  w5200_write((W5200_Sn_TX_WR0(s) + 1),(uint8_t)(ptr & 0x00FF));
  
}

