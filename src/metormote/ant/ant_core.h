#ifndef ANT_CORE_H
#define ANT_CORE_H

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

#include <usart.h>
#include <board.h>
#include <status_codes.h>
#include <util/delay.h>
#include <sleepmgr.h>
#include <sysclk.h>
#include "mem.h"
#include "io_stream.h"
#include "atom.h"
#include "atomqueue.h"
#include "atomsem.h"
#include "atommutex.h"
#include "atomtimer.h"


#define ANT_NO_OF_CHANNELS            8

#define ANT_CORE_RX_QUEUE_SIZE        256
#define ANT_CORE_EVENT_QUEUE_SIZE     16
#define ANT_CORE_TX_QUEUE_SIZE        16
#define ANT_CORE_RECEIVE_TIMEOUT      60*SYSTEM_TICKS_PER_SEC
#define ANT_CORE_TIMEOUT              SYSTEM_TICKS_PER_SEC

#define ANT_SYNC_BYTE                 0xA4

//channel types
#define ANT_SLAVE                0x00
#define ANT_MASTER               0x10
#define ANT_SHARED_SLAVE         0x20
#define ANT_SHARED_MASTER        0x30
#define ANT_RECEIVE_ONLY         0x40
#define ANT_TRANSMIT_ONLY        0x50

//extended channel configuration
#define ANT_ENABLE_BACKGROUND_SCANNING     0x01
#define ANT_ENABLE_FREQUENCY_AGILITY       0x04



//configuration messages
#define ANT_CONF_UNASSIGN_CHANNEL              0x41
#define ANT_CONF_ASSIGN_CHANNEL                0x42
#define ANT_CONF_CHANNEL_ID                    0x51
#define ANT_CONF_CHANNEL_PERIOD                0x43
#define ANT_CONF_SEARCH_TIMEOUT                0x44
#define ANT_CONF_CHANNEL_RF_FREQUENCY          0x45
#define ANT_CONF_SET_NETWORK                   0x46
#define ANT_CONF_TRANSMIT_POWER                0x47
#define ANT_CONF_ID_LIST_ADD                   0x59
#define ANT_CONF_ID_LIST_CONFIG                0x5A
#define ANT_CONF_CHANNEL_TX_POWER              0x60
#define ANT_CONF_LOW_PRIO_SEARCH_TIMEOUT       0x63
#define ANT_CONF_SERIAL_NO_SET_CHANNEL_ID      0x65
#define ANT_CONF_ENABLE_EXT_RX_MSG             0x66
#define ANT_CONF_ENABLE_LED                    0x68
#define ANT_CONF_CRYSTAL_ENABLE                0x6D
#define ANT_CONF_FREQUENCY_AGILITY             0x70
#define ANT_CONF_PROXIMITY_SEARCH              0x71
#define ANT_CONF_SEARCH_PRIORITY               0x75

#define ANT_NOTIFICATION_STARTUP_MESSAGE       0x6F

#define ANT_CTRL_SYSTEM_RESET                  0x4A
#define ANT_CTRL_OPEN_CHANNEL                  0x4B
#define ANT_CTRL_CLOSE_CHANNEL                 0x4C
#define ANT_CTRL_OPEN_RX_SCAN_MODE             0x5B
#define ANT_CTRL_REQUEST_MESSAGE               0x4D
#define ANT_CTRL_SLEEP_MESSAGE                 0xC5

#define ANT_DATA_BROADCAST                     0x4E
#define ANT_DATA_ACKNOWLEDGE                   0x4F
#define ANT_DATA_BURST_TRANSFER                0x50

#define ANT_EVENT_CHANNEL_RESPONSE             0x40

#define ANT_REQUEST_CHANNEL_STATUS             0x52
#define ANT_REQUEST_CHANNEL_ID                 0x51
#define ANT_REQUEST_ANT_VERSION                0x3E
#define ANT_REQUEST_CAPABILITIES               0x54
#define ANT_REQUEST_SERIAL_NO                  0x61

#define ANT_TEST_CW_INIT                       0x53
#define ANT_TEST_CW_TEST                       0x48

#define ANT_EXT_DATA_BROADCAST                 0x5D
#define ANT_EXT_DATA_ACKNOWLEDGE               0x5E
#define ANT_EXT_DATA_BURST_TRANSFER            0x5F

#define ANT_CHANNEL_STATUS_UNASSIGNED          0
#define ANT_CHANNEL_STATUS_ASSIGNED            1
#define ANT_CHANNEL_STATUS_SEARCHING           2
#define ANT_CHANNEL_STATUS_TRACKING            3

//events
#define ANT_RESPONSE_NO_ERROR                  0
#define ANT_EVENT_RX_SEARCH_TIMEOUT            1
#define ANT_EVENT_RX_FAIL                      2
#define ANT_EVENT_TX                           3
#define ANT_EVENT_TRANSFER_RX_FAILED           4
#define ANT_EVENT_TRANSFER_TX_COMPLETED        5
#define ANT_EVENT_TRANSFER_TX_FAILED           6
#define ANT_EVENT_CHANNEL_CLOSED               7
#define ANT_EVENT_RX_FAIL_GO_TO_SEARCH         8
#define ANT_EVENT_CHANNEL_COLLISION            9
#define ANT_EVENT_TRANSFER_TX_START            10
#define ANT_CHANNEL_IN_WRONG_STATE             21
#define ANT_CHANNEL_NOT_OPENED                 22
#define ANT_CHANNEL_ID_NOT_SET                 24
#define ANT_CLOSE_ALL_CHANNELS                 25
#define ANT_TRANSFER_IN_PROGRESS               31
#define ANT_TRANSFER_SEQUENCE_NUMBER_ERROR     32
#define ANT_TRANSFER_IN_ERROR                  33
#define ANT_INVALID_MESSAGE                    40
#define ANT_INVALID_NETWORK_NUMBER             41
#define ANT_INVALID_LIST_ID                    48
#define ANT_INVALID_SCAN_TX_CHANNEL            49
#define ANT_INVALID_PARAMETER_PROVIDED         51
#define ANT_EVENT_QUE_OVERFLOW                 53

#define ANT_EVENT_TIMEOUT                      255
#define ANT_EVENT_INVALID_STATE                254

#define ANT_PUBLIC_NETWORK                     0


#ifdef __cplusplus
extern "C" { 
#endif

//ant config struct
struct ant_config {
  uint8_t   master:1, shared:1, bg_scan:1, freq_agility:1;
  uint8_t   tx_type;
  uint8_t   pairing:1, device_type:7;
  uint16_t  device_no; 
  uint8_t   frequency;
  uint8_t   tx_power;
  uint16_t  period;
  uint8_t   timeout_hp;
  uint8_t   timeout_lp;
  uint8_t   freq[3];
  uint8_t   proximity;
  uint8_t   search_priority;
};

//ant frame
struct ant_frame {
  uint8_t   msg_id;
  uint8_t   data_len;
  uint8_t   data[9];
} __attribute__((packed));


//ant buffer
struct ant_event {
  uint8_t   channel;
  uint16_t  shared_address;
  uint8_t   event_code;
  uint8_t   data_len;
  io_input_stream_t *stream;
} __attribute__((packed));


struct ant_channel {
  uint8_t event_code;
  uint8_t status;
  uint8_t tx_type;
  uint8_t pairing:1, device_type:7;
  uint16_t device_no;
  uint8_t   master:1, shared:1;
  struct ant_frame broadcast_frame;
  ATOM_SEM rx_sem;
  ATOM_SEM tx_sem;
  void (*callback)(struct ant_event *);
} __attribute__((packed));



extern struct ant_channel ant_core_channels[ANT_NO_OF_CHANNELS];

void ant_core_init(void);
void ant_core_start(void (*callback)(void));
void ant_core_reset(void);
void ant_core_sleep(uint8_t state);
void ant_core_suspend(uint8_t state);
void ant_core_set_rx_port_interrupt_level(uint8_t level);
void ant_core_set_rts_port_interrupt_level(uint8_t level);
void ant_core_clear_rx_stream(uint16_t len);
int8_t ant_core_config(uint8_t channel, struct ant_config *config);

int8_t ant_core_assign_channel(uint8_t channel, uint8_t channel_type, uint8_t network, uint8_t extended);
int8_t ant_core_unassign_channel(uint8_t channel);
int8_t ant_core_set_channel_id(uint8_t channel, uint16_t device_no, uint8_t pairing, uint8_t device_type, uint8_t tx_type);
int8_t ant_core_set_channel_period(uint8_t channel, uint16_t period);
int8_t ant_core_set_channel_search_timeout(uint8_t channel, uint8_t timeout);
int8_t ant_core_set_channel_rf_frequency(uint8_t channel, uint8_t rf_freq);
int8_t ant_core_set_transmit_power(uint8_t power);
int8_t ant_core_set_channel_tx_power(uint8_t channel, uint8_t power);
int8_t ant_core_set_channel_low_prio_search_timeout(uint8_t channel, uint8_t timeout);
int8_t ant_core_config_frequency_agility(uint8_t channel, uint8_t rf_freq1, uint8_t rf_freq2, uint8_t rf_freq3);
int8_t ant_core_set_proximity_search(uint8_t channel, uint8_t threshold);
int8_t ant_core_set_channel_search_priority(uint8_t channel, uint8_t priority);

//not supported
//int8_t ant_core_set_serial_no_channel_id(uint8_t channel, uint8_t device_type, uint8_t tx_type);

//not implemented
//int8_t ant_core_add_channel_id(uint16_t device_no, uint8_t device_type, uint8_t tx_type, uint8_t index);
//int8_t ant_core_config_list_id(uint8_t channel, uint8_t list_size, uint8_t exclude);
//int8_t ant_core_set_network_key(uint8_t channel, uint8_t *key);

int8_t ant_core_enable_extended_messages(uint8_t enable);
int8_t ant_core_enable_led(uint8_t enable);
int8_t ant_core_enable_crystal(void);

int8_t ant_core_reset_system(void);
int8_t ant_core_open_channel(uint8_t channel, void (*callback)(struct ant_event *));
int8_t ant_core_close_channel(uint8_t channel);
int8_t ant_core_request_message(uint8_t channel, uint8_t message_id);
int8_t ant_core_open_rx_scan_mode(void);
int8_t ant_core_sleep_message(void);


int8_t ant_core_send_broadcast_data(uint8_t channel, uint8_t *data, bool blocking);
int8_t ant_core_send_acknowledged_data(uint8_t channel, uint8_t *data);
int8_t ant_core_send_burst_transfer(uint8_t channel, uint16_t *shared_address, uint8_t *head, uint8_t *burst, uint16_t data_len);

int8_t ant_core_init_cw_test_mode(void);
int8_t ant_core_set_cw_test_mode(uint8_t tx_power, uint8_t rf_freq);
int8_t ant_core_reset_state(void);
int8_t ant_core_set_shared_address_msg(uint8_t channel, uint16_t shared_address);
void ant_core_cleanup_channels(void);
uint8_t ant_core_channel_is_open(uint8_t channel);

#ifdef __cplusplus
}
#endif


#endif /* ANT_CORE_H */
