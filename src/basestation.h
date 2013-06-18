/*
 * basestation.h
 *
 * Created: 10/4/2011 10:10:56 AM
 *  Author: Administrator
 */ 


#ifndef BASESTATION_H_
#define BASESTATION_H_

/*
 * Include header files for all drivers that have been imported from
 * AVR Software Framework (ASF).
 */
#include "asf.h"
#include "flash_fifo.h"
#include "conf_board.h"
#include "atom.h"
#include "atomport-private.h"
#include "atommutex.h"
#include "gprs.h"
#include "ant.h"
#include "ant_tunnel.h"
#include "utils.h"
#include "base64.h"
#include "mem.h"
#include "board.h"
#include "ui.h"
#include "crypto.h"
#include "hmac_sha1.h"
#include "net/net.h"
#include "net/w5200.h"
#include "net/websocket.h"
#include "net/dns_client.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "iop.pb.h"
#include "basestation.pb.h"
#include "twi_master.h"
#include "power.h"
#include "seed.h"
#include "firmware.pb.h"

/*
 * Idle thread stack size
 *
 * This needs to be large enough to handle any interrupt handlers
 * and callbacks called by interrupt handlers (e.g. user-created
 * timer callbacks) as well as the saving of all context when
 * switching away from this thread.
 *
 * In this case, the idle stack is allocated at the end of the 
 * ram memory area.
 */
#define IDLE_THREAD_STACK_SIZE_BYTES       0xC0

/*
 * Main thread stack sizes
 *
 * The Main thread stacks generally need to be larger than the idle
 * thread stack, as not only does it need to store interrupt handler
 * stack saves and context switch saves, but the application main thread
 * will generally be carrying out more nested function calls and require
 * stack for application code local variables etc.
 *
 */
#define ANT_THREAD_STACK_SIZE_BYTES         0x200
#define ANT_THREAD_PRIO                     16

#define PROCESS_THREAD_STACK_SIZE_BYTES     0x260
#define PROCESS_THREAD_PRIO                 96

#define GPRS_THREAD_STACK_SIZE_BYTES        0x240
#define GPRS_THREAD_PRIO                    48

#define WEBSOCKET_THREAD_STACK_SIZE_BYTES   0x240
#define WEBSOCKET_THREAD_PRIO               24


#define BASESTATION_OUT_MSG_CODE           16
#define BASESTATION_IN_MSG_CODE            17

#define FIRMWARE_MSG_CODE                  5
#define FIRMWARE_RESPONSE_MSG_CODE         7


#define FIFO_BUFFER_OFFSET                 0x0 //must be multiple of 256
#define FIFO_BUFFER_LENGTH                 0x400 //must be power of 2
#define FIFO_BUFFER_NO                     AT45DBX_BUFFER_1

#define WEBSOCKET_FIFO_BUFFER_OFFSET      0x80000 //must be multiple of 256
#define WEBSOCKET_FIFO_BUFFER_LENGTH      0x80000 //must be power of 2
#define WEBSOCKET_FIFO_BUFFER_NO          AT45DBX_BUFFER_2

#define ENV_BUF_SIZE  256
#define MSG_BUF_SIZE  192

struct basestation_signature_t {
  uint64_t device_id;
  uint8_t key[AES_KEY_SIZE];
  int16_t temp_offset;
};

struct basestation_state_t {
  uint64_t network_id;
  uint16_t transfer_interval;
  uint16_t snapshot_interval;
  uint8_t send_immediate;
  uint64_t rx_nonce;
  uint64_t tx_nonce;
};


int basestation_start (void);


#endif /* BASESTATION_H_ */