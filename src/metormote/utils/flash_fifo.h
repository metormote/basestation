/**
 * \file
 *
 * \brief FIFO buffer based on AT45DBX DataFlash component.
 *
 * Copyright (C) 2011 Metormote AB. All rights reserved.
 *
 */
#ifndef FLASH_FIFO_H_
#define FLASH_FIFO_H_

#include "atom.h"
#include "atommutex.h"
#include "at45dbx.h"

#define FLASH_FIFO_MUTEX_TIMEOUT     SYSTEM_TICKS_PER_SEC

#ifdef __cplusplus
extern "C" {
#endif

//NOTE buffer size must be power of two
struct flash_fifo_t {
    uint8_t  memidx;
    uint32_t offset;      /* offset into flash memory */
    uint32_t head;        /* first byte of data */
    uint32_t tail;        /* last byte of data */
    uint32_t buffer_len;  /* length of the data */
    uint8_t  buffer_no;
};
typedef struct flash_fifo_t FLASH_FIFO;


/*! \name Control Functions
 */
//! @{

/*! \brief Initializes the data flash controller and the SPI channel by which
 *         the DF is controlled.
 *
 * \retval OK Success.
 * \retval KO Failure.
 */
bool flash_fifo_init(void);
uint32_t flash_fifo_count (FLASH_FIFO *fifo);
bool flash_fifo_full (FLASH_FIFO *fifo);
bool flash_fifo_empty (FLASH_FIFO *fifo);
void flash_fifo_clear (FLASH_FIFO *fifo);
uint16_t flash_fifo_read(FLASH_FIFO *fifo, uint8_t *buf, uint16_t len);
uint16_t flash_fifo_write(FLASH_FIFO *fifo, const uint8_t *buf, uint16_t len);
void flash_fifo_set_mark(FLASH_FIFO *fifo);
void flash_fifo_rollback_to_mark(FLASH_FIFO *fifo);

//! @}

#ifdef __cplusplus
}
#endif


#endif /* FLASH_FIFO_H_ */