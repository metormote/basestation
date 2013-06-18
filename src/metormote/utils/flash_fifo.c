/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Use the AT45DBX data flash driver as a FIFO buffer through SPI.
 *
 * This file manages the accesses to the AT45DBX data flash components.
 *
 *
 ******************************************************************************/

/* Copyright (c) 2013 Metormote AB. All rights reserved.
 *
 */

#include "flash_fifo.h"


//! Boolean indicating whether memory is in busy state.
static bool at45dbx_busy;
static uint32_t last_tail;

static ATOM_MUTEX fifo_mutex;

static void flash_fifo_chipselect_df(uint8_t memidx, bool bSelect);
static void flash_fifo_wait_ready(FLASH_FIFO *fifo);
static void flash_fifo_load_buffer(FLASH_FIFO *fifo);
static void flash_fifo_save_buffer(FLASH_FIFO *fifo);


/*! \brief Returns the number of elements in the ring buffer
 *
 * \retval Number of elements in the ring buffer.
 */
uint32_t flash_fifo_count (FLASH_FIFO *fifo)
{
  uint32_t count;
  if(atomMutexGet(&fifo_mutex, FLASH_FIFO_MUTEX_TIMEOUT)!=ATOM_OK) {
    return 0;
  }
  count=(fifo ? (fifo->head - fifo->tail) : 0);
  atomMutexPut(&fifo_mutex);
  return count;
}

/*! \brief Clear the ring buffer
 *
 */
void flash_fifo_clear (FLASH_FIFO *fifo)
{
  if(atomMutexGet(&fifo_mutex, FLASH_FIFO_MUTEX_TIMEOUT)!=ATOM_OK) {
    return;
  }
  if(fifo) fifo->tail = fifo->head;
  atomMutexPut(&fifo_mutex);
}

/*! \brief Returns the empty/full status of the ring buffer.
 *
 * \retval true full.
 * \retval false not full.
 */
bool flash_fifo_full (FLASH_FIFO *fifo)
{
  bool full;
  if(atomMutexGet(&fifo_mutex, FLASH_FIFO_MUTEX_TIMEOUT)!=ATOM_OK) {
    return true;
  }
  full=(fifo ? (flash_fifo_count(fifo) == fifo->buffer_len) : true);
  atomMutexPut(&fifo_mutex);
  return full;
}


/*! \brief Returns the empty/full status of the ring buffer.
 *
 * \retval true empty.
 * \retval false not empty.
 */
bool flash_fifo_empty (FLASH_FIFO *fifo)
{
  bool empty;
  if(atomMutexGet(&fifo_mutex, FLASH_FIFO_MUTEX_TIMEOUT)!=ATOM_OK) {
    return true;
  }
  empty=(fifo ? (flash_fifo_count(fifo) == 0) : true);
  atomMutexPut(&fifo_mutex);
  return empty;
}


/*! \brief Rollbacks the last read.
 *
 * \retval none.
 */
void flash_fifo_set_mark(FLASH_FIFO *fifo) {
  if(atomMutexGet(&fifo_mutex, FLASH_FIFO_MUTEX_TIMEOUT)!=ATOM_OK) {
    return;
  }
  last_tail=fifo->tail;
  atomMutexPut(&fifo_mutex);
}

/*! \brief Rollbacks the last read.
 *
 * \retval none.
 */
void flash_fifo_rollback_to_mark(FLASH_FIFO *fifo) {
  if(atomMutexGet(&fifo_mutex, FLASH_FIFO_MUTEX_TIMEOUT)!=ATOM_OK) {
    return;
  }
  fifo->tail=last_tail;
  atomMutexPut(&fifo_mutex);
}

/*! \brief Selects or unselects a DF memory.
 *
 * \param memidx  Memory ID of DF to select or unselect.
 * \param bSelect Boolean indicating whether the DF memory has to be selected.
 */
static void flash_fifo_chipselect_df(uint8_t memidx, bool bSelect)
{
    if (bSelect) {
        // Select SPI chip.
        at45dbx_spi_select_device(memidx);
    }
    else {
     // Unselect SPI chip.
         at45dbx_spi_deselect_device(memidx);
    }
}



static void flash_fifo_wait_ready(FLASH_FIFO *fifo)
{
    uint8_t status;

    // Select the DF memory at45dbx_gl_ptr_mem points to.
    flash_fifo_chipselect_df(fifo->memidx, TRUE);

    // Send the Status Register Read command.
    at45dbx_spi_write_byte(AT45DBX_CMDC_RD_STATUS_REG);
    // Read the status register until the DF is ready.
    do {
        // Send a dummy byte to read the status register.
        at45dbx_spi_read_byte(&status);    
    } while ((status & AT45DBX_MSK_BUSY) == AT45DBX_BUSY);

    // Unselect the DF memory.
    flash_fifo_chipselect_df(fifo->memidx, FALSE);
}



static void flash_fifo_load_buffer(FLASH_FIFO *fifo) 
{
    uint32_t addr;
    
    if (at45dbx_busy) {
        flash_fifo_wait_ready(fifo);
    }
    at45dbx_busy = TRUE;

    addr = fifo->offset + (fifo->head % fifo->buffer_len);
    addr = (Rd_bitfield(addr, (uint32_t)AT45DBX_MSK_PTR_PAGE) << AT45DBX_BYTE_ADDR_BITS);
    
    flash_fifo_chipselect_df(fifo->memidx, TRUE);
    at45dbx_spi_write_byte(fifo->buffer_no==AT45DBX_BUFFER_1 ? AT45DBX_CMDB_XFR_PAGE_TO_BUF1 : AT45DBX_CMDB_XFR_PAGE_TO_BUF2);
    at45dbx_spi_write_byte(LSB2W(addr));
    at45dbx_spi_write_byte(LSB1W(addr));
    at45dbx_spi_write_byte(LSB0W(addr));

  // Unselect the DF memory at45dbx_gl_ptr_mem points to.
    flash_fifo_chipselect_df(fifo->memidx, FALSE);
}

static void flash_fifo_save_buffer(FLASH_FIFO *fifo) 
{
    uint32_t addr;

    if (at45dbx_busy) {
        flash_fifo_wait_ready(fifo);
    }
    at45dbx_busy = TRUE;

    addr = fifo->offset + (fifo->head % fifo->buffer_len);
    addr = (Rd_bitfield((uint32_t)addr, (uint32_t)AT45DBX_MSK_PTR_PAGE) << AT45DBX_BYTE_ADDR_BITS);

    // Select the DF memory.
    flash_fifo_chipselect_df(fifo->memidx, TRUE);
    at45dbx_spi_write_byte(fifo->buffer_no==AT45DBX_BUFFER_1 ? AT45DBX_CMDB_PR_BUF1_TO_PAGE_ER : AT45DBX_CMDB_PR_BUF2_TO_PAGE_ER);
    at45dbx_spi_write_byte(LSB2W(addr));
    at45dbx_spi_write_byte(LSB1W(addr));
    at45dbx_spi_write_byte(LSB0W(addr));
  
  // Unselect the DF memory.
    flash_fifo_chipselect_df(fifo->memidx, FALSE);
}


/*! \name Control Functions
 */
//! @{
bool flash_fifo_init()
{
    at45dbx_spi_init();
    //Memory ready by default.
    at45dbx_busy = FALSE;
    
    //create mutex protecting dataflash
    atomMutexCreate (&fifo_mutex);
    
    return OK;
}


uint16_t flash_fifo_read(FLASH_FIFO *fifo, uint8_t *buf, uint16_t len) 
{
    uint32_t addr;
    uint16_t i;
    uint8_t *p = buf;

    if (len==0 || flash_fifo_empty(fifo)) {
        return 0;
    }

    if(atomMutexGet(&fifo_mutex, FLASH_FIFO_MUTEX_TIMEOUT)!=ATOM_OK) {
      return 0;
    }
    
  // If the DF memory is busy, wait until it's ready.
    if (at45dbx_busy) {
        flash_fifo_wait_ready(fifo);
    }
    at45dbx_busy = FALSE;
    
    addr = fifo->offset + fifo->tail % fifo->buffer_len;
    addr = (Rd_bitfield(addr, (uint32_t)AT45DBX_MSK_PTR_PAGE) << AT45DBX_BYTE_ADDR_BITS) |
      Rd_bitfield(addr, AT45DBX_MSK_PTR_BYTE);

    // Select the DF memory.
    flash_fifo_chipselect_df(fifo->memidx, TRUE);
    at45dbx_spi_write_byte(AT45DBX_CMDA_RD_PAGE);
    at45dbx_spi_write_byte(LSB2W(addr));
    at45dbx_spi_write_byte(LSB1W(addr));
    at45dbx_spi_write_byte(LSB0W(addr));

    // Send 32 don't care clock cycles to initialize the read operation.
    at45dbx_spi_write_byte(0x55);
    at45dbx_spi_write_byte(0x55);
    at45dbx_spi_write_byte(0x55);
    at45dbx_spi_write_byte(0x55);

    for(i=0;i<len;i++) {
        // Read the next data byte.
        at45dbx_spi_read_byte(p++);

        if (flash_fifo_empty(fifo)) {
          break;
        }

        // If end of page reached
        if((fifo->tail++ & AT45DBX_MSK_PTR_BYTE)==AT45DBX_MSK_PTR_BYTE) {

            // Unselect the DF memory.
            flash_fifo_chipselect_df(fifo->memidx, FALSE);

            //start reading next page
            addr = fifo->offset + fifo->tail % fifo->buffer_len;
            addr = (Rd_bitfield(addr, (uint32_t)AT45DBX_MSK_PTR_PAGE) << AT45DBX_BYTE_ADDR_BITS) |
            Rd_bitfield(addr, AT45DBX_MSK_PTR_BYTE);

            flash_fifo_chipselect_df(fifo->memidx, TRUE);
            at45dbx_spi_write_byte(AT45DBX_CMDA_RD_PAGE);
            at45dbx_spi_write_byte(LSB2W(addr));
            at45dbx_spi_write_byte(LSB1W(addr));
            at45dbx_spi_write_byte(LSB0W(addr));

            // Send 32 don't care clock cycles to initialize the read operation.
            at45dbx_spi_write_byte(0x55);
            at45dbx_spi_write_byte(0x55);
            at45dbx_spi_write_byte(0x55);
            at45dbx_spi_write_byte(0x55);
        }
    }

    // Unselect the DF memory.
    flash_fifo_chipselect_df(fifo->memidx, FALSE);
    
    atomMutexPut(&fifo_mutex);
    
    return p-buf;
}



uint16_t flash_fifo_write(FLASH_FIFO *fifo, const uint8_t *buf, uint16_t len)
{
    uint32_t addr;
    uint16_t i;
    uint8_t *p=(uint8_t *)buf;
    
    if (len==0 || flash_fifo_full(fifo)) {
        return 0;
    }
    
    if(atomMutexGet(&fifo_mutex, FLASH_FIFO_MUTEX_TIMEOUT)!=ATOM_OK) {
      return 0;
    }
    
    // If the DF memory is busy, wait until it's ready.
    if (at45dbx_busy) {
        flash_fifo_wait_ready(fifo);
    }
    at45dbx_busy = FALSE;
  
    addr = Rd_bitfield(fifo->head, AT45DBX_MSK_PTR_BYTE);
    
    // Select the DF memory.
    flash_fifo_chipselect_df(fifo->memidx, TRUE);
    at45dbx_spi_write_byte(fifo->buffer_no==AT45DBX_BUFFER_1 ? AT45DBX_CMDC_WR_BUF1 : AT45DBX_CMDC_WR_BUF2);
    at45dbx_spi_write_byte(LSB2W(addr));
    at45dbx_spi_write_byte(LSB1W(addr));
    at45dbx_spi_write_byte(LSB0W(addr));
  
    for(i=0;i<len;i++) {
        
        // Write the next data byte.
        at45dbx_spi_write_byte(*p++);

        // If end of page reached
        if((fifo->head & AT45DBX_MSK_PTR_BYTE)==AT45DBX_MSK_PTR_BYTE) {

            // Unselect the DF memory.
            flash_fifo_chipselect_df(fifo->memidx, FALSE);

            //save the page to main memory
            flash_fifo_save_buffer(fifo);
            flash_fifo_wait_ready(fifo);
            at45dbx_busy = FALSE;

            fifo->head++;

            //load next page to buffer
            flash_fifo_load_buffer(fifo);
            flash_fifo_wait_ready(fifo);
            at45dbx_busy = FALSE;
            
            //start writing byte at address 0 in the new page buffer
            addr=0;

            // Select the DF memory.
            flash_fifo_chipselect_df(fifo->memidx, TRUE);
            at45dbx_spi_write_byte(fifo->buffer_no==AT45DBX_BUFFER_1 ? AT45DBX_CMDC_WR_BUF1 : AT45DBX_CMDC_WR_BUF2);
            at45dbx_spi_write_byte(LSB2W(addr));
            at45dbx_spi_write_byte(LSB1W(addr));
            at45dbx_spi_write_byte(LSB0W(addr));
        }
        else {
            fifo->head++;
        }
        
        if (flash_fifo_full(fifo)) {
          break;
        }
    }
  
    // Unselect the DF memory.
    flash_fifo_chipselect_df(fifo->memidx, FALSE);
  
    // Transfer the write buffer to memory
    flash_fifo_save_buffer(fifo);
    
    atomMutexPut(&fifo_mutex);
    
    return p-buf;
}

//! @}
