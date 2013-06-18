/**
 * \file
 *
 * \brief Main
 *
 */
#include "basestation.h"

int main (void)
{
  basestation_start();
}


#define ISR_MONITORING
  
// for debug purposes only
#ifdef ISR_MONITORING

volatile uint8_t iflag;

static void unexpected_interrupt(uint8_t f) {
  iflag=f;
}



/* OSC interrupt vectors */
ISR (OSC_XOSCF_vect) {unexpected_interrupt(1);}  /* External Oscillator Failure Interrupt (NMI) */

/* PORTC interrupt vectors */
ISR (PORTC_INT0_vect) {unexpected_interrupt(2);}  /* External Interrupt 0 */
ISR (PORTC_INT1_vect) {unexpected_interrupt(3);}  /* External Interrupt 1 */

/* PORTR interrupt vectors */
ISR (PORTR_INT0_vect) {unexpected_interrupt(4);}  /* External Interrupt 0 */
ISR (PORTR_INT1_vect) {unexpected_interrupt(5);}  /* External Interrupt 1 */

/* DMA interrupt vectors */
//ISR (DMA_CH0_vect) {unexpected_interrupt(6);}  /* Channel 0 Interrupt */
//ISR (DMA_CH1_vect) {unexpected_interrupt(7);}  /* Channel 1 Interrupt */
//ISR (DMA_CH2_vect) {unexpected_interrupt(8);}  /* Channel 2 Interrupt */
//ISR (DMA_CH3_vect) {unexpected_interrupt(9);}  /* Channel 3 Interrupt */

/* RTC interrupt vectors */
//ISR (RTC_OVF_vect) {unexpected_interrupt(10);}  /* Overflow Interrupt */
//ISR (RTC_COMP_vect) {unexpected_interrupt(11);}  /* Compare Interrupt */

/* TWIC interrupt vectors */
ISR (TWIC_TWIS_vect) {unexpected_interrupt(12);}  /* TWI Slave Interrupt */
//ISR (TWIC_TWIM_vect) {unexpected_interrupt(13);}  /* TWI Master Interrupt */

/* TCC0 interrupt vectors */
//ISR (TCC0_OVF_vect) {unexpected_interrupt(14);}  /* Overflow Interrupt */
//ISR (TCC0_ERR_vect) {unexpected_interrupt(15);}  /* Error Interrupt */
//ISR (TCC0_CCA_vect) {unexpected_interrupt(16);}  /* Compare or Capture A Interrupt */
//ISR (TCC0_CCB_vect) {unexpected_interrupt(17);}  /* Compare or Capture B Interrupt */
//ISR (TCC0_CCC_vect) {unexpected_interrupt(18);}  /* Compare or Capture C Interrupt */
//ISR (TCC0_CCD_vect) {unexpected_interrupt(19);}  /* Compare or Capture D Interrupt */

/* TCC1 interrupt vectors */
//ISR (TCC1_OVF_vect) {unexpected_interrupt(20);}  /* Overflow Interrupt */
//ISR (TCC1_ERR_vect) {unexpected_interrupt(21);}  /* Error Interrupt */
//ISR (TCC1_CCA_vect) {unexpected_interrupt(22);}  /* Compare or Capture A Interrupt */
//ISR (TCC1_CCB_vect) {unexpected_interrupt(23);}  /* Compare or Capture B Interrupt */

/* SPIC interrupt vectors */
ISR (SPIC_INT_vect) {unexpected_interrupt(24);}  /* SPI Interrupt */

/* USARTC0 interrupt vectors */
//ISR (USARTC0_RXC_vect) {unexpected_interrupt(25);}  /* Reception Complete Interrupt */
//ISR (USARTC0_DRE_vect) {unexpected_interrupt(26);}  /* Data Register Empty Interrupt */
//ISR (USARTC0_TXC_vect) {unexpected_interrupt(27);}  /* Transmission Complete Interrupt */

/* USARTC1 interrupt vectors */
ISR (USARTC1_RXC_vect) {unexpected_interrupt(28);}  /* Reception Complete Interrupt */
ISR (USARTC1_DRE_vect) {unexpected_interrupt(29);}  /* Data Register Empty Interrupt */
ISR (USARTC1_TXC_vect) {unexpected_interrupt(30);}  /* Transmission Complete Interrupt */

/* AES interrupt vectors */
//ISR (AES_INT_vect) {unexpected_interrupt(31);}  /* AES Interrupt */

/* NVM interrupt vectors */
ISR (NVM_EE_vect) {unexpected_interrupt(32);}  /* EE Interrupt */
ISR (NVM_SPM_vect) {unexpected_interrupt(33);}  /* SPM Interrupt */

/* PORTB interrupt vectors */
//ISR (PORTB_INT0_vect) {unexpected_interrupt(34);}  /* External Interrupt 0 */
//ISR (PORTB_INT1_vect) {unexpected_interrupt(35);}  /* External Interrupt 1 */

/* ACB interrupt vectors */
//ISR (ACB_AC0_vect) {unexpected_interrupt(36);}  /* AC0 Interrupt */
//ISR (ACB_AC1_vect) {unexpected_interrupt(37);}  /* AC1 Interrupt */
//ISR (ACB_ACW_vect) {unexpected_interrupt(38);}  /* ACW Window Mode Interrupt */

/* ADCB interrupt vectors */
ISR (ADCB_CH0_vect) {unexpected_interrupt(39);}  /* Interrupt 0 */
ISR (ADCB_CH1_vect) {unexpected_interrupt(40);}  /* Interrupt 1 */
ISR (ADCB_CH2_vect) {unexpected_interrupt(41);}  /* Interrupt 2 */
ISR (ADCB_CH3_vect) {unexpected_interrupt(42);}  /* Interrupt 3 */

/* PORTE interrupt vectors */
ISR (PORTE_INT0_vect) {unexpected_interrupt(43);}  /* External Interrupt 0 */
ISR (PORTE_INT1_vect) {unexpected_interrupt(44);}  /* External Interrupt 1 */

/* TWIE interrupt vectors */
ISR (TWIE_TWIS_vect) {unexpected_interrupt(45);}  /* TWI Slave Interrupt */
//ISR (TWIE_TWIM_vect) {unexpected_interrupt(46);}  /* TWI Master Interrupt */

/* TCE0 interrupt vectors */
//ISR (TCE0_OVF_vect) {unexpected_interrupt(47);}  /* Overflow Interrupt */
//ISR (TCE0_ERR_vect) {unexpected_interrupt(48);}  /* Error Interrupt */
//ISR (TCE0_CCA_vect) {unexpected_interrupt(49);}  /* Compare or Capture A Interrupt */
//ISR (TCE0_CCB_vect) {unexpected_interrupt(50);}  /* Compare or Capture B Interrupt */
//ISR (TCE0_CCC_vect) {unexpected_interrupt(51);}  /* Compare or Capture C Interrupt */
//ISR (TCE0_CCD_vect) {unexpected_interrupt(52);}  /* Compare or Capture D Interrupt */

/* TCE1 interrupt vectors */
//ISR (TCE1_OVF_vect) {unexpected_interrupt(53);}  /* Overflow Interrupt */
//ISR (TCE1_ERR_vect) {unexpected_interrupt(54);}  /* Error Interrupt */
//ISR (TCE1_CCA_vect) {unexpected_interrupt(55);}  /* Compare or Capture A Interrupt */
//ISR (TCE1_CCB_vect) {unexpected_interrupt(56);}  /* Compare or Capture B Interrupt */

/* SPIE interrupt vectors */
//ISR (SPIE_INT_vect) {unexpected_interrupt(57);}  /* SPI Interrupt */

/* USARTE0 interrupt vectors */
ISR (USARTE0_RXC_vect) {unexpected_interrupt(58);}  /* Reception Complete Interrupt */
ISR (USARTE0_DRE_vect) {unexpected_interrupt(59);}  /* Data Register Empty Interrupt */
ISR (USARTE0_TXC_vect) {unexpected_interrupt(60);}  /* Transmission Complete Interrupt */

/* USARTE1 interrupt vectors */
ISR (USARTE1_RXC_vect) {unexpected_interrupt(61);}  /* Reception Complete Interrupt */
ISR (USARTE1_DRE_vect) {unexpected_interrupt(62);}  /* Data Register Empty Interrupt */
ISR (USARTE1_TXC_vect) {unexpected_interrupt(63);}  /* Transmission Complete Interrupt */

/* PORTD interrupt vectors */
//ISR (PORTD_INT0_vect) {unexpected_interrupt(64);}  /* External Interrupt 0 */
ISR (PORTD_INT1_vect) {unexpected_interrupt(65);}  /* External Interrupt 1 */

/* PORTA interrupt vectors */
//ISR (PORTA_INT0_vect) {unexpected_interrupt(66);}  /* External Interrupt 0 */
ISR (PORTA_INT1_vect) {unexpected_interrupt(67);}  /* External Interrupt 1 */

/* ACA interrupt vectors */
//ISR (ACA_AC0_vect) {unexpected_interrupt(68);}  /* AC0 Interrupt */
//ISR (ACA_AC1_vect) {unexpected_interrupt(69);}  /* AC1 Interrupt */
//ISR (ACA_ACW_vect) {unexpected_interrupt(70);}  /* ACW Window Mode Interrupt */

/* ADCA interrupt vectors */
ISR (ADCA_CH0_vect) {unexpected_interrupt(71);}  /* Interrupt 0 */
ISR (ADCA_CH1_vect) {unexpected_interrupt(72);}  /* Interrupt 1 */
ISR (ADCA_CH2_vect) {unexpected_interrupt(73);}  /* Interrupt 2 */
ISR (ADCA_CH3_vect) {unexpected_interrupt(74);}  /* Interrupt 3 */

/* TCD0 interrupt vectors */
//ISR (TCD0_OVF_vect) {unexpected_interrupt(77);}  /* Overflow Interrupt */
//ISR (TCD0_ERR_vect) {unexpected_interrupt(78);}  /* Error Interrupt */
//ISR (TCD0_CCA_vect) {unexpected_interrupt(79);}  /* Compare or Capture A Interrupt */
//ISR (TCD0_CCB_vect) {unexpected_interrupt(80);}  /* Compare or Capture B Interrupt */
//ISR (TCD0_CCC_vect) {unexpected_interrupt(81);}  /* Compare or Capture C Interrupt */
//ISR (TCD0_CCD_vect) {unexpected_interrupt(82);}  /* Compare or Capture D Interrupt */

/* TCD1 interrupt vectors */
//ISR (TCD1_OVF_vect) {unexpected_interrupt(83);}  /* Overflow Interrupt */
//ISR (TCD1_ERR_vect) {unexpected_interrupt(84);}  /* Error Interrupt */
//ISR (TCD1_CCA_vect) {unexpected_interrupt(85);}  /* Compare or Capture A Interrupt */
//ISR (TCD1_CCB_vect) {unexpected_interrupt(86);}  /* Compare or Capture B Interrupt */

/* SPID interrupt vectors */
ISR (SPID_INT_vect) {unexpected_interrupt(87);}  /* SPI Interrupt */

/* USARTD0 interrupt vectors */
//ISR (USARTD0_RXC_vect) {unexpected_interrupt(88);}  /* Reception Complete Interrupt */
//ISR (USARTD0_DRE_vect) {unexpected_interrupt(89);}  /* Data Register Empty Interrupt */
//ISR (USARTD0_TXC_vect) {unexpected_interrupt(90);}  /* Transmission Complete Interrupt */

/* USARTD1 interrupt vectors */
ISR (USARTD1_RXC_vect) {unexpected_interrupt(91);}  /* Reception Complete Interrupt */
ISR (USARTD1_DRE_vect) {unexpected_interrupt(92);}  /* Data Register Empty Interrupt */
ISR (USARTD1_TXC_vect) {unexpected_interrupt(93);}  /* Transmission Complete Interrupt */

/* PORTF interrupt vectors */
//ISR (PORTF_INT0_vect) {unexpected_interrupt(104);}  /* External Interrupt 0 */
//ISR (PORTF_INT1_vect) {unexpected_interrupt(105);}  /* External Interrupt 1 */

/* TCF0 interrupt vectors */
//ISR (TCF0_OVF_vect) {unexpected_interrupt(108);}  /* Overflow Interrupt */
//ISR (TCF0_ERR_vect) {unexpected_interrupt(109);}  /* Error Interrupt */
//ISR (TCF0_CCA_vect) {unexpected_interrupt(110);}  /* Compare or Capture A Interrupt */
//ISR (TCF0_CCB_vect) {unexpected_interrupt(111);}  /* Compare or Capture B Interrupt */
//ISR (TCF0_CCC_vect) {unexpected_interrupt(112);}  /* Compare or Capture C Interrupt */
//ISR (TCF0_CCD_vect) {unexpected_interrupt(113);}  /* Compare or Capture D Interrupt */

/* USARTF0 interrupt vectors */
ISR (USARTF0_RXC_vect) {unexpected_interrupt(119);}  /* Reception Complete Interrupt */
ISR (USARTF0_DRE_vect) {unexpected_interrupt(120);}  /* Data Register Empty Interrupt */
ISR (USARTF0_TXC_vect) {unexpected_interrupt(121);}  /* Transmission Complete Interrupt */

#endif //ISR_MONITORING