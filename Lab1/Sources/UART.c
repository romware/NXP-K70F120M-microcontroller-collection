/*
 * UART.c
 *
 *  Created on: 22 Mar 2018
 *      Author: 12403756
 */
#include "UART.h"

/*! @brief Sets up the UART interface before first use.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
 *  @return bool - TRUE if the UART was successfully initialized.
 */
bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  // enable UART2 by turning the clock on
  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

  // set PTE16 to be the UART2_Tx pin
  PORTE_PCR16 = PORT_PCR_MUX(3);
  // set PTE17 to be the UART2_Rx pin
  PORTE_PCR17 = PORT_PCR_MUX(3);
  // set UART2_C2 transmit enable to 1;
  UART2_C2 &= ~UART_C2_TE_MASK;
  // set UART2_C2 receive enable to 1;
  UART2_C2 &= ~UART_C2_RE_MASK;
  uint16_t BRFA = 0;
  uint16union_t SBR;

  BRFA = ((moduleClk/(16*baudRate))*32)%32;			//calculate BRFA from %32

  SBR.l = ((moduleClk/(16*baudRate))*32)/32 - BRFA;	//calculate the module clock divisor
  UART2_BDL = SBR.s.Lo;
  UART2_BDH = SBR.s.Hi & UART_BDH_SBR_MASK;

  // set UART2_C2 transmit enable to 1;
    UART2_C2 |= UART_C2_TE_MASK;
    // set UART2_C2 receive enable to 1;
    UART2_C2 |= UART_C2_RE_MASK;

  FIFO_Init(&RxFIFO);
  FIFO_Init(&TxFIFO);
  
  return true;
}


/*! @brief Get a character from the receive FIFO if it is not empty.
 *
 *  @param dataPtr A pointer to memory to store the retrieved byte.
 *  @return bool - TRUE if the receive FIFO returned a character.
 *  @note Assumes that UART_Init has been called.
 */
bool UART_InChar(uint8_t * const dataPtr)
{
  return FIFO_Get(&RxFIFO, dataPtr);
}


/*! @brief Put a byte in the transmit FIFO if it is not full.
 *
 *  @param data The byte to be placed in the transmit FIFO.
 *  @return bool - TRUE if the data was placed in the transmit FIFO.
 *  @note Assumes that UART_Init has been called.
 */
bool UART_OutChar(const uint8_t data)
{
  return FIFO_Put(&TxFIFO, data);
}


/*! @brief Poll the UART status register to try and receive and/or transmit one character.
 *
 *  @return void
 *  @note Assumes that UART_Init has been called.
 */
void UART_Poll(void)
{
  /* 
  The incoming serial data will set the Receive Data Register Full (RDRF) flag in
  the UART Status Register 1 (UART_S1) register, indicating that the receiver
  hardware has just received a byte of data. In the main loop, a poll of the RDRF
  flag is performed. If it is set, then the program tries to accept the data and put it
  in the RxFIFO. The RxFIFO buffers data between the input hardware and the
  main program that processes the data.
  */
  if(UART2_S1 & UART_S1_RDRF_MASK) //And the UART2 Status register with the receive mask (Binary AND Operator copies a bit to the result if it exists in both operands.)
  {
    /*
    When the packet module wishes to output, it calls UART_OutChar, which
    will put the data in the TxFIFO and arm the output device
    */
    FIFO_Put(&RxFIFO, UART2_D); //UART_OutChar(UART2_D); //4006_C007 UART Data Register (UART2_D)
  }
    
  /*
  The setting of the Transmit Data Register Empty (TDRE) flag by the UART
  hardware signals that the output shift register is idle and ready to output more
  data. In the main loop, a poll of the TDRE flag is performed. If it is set, then the
  program tries to retrieve the data in the TxFIFO., and send it out the serial
  port. If the TxFIFO becomes empty, then no data will be sent out the serial
  port.
  */
  if(UART2_S1 & UART_S1_TDRE_MASK) //And the UART2 Status register with the transmit mask (Binary AND Operator copies a bit to the result if it exists in both operands.)
  {
    /*
    When the packet module wishes to receive input, it calls UART_InChar,
    which will attempt to get data from the RxFIFO (it will fail if the FIFO buffer
    is empty). How does data get in the RxFIFO?
    */

      FIFO_Get(&TxFIFO, (uint8_t*)&UART2_D); //UART_InChar(&UART2_D); //4006_C007 UART Data Register (UART2_D)
  }
}
