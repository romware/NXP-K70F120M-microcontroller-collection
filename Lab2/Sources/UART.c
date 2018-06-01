/*! @file UART.c
 *
 *  @brief I/O routines for UART communications on the TWR-K70F120M.
 *
 *  This contains the functions for operating the UART (serial port).
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
 *  @modified 2018-04-13
 */

#include "UART.h"
#include "FIFO.h"
#include "MK70F12.h"

// Receive and transmit FIFOs
TFIFO RxFIFO, TxFIFO;

/*! @brief Sets up the UART interface before first use.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
 *  @return bool - TRUE if the UART was successfully initialized.
 */
bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  // Enable UART2 clock: For System Clock Gating Control Register 4 see 12.2.12 of K70P256M150SF3RM.pdf
  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
  // Enable PORTE clock: For System Clock Gating Control Register 5 see 12.2.13 of K70P256M150SF3RM.pdf
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

  // Set PTE16 (BGA Map 'J3') to be the UART2_Tx pin by setting to ALT3
  PORTE_PCR16 = PORT_PCR_MUX(3);
  // Set PTE17 (BGA Map 'K2') to be the UART2_Rx pin by setting to ALT3
  PORTE_PCR17 = PORT_PCR_MUX(3);
  // For K70 Signal Multiplexing and Pin Assignments see 10.3.1 of K70P256M150SF3RM.pdf

  // Confirm UART2_C2 transmit enable is set to 0 to allow setting of the  Baud Rate Divisor Register
  UART2_C2 &= ~UART_C2_TE_MASK;
  // Confirm UART2_C2 receive enable is set to 0 to allow setting of the  Baud Rate Divisor Register
  UART2_C2 &= ~UART_C2_RE_MASK;
  // For UART Control Register 2 see 56.3.4 of K70P256M150SF3RM.pdf

  // Declare union to store the baud rate setting, which is a 13 bit divisor.
  // BDL is stored in SBR.s.Lo and BDH is stored in SBR.s.Hi. Note: The working value in BDH does not change until BDL is written.
  uint16union_t locSBR;
  locSBR.l = moduleClk/(16*baudRate);

  // Declare variable to store the Baud Rate Fine Adjust divisor. This number represents 1/32 remainder from the baud rate division.
  // Calculate the BRFA value (%32 remainder). Only use BDL to avoid overflowing the uint32_t.
  uint8_t locBRFA = (2*moduleClk/baudRate)%32;
  // For SBR and BRFA calculations see 56.4.4 of K70P256M150SF3RM.pdf

  // Set the UART2 BDH and BDL. Note: The working value in BDH does not change until BDL is written. BDH is not 8 bits long so it should be masked
  UART2_BDH = UART_BDH_SBR(locSBR.s.Hi);
  UART2_BDL = UART_BDL_SBR(locSBR.s.Lo);
  UART2_C4 = UART_C4_BRFA(locBRFA);

  // Set UART2_C2 transmit enable to 1
  UART2_C2 |= UART_C2_TE_MASK;
  // Set UART2_C2 receive enable to 1
  UART2_C2 |= UART_C2_RE_MASK;
  // For UART Control Register 2 see 56.3.4 of K70P256M150SF3RM.pdf

  // Initiate the RxFIFO and TxFIFO
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
  // Get one character from the RxFIFO.
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
  // Put one character into the TxFIFO.
  return FIFO_Put(&TxFIFO, data);
}

/*! @brief Poll the UART status register to try and receive and/or transmit one character.
 *
 *  @return void
 *  @note Assumes that UART_Init has been called.
 */
void UART_Poll(void)
{
  /*!  "The incoming serial data will set the Receive Data Register Full (RDRF) flag
   *   in the UART Status Register 1 (UART_S1) register, indicating that the receiver
   *   hardware has just received a byte of data. In the main loop, a poll of the RDRF
   *   flag is performed. If it is set, then the program tries to accept the data and
   *   put it in the RxFIFO. The RxFIFO buffers data between the input hardware and
   *   the main program that processes the data." - Assignment Specification
   */

  // AND the UART2 Status register with the receive mask
  if(UART2_S1 & UART_S1_RDRF_MASK)
  {
    // Put the value in UART2 Data Register (UART2_D) in the RxFIFO
    FIFO_Put(&RxFIFO, UART2_D);
  }

  /*!  "The setting of the Transmit Data Register Empty (TDRE) flag by the UART
   *   hardware signals that the output shift register is idle and ready to output
   *   more data. In the main loop, a poll of the TDRE flag is performed. If it is
   *   set, then the program tries to retrieve the data in the TxFIFO, and send it
   *   out the serial port. If the TxFIFO becomes empty, then no data will be sent
   *   out the serial port." - Assignment Specification
   */

  // AND the UART2 Status register with the transmit mask
  if(UART2_S1 & UART_S1_TDRE_MASK)
  {
    // Put the value in TxFIFO into the UART2 Data Register (UART2_D)
    FIFO_Get(&TxFIFO, (uint8_t*)&UART2_D);
  }
}
