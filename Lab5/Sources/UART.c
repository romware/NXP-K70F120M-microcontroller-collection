/*! @file UART.c
 *
 *  @brief I/O routines for UART communications on the TWR-K70F120M.
 *
 *  This contains the functions for operating the UART (serial port).
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
 */
/*!
**  @addtogroup UART_module UART module documentation
**  @{
*/
/* MODULE UART */

#include "UART.h"
#include "FIFO.h"
#include "MK70F12.h"
#include "PE_Types.h"
#include "Cpu.h"
#include "OS.h"


/*! @brief Sets up the UART interface before first use.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
 *  @return bool - TRUE if the UART was successfully initialized.
 */
bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  RxUART = OS_SemaphoreCreate(0);
  TxUART = OS_SemaphoreCreate(0);

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

  // Ensure global interrupts are disabled
  EnterCritical();

  // Set UART2_C2 transmit interrupt enable to 0
  UART2_C2 &= ~UART_C2_TIE_MASK;

  // Set UART2_C2 receive interrupt enable to 1
  UART2_C2 |= UART_C2_RIE_MASK;
  // For UART Control Register 2 see 56.3.4 of K70P256M150SF3RM.pdf

  // Address     | Vector | IRQ  | NVIC non-IPR register | NVIC IPR register | Source module | Source description
  // 0x0000_0104 | 65     | 49   | 1                     | 12                | UART2         | Single interrupt vector for UART status sources
  // IRQ modulo 32 = 17

  // Clear any pending interrupts from UART2 (bit 17 of register 1 (IRQ49))
  NVICICPR1 |= (1 << 17);

  // Enable interrupt source for UART2 in NVIC (bit 17 of register 1 (IRQ49))
  NVICISER1 |= (1 << 17);
  // For NVIC configuration see 3.2.2 of K70P256M150SF3RM.pdf

  // Return global interrupts to how they were
  ExitCritical();

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
  FIFO_Get(&RxFIFO, dataPtr);
  return true;
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
  FIFO_Put(&TxFIFO, data);
  //UART2_C2 |= UART_C2_TIE_MASK;
  return true;
}

/*! @brief Interrupt service routine for the UART.
 *
 *  @note Assumes the transmit and receive FIFOs have been initialized.
 */
void __attribute__ ((interrupt)) UART_ISR(void)
{
  // Notify RTOS of start of ISR
  OS_ISREnter();

  // Check if UART2 receive interrupt is enabled and the UART2 receive data register full flag is set
  if((UART2_C2 & UART_C2_RIE_MASK) && (UART2_S1 & UART_S1_RDRF_MASK))
  {
    //TODO: dummy read to clear interrupt flag
    DummyRead = UART2_D;

    // Put the value in UART2 Data Register (UART2_D) in the RxFIFO
    OS_SemaphoreSignal(RxUART);
  }

  // Check if the UART2 transmit interrupt is enabled and the UART2 transmit data register empty flag is set
  if((UART2_C2 & UART_C2_TIE_MASK) && (UART2_S1 & UART_S1_TDRE_MASK))
  {
    // If the FIFO is empty clear the transmit interrupt enable
    //if(!TxFIFO.CanGet)
    //{
    //}

    OS_SemaphoreSignal(TxUART);
    UART2_C2 &= ~UART_C2_TIE_MASK;
  }
  OS_ISRExit();
}
/* END UART */
/*!
** @}
*/
