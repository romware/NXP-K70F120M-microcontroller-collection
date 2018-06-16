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

// Thread stacks
OS_THREAD_STACK(RxUARTThreadStack, THREAD_STACK_SIZE);      /*!< The stack for the RxUART thread. */
OS_THREAD_STACK(TxUARTThreadStack, THREAD_STACK_SIZE);      /*!< The stack for the TxUART thread. */

OS_ECB* RxUARTSemaphore;                                    /*!< The Receive FIFO semaphore */
OS_ECB* TxUARTSemaphore;                                    /*!< The Transmit FIFO semaphore */

TFIFO RxFIFO;                                               /*!< The Receive FIFO */
TFIFO TxFIFO;                                               /*!< The Transmit FIFO */

// Dummy/buffer read to allow clearing of RDRF flag
// and calling blocking functions outside ISR.
uint8_t DummyRead;                                          /*!< The dummy read of UART2_D. */

/*! @brief Puts data into the receive FIFO
 *
 *  @note Assumes that UART_Init has been called successfully.
 */
static void RxUARTThread(void* pData)
{
  for (;;)
  {
    // Wait for received data
    OS_SemaphoreWait(RxUARTSemaphore,0);

    // Put the value in UART2 Data Register (UART2_D) in the RxFIFO
    FIFO_Put(&RxFIFO, DummyRead);
  }
}

/*! @brief Puts data into the transmit FIFO
 *
 *  @note Assumes that UART_Init has been called successfully.
 */
static void TxUARTThread(void* pData)
{
  for (;;)
  {
    // Wait until ready to transmit data
    OS_SemaphoreWait(TxUARTSemaphore,0);

    // Read TDRE flag and put data in to clear
    if(UART2_S1 & UART_S1_TDRE_MASK)
    {
      // Put the value in TxFIFO into the UART2 Data Register (UART2_D)
      FIFO_Get(&TxFIFO, (uint8_t*)&UART2_D);

      // Enable transmit interrupts to allow the ISR to signal the semaphore when complete.
      UART2_C2 |= UART_C2_TIE_MASK;
    }
  }
}

/*! @brief Sets up the UART interface before first use.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
 *  @return bool - TRUE if the UART was successfully initialized.
 */
bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  RxUARTSemaphore = OS_SemaphoreCreate(0);
  TxUARTSemaphore = OS_SemaphoreCreate(0);

  OS_ERROR error;

  // 1st Highest priority
  error = OS_ThreadCreate(RxUARTThread,
                          NULL,
                          &RxUARTThreadStack[THREAD_STACK_SIZE - 1],
                          1);
  // 3rd Highest priority
  error = OS_ThreadCreate(TxUARTThread,
                          NULL,
                          &TxUARTThreadStack[THREAD_STACK_SIZE - 1],
                          3);

  // Enable UART2 clock
  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;

  // Enable PORTE clock
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

  // Set PTE16 (BGA Map 'J3') to be the UART2_Tx pin by setting to ALT3
  PORTE_PCR16 = PORT_PCR_MUX(3);

  // Set PTE17 (BGA Map 'K2') to be the UART2_Rx pin by setting to ALT3
  PORTE_PCR17 = PORT_PCR_MUX(3);

  // Confirm UART2_C2 transmit enable is set to 0 to allow setting of the  Baud Rate Divisor Register
  UART2_C2 &= ~UART_C2_TE_MASK;

  // Confirm UART2_C2 receive enable is set to 0 to allow setting of the  Baud Rate Divisor Register
  UART2_C2 &= ~UART_C2_RE_MASK;

  // Declare union to store the baud rate setting, which is a 13 bit divisor.
  uint16union_t locSBR;
  locSBR.l = moduleClk/(16*baudRate);

  // Calculate the BRFA value (%32 remainder)
  uint8_t locBRFA = (2*moduleClk/baudRate)%32;

  // Set the UART2 BDH and BDL
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

  // Address     | Vector | IRQ  | NVIC non-IPR register | NVIC IPR register | Source module | Source description
  // 0x0000_0104 | 65     | 49   | 1                     | 12                | UART2         | Single interrupt vector for UART status sources
  // IRQ modulo 32 = 17

  // Clear any pending interrupts from UART2 (bit 17 of register 1 (IRQ49))
  NVICICPR1 |= (1 << 17);

  // Enable interrupt source for UART2 in NVIC (bit 17 of register 1 (IRQ49))
  NVICISER1 |= (1 << 17);

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
 *  @return void.
 *  @note Assumes that UART_Init has been called.
 */
void UART_InChar(uint8_t * const dataPtr)
{
  // Get one character from the RxFIFO.
  FIFO_Get(&RxFIFO, dataPtr);
}

/*! @brief Put a byte in the transmit FIFO if it is not full.
 *
 *  @param data The byte to be placed in the transmit FIFO.
 *  @return void.
 *  @note Assumes that UART_Init has been called.
 */
void UART_OutChar(const uint8_t data)
{
  // Put one character into the TxFIFO.
  FIFO_Put(&TxFIFO, data);
  
  // Enable transmit interrupts when data is ready to transmit
  UART2_C2 |= UART_C2_TIE_MASK;
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
    // Dummy read UART2_D to clear interrupt flag
    DummyRead = UART2_D;

    // Signal semaphore to allow thread to read another byte
    OS_SemaphoreSignal(RxUARTSemaphore);
  }

  // Check if the UART2 transmit interrupt is enabled and the UART2 transmit data register empty flag is set
  if((UART2_C2 & UART_C2_TIE_MASK) && (UART2_S1 & UART_S1_TDRE_MASK))
  {
    // Clear the transmit interrupt enable
    UART2_C2 &= ~UART_C2_TIE_MASK;
    
    // Signal the semaphore to allow thread to put another byte in Data.
    OS_SemaphoreSignal(TxUARTSemaphore);
  }
  
  // Notify RTOS of exit of ISR
  OS_ISRExit();
}
/* END UART */
/*!
** @}
*/
