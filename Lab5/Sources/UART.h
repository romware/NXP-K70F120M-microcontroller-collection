/*! @file UART.h
 *
 *  @brief I/O routines for UART communications on the TWR-K70F120M.
 *
 *  This contains the functions for operating the UART (serial port).
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
 */

#ifndef UART_H
#define UART_H

// new types
#include "types.h"
#include "OS.h"
#include "FIFO.h"

/*! @brief Sets up the UART interface before first use.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz.
 *  @return bool - TRUE if the UART was successfully initialized.
 */
bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk);
 
/*! @brief Get a character from the receive FIFO if it is not empty.
 *
 *  @param dataPtr A pointer to memory to store the retrieved byte.
 *  @return void
 *  @note Assumes that UART_Init has been called.
 */
void UART_InChar(uint8_t* const dataPtr);
 
/*! @brief Put a byte in the transmit FIFO if it is not full.
 *
 *  @param data The byte to be placed in the transmit FIFO.
 *  @return void.
 *  @note Assumes that UART_Init has been called.
 */
void UART_OutChar(const uint8_t data);

/*! @brief Interrupt service routine for the UART.
 *
 *  @note Assumes the transmit and receive FIFOs have been initialized.
 */
void __attribute__ ((interrupt)) UART_ISR(void);

#endif
