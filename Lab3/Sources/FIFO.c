/*! @file FIFO.c
 *
 *  @brief Routines to implement a FIFO buffer.
 *
 *  This contains the structure and "methods" for accessing a byte-wide FIFO.
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
 */
/*!
**  @addtogroup FIFO_module FIFO module documentation
**  @{
*/
/* MODULE FIFO */

// new types
#include "FIFO.h"
#include "PE_Types.h"
#include "Cpu.h"

/*! @brief Initialize the FIFO before first use.
 *
 *  @param FIFO A pointer to the FIFO that needs initializing.
 *  @return void
 */
void FIFO_Init(TFIFO * const FIFO)
{
  FIFO->NbBytes = 0;
  FIFO->Start = 0;
  FIFO->End = 0;
}

/*! @brief Put one character into the FIFO.
 *
 *  @param FIFO A pointer to a FIFO struct where data is to be stored.
 *  @param data A byte of data to store in the FIFO buffer.
 *  @return bool - TRUE if data is successfully stored in the FIFO.
 *  @note Assumes that FIFO_Init has been called.
 */
bool FIFO_Put(TFIFO * const FIFO, const uint8_t data)
{
  // Check if there is room left in the FIFO
  if(FIFO->NbBytes < FIFO_SIZE)
  {
    // Put data byte into the buffer array
    FIFO->Buffer[FIFO->End] = data;

    // Increment the End and NbBytes counters
    FIFO->End++;
    // EnterCritical() to disable global interrupts while modifying NbBytes
    EnterCritical();
    FIFO->NbBytes++;
    // ExitCritical to return interrupt enable state to previous conditions
    ExitCritical();

    // Check if End is past the last element of the array
    if(FIFO->End == FIFO_SIZE)
    {
      // If so, move to element 0
      FIFO->End = 0;
    }
    return true;
  }
  return false;
}

/*! @brief Get one character from the FIFO.
 *
 *  @param FIFO A pointer to a FIFO struct with data to be retrieved.
 *  @param dataPtr A pointer to a memory location to place the retrieved byte.
 *  @return bool - TRUE if data is successfully retrieved from the FIFO.
 *  @note Assumes that FIFO_Init has been called.
 */
bool FIFO_Get(TFIFO * const FIFO, uint8_t * const dataPtr)
{
  // Check that there is data in the FIFO
  if(FIFO->NbBytes > 0)
  {
    // Put the Start data byte into *dataPtr
    *dataPtr = FIFO->Buffer[FIFO->Start];

    // Decrement NbBytes, increment Start
    // EnterCritical() to disable global interrupts while modifying NbBytes
    EnterCritical();
    FIFO->NbBytes--;
    // ExitCritical to return interrupt enable state to previous conditions
    ExitCritical();
    FIFO->Start++;

    // Check if Start is past the last element of the array
    if(FIFO->Start == FIFO_SIZE)
    {
      // If so, move to element 0
      FIFO->Start = 0;
    }
    return true;
  }
  return false;
}

/* END FIFO */
/*!
** @}
*/
