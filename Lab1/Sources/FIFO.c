/*
 * FIFO.c
 *
 *  Created on: 21 Mar 2018
 *      Author: 12403756
 */

// new types
#include "FIFO.h"

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
  if(FIFO->NbBytes < FIFO_SIZE)
  {
    FIFO->Buffer[FIFO->End] = data;
    FIFO->End++;
    FIFO->NbBytes++;
    
    if(FIFO->End == FIFO_SIZE)
    {
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
  if(FIFO->NbBytes > 0)
  {
    *dataPtr = FIFO->Buffer[FIFO->Start];
    FIFO->NbBytes--;
    FIFO->Start++;
    
    if(FIFO->Start == FIFO_SIZE)
    {
      FIFO->Start = 0;
    }
    return true;
  }
  return false;
}