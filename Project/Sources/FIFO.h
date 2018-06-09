/*! @file FIFO.h
 *
 *  @brief Routines to implement a FIFO buffer.
 *
 *  This contains the structure and "methods" for accessing a byte-wide FIFO.
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
 */

#ifndef FIFO_H
#define FIFO_H

// new types
#include "types.h"
#include "OS.h"

// Number of bytes in a FIFO
#define FIFO_SIZE 512

/*!
 * @struct TFIFO
 */
typedef struct
{
  uint16_t Start;               /*!< The index of the position of the oldest data in the FIFO */
  uint16_t End;                 /*!< The index of the next available empty position in the FIFO */
  uint8_t Buffer[FIFO_SIZE];    /*!< The actual array of bytes to store the data */
  OS_ECB* CanPut;               /*!< The semaphore which allows putting into the FIFO */
  OS_ECB* CanGet;               /*!< The semaphore which allows getting from the FIFO */
} TFIFO;

/*! @brief Initialize the FIFO before first use.
 *
 *  @param FIFO A pointer to the FIFO that needs initializing.
 *  @return void
 */
void FIFO_Init(TFIFO* const FIFO);

/*! @brief Put one character into the FIFO.
 *
 *  @param FIFO A pointer to a FIFO struct where data is to be stored.
 *  @param data A byte of data to store in the FIFO buffer.
 *  @return void.
 *  @note Assumes that FIFO_Init has been called.
 */
void FIFO_Put(TFIFO* const FIFO, const uint8_t data);

/*! @brief Get one character from the FIFO.
 *
 *  @param FIFO A pointer to a FIFO struct with data to be retrieved.
 *  @param dataPtr A pointer to a memory location to place the retrieved byte.
 *  @return void.
 *  @note Assumes that FIFO_Init has been called.
 */
void FIFO_Get(TFIFO* const FIFO, uint8_t* const dataPtr);

#endif
