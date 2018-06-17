/*! @file Flash.h
 *
 *  @brief Routines for erasing and writing to the Flash.
 *
 *  This contains the functions needed for accessing the internal Flash.
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
 */

#ifndef FLASH_H
#define FLASH_H

// FCMD commands
static const uint8_t PROGRAM_PHRASE = 0x07;
static const uint8_t ERASE_FLASH_SECTOR = 0x09;

// new types
#include "types.h"

// TFCCOB struct of containing the Flash command, address and 8 data bytes
typedef struct
{
  uint8_t FCMD;                /*!< The TFCCOB command byte */
  uint8_t flashAddress23to16;  /*!< The TFCCOB address higher byte */
  uint8_t flashAddress15to08;  /*!< The TFCCOB address middle byte */
  uint8_t flashAddress07to00;  /*!< The TFCCOB address lower byte */
  uint8_t dataByte0;           /*!< The TFCCOB data byte 0 */
  uint8_t dataByte1;           /*!< The TFCCOB data byte 1 */
  uint8_t dataByte2;           /*!< The TFCCOB data byte 2 */
  uint8_t dataByte3;           /*!< The TFCCOB data byte 3 */
  uint8_t dataByte4;           /*!< The TFCCOB data byte 4 */
  uint8_t dataByte5;           /*!< The TFCCOB data byte 5 */
  uint8_t dataByte6;           /*!< The TFCCOB data byte 6 */
  uint8_t dataByte7;           /*!< The TFCCOB data byte 7 */
} TFCCOB;

// FLASH data access
//#define _FB(flashAddress)  *(uint8_t  volatile *)(flashAddress)
//#define _FH(flashAddress)  *(uint16_t volatile *)(flashAddress)
//#define _FW(flashAddress)  *(uint32_t volatile *)(flashAddress)
//#define _FP(flashAddress)  *(uint64_t volatile *)(flashAddress)

// Address of the start of the Flash block we are using for data storage
#define FLASH_DATA_START 0x00080000LU
// Address of the end of the Flash block we are using for data storage
#define FLASH_DATA_END   0x00080007LU

/*! @brief Enables the Flash module.
 *
 *  @return bool - TRUE if the Flash was setup successfully.
 */
bool Flash_Init(void);
 
/*! @brief Allocates space for a non-volatile variable in the Flash memory.
 *
 *  @param variable is the address of a pointer to a variable that is to be allocated space in Flash memory.
 *         The pointer will be allocated to a relevant address:
 *         If the variable is a byte, then any address.
 *         If the variable is a half-word, then an even address.
 *         If the variable is a word, then an address divisible by 4.
 *         This allows the resulting variable to be used with the relevant Flash_Write function which assumes a certain memory address.
 *         e.g. a 16-bit variable will be on an even address
 *  @param size The size, in bytes, of the variable that is to be allocated space in the Flash memory. Valid values are 1, 2 and 4.
 *  @return bool - TRUE if the variable was allocated space in the Flash memory.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_AllocateVar(volatile void** variable, const uint8_t size);


/*! @brief Erases the entire Flash sector.
 *
 *  @return bool - TRUE if the Flash "data" sector was erased successfully.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Erase(void);


/*! @brief Writes to flash with mutex access for use in an RTOS.
 *
 *  @param address The address of the data.
 *  @param data The data to be written. Cast as 32 bit but will write 16 and 8.
 *  @param dataSize The size in bits of the data to be written.
 *  @return bool - TRUE if the Flash "data"  was written successfully.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write(volatile void * const address, const uint32_t data, uint8_t dataSize);


/*! @brief Read one byte from flash with mutex access for use in RTOS.
 *
 *  @param flashAddress The pointer to the data to read.
 *  @return uint8_t - The data at the given address
 *  @note Assumes Flash has been initialized.
 */
uint8_t _FB(volatile uint8_t* flashAddress);


/*! @brief Read one half word from flash with mutex access for use in RTOS.
 *
 *  @param flashAddress The pointer to the data to read.
 *  @return uint16_t - The data at the given address
 *  @note Assumes Flash has been initialized.
 */
uint16_t _FH(volatile uint16_t* flashAddress);


/*! @brief Read one word from flash with mutex access for use in RTOS.
 *
 *  @param flashAddress The pointer to the data to read.
 *  @return uint32_t - The data at the given address
 *  @note Assumes Flash has been initialized.
 */
uint32_t _FW(volatile uint32_t* flashAddress);


/*! @brief Read one phrase from flash with mutex access for use in RTOS.
 *
 *  @param flashAddress The pointer to the data to read.
 *  @return uint64_t - The data at the given address
 *  @note Assumes Flash has been initialized.
 */
uint64_t _FP(volatile uint64_t* flashAddress);


/*! @brief Interrupt service routine for the Flash.
 *
 *  @note Assumes flash has been initialized and a RTOS is being used.
 */
void __attribute__ ((interrupt)) Flash_ISR(void);
#endif
