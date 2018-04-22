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
#define _FB(flashAddress)  *(uint8_t  volatile *)(flashAddress)
#define _FH(flashAddress)  *(uint16_t volatile *)(flashAddress)
#define _FW(flashAddress)  *(uint32_t volatile *)(flashAddress)
#define _FP(flashAddress)  *(uint64_t volatile *)(flashAddress)

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

/*! @brief Writes a 32-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 32-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 4-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write32(volatile uint32_t* const address, const uint32_t data);
 
/*! @brief Writes a 16-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 16-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 2-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write16(volatile uint16_t* const address, const uint16_t data);

/*! @brief Writes an 8-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 8-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write8(volatile uint8_t* const address, const uint8_t data);

/*! @brief Erases the entire Flash sector.
 *
 *  @return bool - TRUE if the Flash "data" sector was erased successfully.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Erase(void);

#endif
