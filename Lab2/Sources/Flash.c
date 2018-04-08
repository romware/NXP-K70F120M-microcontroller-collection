/*
 * flash.c
 *
 *  Created on: 6 Apr 2018
 *      Author: 12403756
 */

#include "MK70F12.h"
#include "LEDs.h"
#include "Flash.h"


static bool LaunchCommand(TFCCOB* commonCommandObject);
/*static*/ bool WritePhrase(const uint32_t address, const uint64union_t phrase);
static bool EraseSector(const uint32_t address);
static bool ModifyPhrase(const uint32_t address, const uint64union_t phrase);


/*! @brief Enables the Flash module.
 *
 *  @return bool - TRUE if the Flash was setup successfully.
 */
bool Flash_Init(void)
{
  // Check that CCIF is 1 (command completed) and there are no flash access errors or flash protection violation flags set.
  if ((FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK) && !(FTFE_FSTAT & FTFE_FSTAT_ACCERR_MASK) && !(FTFE_FSTAT & FTFE_FSTAT_FPVIOL_MASK))
  {
    return true;
  }
  // If so, erase the flash sector and return false
  else
  {
    // Write 1 to clear ACCERR and FPVIOL
    FTFE_FSTAT |= FTFE_FSTAT_ACCERR_MASK;
    FTFE_FSTAT |= FTFE_FSTAT_FPVIOL_MASK;

    // Clear flash
    Flash_Erase();
    return false;
  }
}

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
bool Flash_AllocateVar(volatile void** variable, const uint8_t size)
{

}

/*! @brief Writes a 32-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 32-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 4-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write32(volatile uint32_t* const address, const uint32_t data)
{

}

/*! @brief Writes a 16-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 16-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 2-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write16(volatile uint16_t* const address, const uint16_t data)
{

}

/*! @brief Writes an 8-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 8-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write8(volatile uint8_t* const address, const uint8_t data)
{

}

/*! @brief Erases the entire Flash sector.
 *
 *  @return bool - TRUE if the Flash "data" sector was erased successfully.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Erase(void)
{
  return EraseSector(FLASH_DATA_START);
}


static bool EraseSector(const uint32_t address)
{
  TFCCOB commonCommandObject;
  commonCommandObject.FCMD = FTFE_FCCOB0_CCOBn(ERASE_FLASH_SECTOR);
  commonCommandObject.flashAddress23to16 = ( (address >> 16) );
  commonCommandObject.flashAddress15to08 = ( (address >> 8) );
  commonCommandObject.flashAddress07to00 = ( address );

  return LaunchCommand(&commonCommandObject);
}

static bool LaunchCommand(TFCCOB* commonCommandObject)
{
  // Write 1 to clear ACCERR and FPVIOL flags
  FTFE_FSTAT |= FTFE_FSTAT_ACCERR_MASK | FTFE_FSTAT_FPVIOL_MASK;

  // Move commonCommandObject to FCCOB
  FTFE_FCCOB0 = commonCommandObject->FCMD;
  FTFE_FCCOB1 = commonCommandObject->flashAddress23to16;
  FTFE_FCCOB2 = commonCommandObject->flashAddress15to08;
  FTFE_FCCOB3 = commonCommandObject->flashAddress07to00;
  FTFE_FCCOB4 = commonCommandObject->dataByte0;
  FTFE_FCCOB5 = commonCommandObject->dataByte1;
  FTFE_FCCOB6 = commonCommandObject->dataByte2;
  FTFE_FCCOB7 = commonCommandObject->dataByte3;
  FTFE_FCCOB8 = commonCommandObject->dataByte4;
  FTFE_FCCOB9 = commonCommandObject->dataByte5;
  FTFE_FCCOBA = commonCommandObject->dataByte6;
  FTFE_FCCOBB = commonCommandObject->dataByte7;

  // Wait for write to finish
  while(!(FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK)){}
  return true;
}

/*static*/ bool WritePhrase(const uint32_t address, const uint64union_t phrase)
{
  TFCCOB cornCob;
  cornCob.FCMD = ( FTFE_FCCOB0_CCOBn(PROGRAM_PHRASE) );
  cornCob.flashAddress23to16 = ( (address >> 16) );
  cornCob.flashAddress15to08 = ( (address >>  8) );
  cornCob.flashAddress07to00 = ( (address >>  0) );

  cornCob.dataByte0 = ( (phrase.l >> 56) );
  cornCob.dataByte1 = ( (phrase.l >> 48) );
  cornCob.dataByte2 = ( (phrase.l >> 40) );
  cornCob.dataByte3 = ( (phrase.l >> 32) );
  cornCob.dataByte4 = ( (phrase.l >> 24) );
  cornCob.dataByte5 = ( (phrase.l >> 16) );
  cornCob.dataByte6 = ( (phrase.l >>  8) );
  cornCob.dataByte7 = ( (phrase.l >>  0) );

  return LaunchCommand(&cornCob);
}

