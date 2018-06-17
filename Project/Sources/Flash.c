/*! @file Flash.c
 *
 *  @brief Routines for erasing and writing to the Flash.
 *
 *  This contains the functions needed for accessing the internal Flash.
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
 */
/*!
**  @addtogroup Flash_module Flash module documentation
**  @{
*/
/* MODULE Flash */

#include "MK70F12.h"
#include "LEDs.h"
#include "Flash.h"
#include "OS.h"

OS_ECB* CCIFSemaphore;                /*!< The CCIF semaphore for Flash */
OS_ECB* FlashAccessMutex;             /*!< The Flash access Mutex */

/*! @brief Writes TFCCOB to flash and waits for it to complete
 *
 *  @param commonCommandObject The address of the TFCCOB.
 *  @return bool - TRUE if TFCCOB was written to Flash successfully
 *  @note Assumes Flash has been initialized.
 */
static bool LaunchCommand(TFCCOB* commonCommandObject);

/*! @brief Encodes an address and phrase into the TFCCOB structure
 *
 *  @param address The address of the data.
 *  @param phrase The 64-bit phrase to write.
 *  @return bool - TRUE if TFCCOB was encoded successfully
 *  @note Assumes Flash has been initialized.
 */
static bool WritePhrase(const uint32_t address, const uint64union_t phrase);

/*! @brief Erases a sector of Flash memory
 *
 *  @param address The address of the Flash sector.
 *  @return bool - TRUE if the sector was erased successfully
 *  @note Assumes Flash has been initialized.
 */
static bool EraseSector(const uint32_t address);


/*! @brief Writes a 32-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 32-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 4-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
static bool Write32(volatile uint32_t* const address, const uint32_t data)
{
  // The phrase to parse
  uint64union_t phrase; 
  
  // The 32 bit cast of the address
  uint32_t address32 = (uint32_t)address;

  // Check if address is evenly divisible by 4 to find which bytes to write it to
  if (address32 % (2*sizeof(data)) == 0)
  {
    // The word takes the low end of the phrase
    phrase.s.Lo = data;
    
    // The value of the next 4 bytes takes the high end of the phrase
    phrase.s.Hi =  *(uint32_t volatile *)(address32 + sizeof(data));
    
    // Write the phrase to the second half of the 8 byte Flash memory
    return WritePhrase( (address32), phrase );
  }
  else
  {
    // The value of the previous 4 bytes takes the low end of the phrase
    phrase.s.Lo =  *(uint32_t volatile *)(address32 - sizeof(data));
    
    // The word takes the high end of the phrase
    phrase.s.Hi = data; 
    
    // Write the phrase to the first half of the 8 byte Flash memory
    return WritePhrase((address32 - sizeof(data)), phrase );
  }
}

/*! @brief Writes a 16-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 16-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 2-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
static bool Write16(volatile uint16_t* const address, const uint16_t data)
{
  // The word to parse
  uint32union_t word; 
  
  // The 32 bit cast of the address
  uint32_t address32 = (uint32_t)address;

  // Check if address is divisible by 4 to find which bytes to write it to
  if(address32 % (2*sizeof(data)) == 0)
  {
    // The half word takes the low end of the word
    word.s.Lo = data; 
    
    // The value of the next 2 bytes takes the high end of the word
    word.s.Hi =  *(uint16_t volatile *)(address32 + sizeof(data));
    
    // Write the word to the second quarter or fourth quarter of the 8 byte Flash memory
    return Write32( (uint32_t volatile *)(address32), word.l );
  }
  else
  {
    // The value of the previous 2 bytes takes the low end of the word
    word.s.Lo =  *(uint16_t volatile *)(address32 - sizeof(data));
    
    // The half word takes the high end of the word
    word.s.Hi = data;
    
    // Write the word to the first quarter or thirst quarter of the 8 byte Flash memory
    return Write32( (uint32_t volatile *)(address32 - sizeof(data)), word.l );
  }
}

/*! @brief Writes an 8-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 8-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
static bool Write8(volatile uint8_t* const address, const uint8_t data)
{
  // The half word to parse
  uint16union_t halfWord;
  
  // The 32 bit cast of the address
  uint32_t address32 = (uint32_t)address;
  
  // Check if address is divisible by 2 to find which bytes to write it to
  if(address32 % (2*sizeof(data)) == 0)
  {
    // The byte takes the low end of the half word
    halfWord.s.Lo = data; 
    
    // The value of the next byte takes the high end of the half word
    halfWord.s.Hi =  *(uint8_t volatile *)(address32 + sizeof(data));
    
    // Write the half word to the second, fourth, sixth or eighth section of the 8 byte Flash memory
    return Write16( (uint16_t volatile *)(address32), halfWord.l );
  }
  else
  {
    // The value of the previous byte takes the low end of the half word
    halfWord.s.Lo =  *(uint8_t volatile *)(address32 - sizeof(data));
    
    // The byte takes the high end of the half word
    halfWord.s.Hi = data; 
    
    // Write the half word to the first, third, fifth or seventh section of the 8 byte Flash memory
    return Write16( (uint16_t volatile *)(address32 - sizeof(data)), halfWord.l );
  }
}

/*! @brief Writes TFCCOB to flash and waits for it to complete
 *
 *  @param commonCommandObject The address of the TFCCOB.
 *  @return bool - TRUE if TFCCOB was written to Flash successfully
 *  @note Assumes Flash has been initialized.
 */
static bool LaunchCommand(TFCCOB* commonCommandObject)
{
  // Write 1 to clear ACCERR and FPVIOL flags
  FTFE_FSTAT = FTFE_FSTAT_ACCERR_MASK | FTFE_FSTAT_FPVIOL_MASK;

  // Move commonCommandObject to FCCOB
  FTFE_FCCOB0 = FTFE_FCCOB0_CCOBn(commonCommandObject->FCMD);
  FTFE_FCCOB1 = FTFE_FCCOB1_CCOBn(commonCommandObject->flashAddress23to16);
  FTFE_FCCOB2 = FTFE_FCCOB2_CCOBn(commonCommandObject->flashAddress15to08);
  FTFE_FCCOB3 = FTFE_FCCOB3_CCOBn(commonCommandObject->flashAddress07to00);

  FTFE_FCCOB8 = FTFE_FCCOB8_CCOBn(commonCommandObject->dataByte0);
  FTFE_FCCOB9 = FTFE_FCCOB9_CCOBn(commonCommandObject->dataByte1);
  FTFE_FCCOBA = FTFE_FCCOBA_CCOBn(commonCommandObject->dataByte2);
  FTFE_FCCOBB = FTFE_FCCOBB_CCOBn(commonCommandObject->dataByte3);

  FTFE_FCCOB4 = FTFE_FCCOB4_CCOBn(commonCommandObject->dataByte4);
  FTFE_FCCOB5 = FTFE_FCCOB5_CCOBn(commonCommandObject->dataByte5);
  FTFE_FCCOB6 = FTFE_FCCOB6_CCOBn(commonCommandObject->dataByte6);
  FTFE_FCCOB7 = FTFE_FCCOB7_CCOBn(commonCommandObject->dataByte7);

  //Write 1 to clear CCIF
  FTFE_FSTAT = FTFE_FSTAT_CCIF_MASK;

  // Enable interrupts from CCIF
  FTFE_FCNFG |= FTFE_FCNFG_CCIE_MASK;

  // Wait for write to finish
  //while(!(FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK)){}
  
  // Wait for semaphore to continue
  OS_SemaphoreWait(CCIFSemaphore, 0);

  return true;
}
/*! @brief Enables the Flash module.
 *
 *  @return bool - TRUE if the Flash was setup successfully.
 */
bool Flash_Init(void)
{
  // Initialize the Flash
  FlashAccessMutex = OS_SemaphoreCreate(1);
  CCIFSemaphore = OS_SemaphoreCreate(0);

  // Address     | Vector | IRQ  | NVIC non-IPR register | NVIC IPR register | Source module | Source description
  // 0x0000_0104 | 34     | 18   | 0                     | 4                 | Flash memory  | Command complete
  // IRQ modulo 32 = 18

  // Clear any pending interrupts from CCIF (bit 18 of register 0 (IRQ18))
  NVICICPR0 |= (1 << 18);

  // Enable interrupt source for CCIF in NVIC (bit 18 of register 0 (IRQ18))
  NVICISER0 |= (1 << 18);

  return true;
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
  // Flash allocation bytes
  static uint8_t occupiedBytes = 0b00000000;

  // Check if size of data to be allocated an address is a byte, half word or word
  if(size == 1 || size == 2 || size == 4)
  {
    // Initialize the allocation to be the first address which can store the data
    uint8_t allocation = size * 2 - 1;

    // Loop through each of the 8 bytes in Flash until an address is available
    for(uint8_t i = 0; i < 8/size; i++)
    {
      // Mask the allocation with the currently occupied bit mask to see if it is available
      if(!(occupiedBytes & allocation))
      {
        // Set the variable value to be the Flash address
        *variable = (uint16_t* volatile)((uint32_t)FLASH_DATA_START + i * size);

        // Set the occupation bit mask to have the newly allocated address as occupied
        occupiedBytes |= allocation;
        return true;
      }

      // If the address is occupied, bit shift to the next location and check again
      allocation = allocation << size;
    }
  }
  return false;
}

/*! @brief Erases the entire Flash sector.
 *
 *  @return bool - TRUE if the Flash "data" sector was erased successfully.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Erase(void)
{
  // Erase the Flash sector with the Flash Data Start address
  return EraseSector(FLASH_DATA_START);
}


/*! @brief Encodes an address and phrase into the TFCCOB struct
 *
 *  @param address The address of the data.
 *  @param phrase The 64-bit phrase to write.
 *  @return bool - TRUE if TFCCOB was encoded successfully
 *  @note Assumes Flash has been initialized.
 */
static bool WritePhrase(const uint32_t address, const uint64union_t phrase)
{
  // Erase Flash before writing
  if(Flash_Erase())
  {
    // Initialize a local TFCCOB structure
    TFCCOB commonCommandObject;
  
    // Set the FTFE command to program a phrase
    commonCommandObject.FCMD = PROGRAM_PHRASE;
  
    // Set the FTFE flash address
    commonCommandObject.flashAddress23to16 = ( (address >> 16) );
    commonCommandObject.flashAddress15to08 = ( (address >>  8) );
    commonCommandObject.flashAddress07to00 = ( (address >>  0) );
  
    // Set the FTFE data bytes to be the phrase in big endian
    commonCommandObject.dataByte0 = ( (phrase.l >> 56) );
    commonCommandObject.dataByte1 = ( (phrase.l >> 48) );
    commonCommandObject.dataByte2 = ( (phrase.l >> 40) );
    commonCommandObject.dataByte3 = ( (phrase.l >> 32) );

    commonCommandObject.dataByte4 = ( (phrase.l >> 24) );
    commonCommandObject.dataByte5 = ( (phrase.l >> 16) );
    commonCommandObject.dataByte6 = ( (phrase.l >>  8) );
    commonCommandObject.dataByte7 = ( (phrase.l >>  0) );

    // Run the command to program the phrase
    return LaunchCommand(&commonCommandObject);
  }
  return false;
}

/*! @brief Erases a sector of Flash memory
 *
 *  @param address The address of the Flash sector.
 *  @return bool - TRUE if the sector was erased successfully
 *  @note Assumes Flash has been initialized.
 */
static bool EraseSector(const uint32_t address)
{
  // Initialize a local TFCCOB structure
  TFCCOB commonCommandObject;
  
  // Set the FTFE command to erase the flash sector
  commonCommandObject.FCMD = ERASE_FLASH_SECTOR;
  
  // Set the FTFE flash address
  commonCommandObject.flashAddress23to16 = ( (address >> 16) );
  commonCommandObject.flashAddress15to08 = ( (address >> 8) );
  commonCommandObject.flashAddress07to00 = ( address );
  
  // Run the command to erase the flash sector
  return LaunchCommand(&commonCommandObject);
}


bool Flash_Write(volatile void * const address, const uint32_t data, uint8_t dataSize)
{
  bool success = false;
  // Gain access to flash
  OS_SemaphoreWait(FlashAccessMutex, 0);

  switch(dataSize){
    case 8:
      success = Write8(address, data);
      break;
    case 16:
      success = Write16(address, data);
      break;
    case 32:
      success = Write32(address, data);
      break;
  }
  // Release access
  OS_SemaphoreSignal(FlashAccessMutex);

  return success;
}

uint8_t _FB(volatile uint8_t* flashAddress)
{
  // Gain access
  OS_SemaphoreWait(FlashAccessMutex, 0);

  uint8_t byte = *(uint8_t  volatile *)(flashAddress);

  // Release access
  OS_SemaphoreSignal(FlashAccessMutex);

  return byte;
}

uint16_t _FH(volatile uint16_t* flashAddress)
{
  // Gain access
  OS_SemaphoreWait(FlashAccessMutex, 0);

  uint16_t halfWord = *(uint16_t volatile *)(flashAddress);

  // Release access
  OS_SemaphoreSignal(FlashAccessMutex);

  return halfWord;

}

uint32_t _FW(volatile uint32_t* flashAddress)
{
  // Gain access
  OS_SemaphoreWait(FlashAccessMutex, 0);

  uint32_t word = *(uint32_t volatile *)(flashAddress);

  // Release access
  OS_SemaphoreSignal(FlashAccessMutex);

  return word;
}

uint64_t _FP(volatile uint64_t* flashAddress)
{
  // Gain access
  OS_SemaphoreWait(FlashAccessMutex, 0);

  uint64_t phrase = *(uint64_t volatile *)(flashAddress);

  // Release access
  OS_SemaphoreSignal(FlashAccessMutex);

  return phrase;
}

void __attribute__ ((interrupt)) Flash_ISR(void)
{
  OS_ISREnter();

  // Disable interrupts from CCIF
  FTFE_FCNFG &= ~FTFE_FCNFG_CCIE_MASK;

  // Signal Semaphore
  OS_SemaphoreSignal(CCIFSemaphore);

  OS_ISRExit();
}

/* END Flash */
/*!
** @}
*/
