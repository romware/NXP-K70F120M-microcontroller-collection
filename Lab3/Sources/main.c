/* ###################################################################
**     Filename    : main.c
**     Project     : Lab2
**     Processor   : MK70FN1M0VMJ12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-04-12, 13:27, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**     Authors     : 12403756, 12551519
**
** ###################################################################*/
/*!
** @file main.c
** @version 2.0
** @brief
**         Main module.
**         This module contains user's application code.
** @author 12403756, 12551519
** @date 2018-04-13
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */

// Included header files
#include "types.h"
#include "Cpu.h"
#include "Events.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "LEDs.h"
#include "MK70F12.h"
#include "UART.h"
#include "FIFO.h"
#include "packet.h"
#include "Flash.h"
#include "PE_Types.h"
#include "PIT.h"
#include "FTM.h"
#include "RTC.h"

#define BAUD_RATE 115200                     /*!< UART2 Baud Rate */

const uint32_t PERIOD_LED_GREEN = 500000000; /*!< Period of Periodic Interrupt Timer 0 in nanoseconds */

const uint8_t COMMAND_STARTUP     = 0x04;    /*!< The serial command byte for tower startup */
const uint8_t COMMAND_VER         = 0x09;    /*!< The serial command byte for tower version */
const uint8_t COMMAND_NUM         = 0x0B;    /*!< The serial command byte for tower number */
const uint8_t COMMAND_PROGRAMBYTE = 0x07;    /*!< The serial command byte for tower program byte */
const uint8_t COMMAND_READBYTE    = 0x08;    /*!< The serial command byte for tower read byte */
const uint8_t COMMAND_MODE        = 0x0D;    /*!< The serial command byte for tower mode */
const uint8_t COMMAND_TIME        = 0x0C;    /*!< The serial command byte for tower time */

const uint8_t PARAM_GET = 1;                 /*!< Get bit of packet parameter 1 */
const uint8_t PARAM_SET = 2;                 /*!< Set bit of packet parameter 1 */

const uint8_t TOWER_VER_MAJ = 1;             /*!< Tower major version */
const uint8_t TOWER_VER_MIN = 0;             /*!< Tower minor version */

volatile uint16union_t* NvTowerNb;           /*!< Tower number union pointer to flash */
volatile uint16union_t* NvTowerMd;           /*!< Tower mode union pointer to flash */


/*! @brief Sends the startup packets to the PC
 *
 *  @return bool - TRUE if all packets are sent
 */
bool HandleTowerStartup(void)
{
  // Sends the tower startup values, version, number and mode to the PC
  return (
    Packet_Put(COMMAND_STARTUP,0x00,0x00,0x00) &&
    Packet_Put(COMMAND_VER,'v',TOWER_VER_MAJ,TOWER_VER_MIN) &&
    Packet_Put(COMMAND_NUM,PARAM_GET,NvTowerNb->s.Lo,NvTowerNb->s.Hi) &&
    Packet_Put(COMMAND_MODE,PARAM_GET,NvTowerMd->s.Lo,NvTowerMd->s.Hi)
  );
}

/*! @brief Sends the tower version packet to the PC
 *
 *  @return bool - TRUE if packet is sent
 */
bool HandleTowerVersion(void)
{
  // Sends the tower number packet
  return Packet_Put(COMMAND_NUM,PARAM_GET,NvTowerNb->s.Lo,NvTowerNb->s.Hi);
}

/*! @brief Sets the tower number or sends the packet to the PC
 *
 *  @return bool - TRUE if packet is sent or number is set
 */
bool HandleTowerNumber(void)
{
  // Check if parameters match tower number GET or SET parameters
  if(Packet_Parameter1 == PARAM_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the tower number packet
    return Packet_Put(COMMAND_NUM,PARAM_GET,NvTowerNb->s.Lo,NvTowerNb->s.Hi);
  }
  else if (Packet_Parameter1 == PARAM_SET)
  {
    // Sets the tower number
    return Flash_Write16((uint16_t*)NvTowerNb,(uint16_t)Packet_Parameter23);
  }
  return false;
}

/*! @brief Erases or programs a byte to Flash based on packet received
 *
 *  @return bool - TRUE if Flash was modified
 */
bool HandleTowerProgramByte(void)
{
  // Check if offset is erase or set
  if(Packet_Parameter1 == 0x08 && Packet_Parameter2 == 0)
  {
    // Erase Flash
    return Flash_Erase();
  }
  else if(Packet_Parameter1 >= 0x00 && Packet_Parameter1 <= 0x07 && Packet_Parameter2 == 0)
  {
    // Program byte to Flash
    volatile uint8_t* nvAddress = (uint8_t*)(FLASH_DATA_START + Packet_Parameter1);
    return Flash_Write8( (uint8_t*)nvAddress, Packet_Parameter3 );
  }
  return false;
}

/*! @brief Sends the byte read from Flash packet to the PC
 *
 *  @return bool - TRUE if packet is sent
 */
bool HandleTowerReadByte(void)
{
  // Check if offset is within range
  if(Packet_Parameter1 >= 0x00 && Packet_Parameter1 <= 0x07 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Send read byte packet
    return Packet_Put(COMMAND_READBYTE,Packet_Parameter1,0,_FB(FLASH_DATA_START + (uint32_t)Packet_Parameter1));
  }
  return false;
}

/*! @brief Sets the tower mode or sends the packet to the PC
 *
 *  @return bool - TRUE if packet is sent or mode is set
 */
bool HandleTowerMode(void)
{
  // Check if parameters match tower mode GET or SET parameters
  if(Packet_Parameter1 == PARAM_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the tower mode packet
    return Packet_Put(COMMAND_MODE,PARAM_GET,NvTowerMd->s.Lo,NvTowerMd->s.Hi);
  }
  else if (Packet_Parameter1 == PARAM_SET)
  {
    // Sets the tower mode
    return Flash_Write16((uint16_t*)NvTowerMd,(uint16_t)Packet_Parameter23);
  }
  return false;
}

/*! @brief Sets the Real Time Clock time as requested by PC
 *
 *  @return bool - TRUE if Real Clock Time is set successfully
 */
bool HandleTowerSetTime(void)
{
  // Check if parameters are within tower time limits
  if(Packet_Parameter1 < 24 && Packet_Parameter2 < 60 && Packet_Parameter3 < 60)
  {
      // Sets the Real Time Clock
      RTC_Set(Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
      return true;
  }
  return false;
}

/*! @brief Executes the command depending on what packet has been received
 *
 *  @return void
 */
void ReceivedPacket(void)
{
  bool success = false;                                         /*!< The success status of the received packet */
  uint8_t commandIgnoreAck = Packet_Command & ~PACKET_ACK_MASK; /*!< The command byte ignoring the ACK mask */
  uint8_t commandAck = Packet_Command & PACKET_ACK_MASK;        /*!< The command byte with the ACK mask */

  // AND the packet command byte with the bitwise inverse ACK MASK to ignore if ACK is requested
  if(commandIgnoreAck == COMMAND_STARTUP && Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Send tower startup packets
    success = HandleTowerStartup();
  }
  else if(commandIgnoreAck == COMMAND_VER && Packet_Parameter1 == 'v' && Packet_Parameter2 == 'x' && Packet_Parameter3 == 13)
  {
    // Send tower version packet
    success = HandleTowerVersion();
  }
  else if(commandIgnoreAck == COMMAND_NUM)
  {
    // Check if parameters match tower number GET or SET parameters
    success = HandleTowerNumber();
  }
  else if(commandIgnoreAck == COMMAND_MODE)
  {
    // Check if parameters match tower mode GET or SET parameters
    success = HandleTowerMode();
  }
  else if(commandIgnoreAck == COMMAND_PROGRAMBYTE)
  {
    // Send tower program byte packet
    success = HandleTowerProgramByte();
  }
  else if(commandIgnoreAck == COMMAND_READBYTE)
  {
    // Send tower read byte packet
    success = HandleTowerReadByte();
  }
  else if(commandIgnoreAck == COMMAND_TIME)
  {
    // Set the Real Time Clock time
    success = HandleTowerSetTime();
  }

  // AND the packet command byte with the ACK MASK to check if ACK is requested
  if(commandAck)
  {
    // Check the success status of the packet which was sent
    if(success == false)
    {
      // Return the sent packet with the NACK command if unsuccessful
      Packet_Put(commandIgnoreAck,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    }
    else
    {
      // Return the sent packet with the ACK command if successful
      Packet_Put(Packet_Command,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    }
  }

  // Reset the packet variables to 0
  Packet_Command    = 0;
  Packet_Parameter1 = 0;
  Packet_Parameter2 = 0;
  Packet_Parameter3 = 0;
  Packet_Checksum   = 0;
}


/*! @brief Toggles the green LED every time it is called
 *
 *  @return void
 */
void PITCallback(void* arg)
{
  // Toggle green LED
  LEDs_Toggle(LED_GREEN);
}

/*! @brief Turns off the blue LED every time it is called
 *
 *  @return void
 */
void FTMCallbackCh0(void* arg)
{
  // Turn off blue LED
  LEDs_Off(LED_BLUE);
}

/*! @brief Toggles the yellow LED and reads the current time to send to the PC
 *
 *  @return void
 */
void RTCCallback(void* arg)
{
  // Toggle the yellow LED
  LEDs_Toggle(LED_YELLOW);

  // Declare variable for hours, minutes seconds
  uint8_t hours, minutes, seconds;

  // Get the current time values
  RTC_Get(&hours, &minutes, &seconds);

  // Send time to PC
  Packet_Put(COMMAND_TIME, hours, minutes, seconds);
}

/*! @brief Initializes the main tower components by calling the initialization routines of the supporting software modules.
 *
 *  @return void
 */
bool TowerInit(void)
{
  return (
    Flash_Init() &&
    LEDs_Init() &&
    Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ) &&
    PIT_Init(CPU_BUS_CLK_HZ, PITCallback, NULL) &&
    FTM_Init() &&
    RTC_Init(RTCCallback, NULL)
  );
}

/*! @brief Sets the default or stored values of the tower
 *
 *  @return void
 */
bool TowerSet(const TFTMChannel* const aFTMChannel)
{
  // Success status of writing default values to Flash and FTM
  bool success = true;

  // Set the periodic interrupt timer 0 to 500ms
  PIT_Set(PERIOD_LED_GREEN, true);

  // Allocates an address in Flash memory to the tower number and tower mode
  if(Flash_AllocateVar((volatile void**)&NvTowerNb, sizeof(*NvTowerNb))
  && Flash_AllocateVar((volatile void**)&NvTowerMd, sizeof(*NvTowerMd)));
  {
    // Checks if tower number is clear
    if(_FH(NvTowerNb) == 0xFFFF)
    {
      // Sets the tower number to the default number
      if(!Flash_Write16((uint16_t*)NvTowerNb,(uint16_t)1519))
      {
        success = false;
      }
    }

    // Checks if tower mode is clear
    if(_FH(NvTowerMd) == 0xFFFF)
    {
      // Sets the tower mode to the default mode
      if(!Flash_Write16((uint16_t*)NvTowerMd,(uint16_t)1))
      {
        success = false;
      }
    }
  }

  // FTM Channel for received packet timer
  if(!FTM_Set(aFTMChannel))
  {
    success = false;
  }

  return success;
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */

  TFTMChannel receivedPacketTmr;          /*!< FTM Channel for received packet timer */
  receivedPacketTmr.channelNb             = 0;
  receivedPacketTmr.delayCount            = 24414;
  receivedPacketTmr.ioType.inputDetection = TIMER_INPUT_ANY;
  receivedPacketTmr.ioType.outputAction   = TIMER_OUTPUT_DISCONNECT;
  receivedPacketTmr.timerFunction         = TIMER_FUNCTION_OUTPUT_COMPARE;
  receivedPacketTmr.userFunction          = FTMCallbackCh0;
  receivedPacketTmr.userArguments         = NULL;

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */

  // Globally disable interrupts
  __DI();

  // Initializes the main tower components
  if(TowerInit())
  {
    // Sets the default or stored values of the main tower components
    if(TowerSet(&receivedPacketTmr))
    {
      // Turn on the orange LED to indicate the tower has initialized successfully
      LEDs_On(LED_ORANGE);
    }
  }

  // Globally enable interrupts
  __EI();

  // Send startup packets to PC
  HandleTowerStartup();

  for(;;)
  {
    // Check if a packet has been received
    if(Packet_Get())
    {
      // Turn on the blue LED if the 1 second timer is set
      if(FTM_StartTimer(&receivedPacketTmr))
      {
        LEDs_On(LED_BLUE);
      }

      // Execute a command depending on what packet has been received
      ReceivedPacket();
    }
  }

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
