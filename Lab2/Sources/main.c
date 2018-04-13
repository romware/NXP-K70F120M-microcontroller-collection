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
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


// CPU module - contains low level hardware initialization routines
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


// Baud rate
#define BAUD_RATE 115200

// Listed command bits of a packet
const uint8_t COMMAND_STARTUP = 0x04;
const uint8_t COMMAND_VER = 0x09;
const uint8_t COMMAND_NUM = 0x0B;
const uint8_t COMMAND_PROGRAMBYTE = 0x07;
const uint8_t COMMAND_READBYTE = 0x08;
const uint8_t COMMAND_MODE = 0x0D;

// Get and set bits of packet parameter 1
const uint8_t PARAM_GET = 1;
const uint8_t PARAM_SET = 2;

// Tower major version and minor version
const uint8_t TOWER_VER_MAJ = 1;
const uint8_t TOWER_VER_MIN = 0;

// Tower number union pointer to flash
volatile uint16union_t* NvTowerNb;

// Tower mode union pointer to flash
volatile uint16union_t* NvTowerMd;


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

/*! @brief Erases or programs a byte to Flash based on packet recieved
 *
 *  @return bool - TRUE if Flash was modified
 */
bool HandleTowerProgramByte(void)
{
  // Check if offset is erase or set
  if(Packet_Parameter1 == 0x08)
  {
    // Erase Flash
    return Flash_Erase();
  }
  else if (Packet_Parameter1 >= 0x00 && Packet_Parameter1 <= 0x07)
  {
    // Program byte to Flash
    volatile uint8_t* nvData = FLASH_DATA_START + Packet_Parameter1;
    return Flash_Write8( (uint8_t*)nvData, Packet_Parameter3 );
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
  if (Packet_Parameter1 >= 0x00 && Packet_Parameter1 <= 0x07)
  {
    // Send read byte packet
    return Packet_Put(COMMAND_READBYTE,Packet_Parameter1,0,FLASH_DATA_START + Packet_Parameter1);
  }
  return false;
}

/*! @brief Sets the tower mode or sends the packet to the PC
 *
 *  @return bool - TRUE if packet is sent or number is set
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

/*! @brief Executes the command depending on what packet has been received
 *
 *  @return void
 */
void ReceivedPacket(void)
{
  // Initializes the success status of the received packet to false
  bool success = false;
  uint8_t commandIgnoreAck = Packet_Command & ~PACKET_ACK_MASK;
  uint8_t commandAck = Packet_Command & PACKET_ACK_MASK;

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
  else if(commandIgnoreAck == COMMAND_PROGRAMBYTE && Packet_Parameter2 == 0)
  {
    // Send tower program byte packet
    success = HandleTowerProgramByte();
  }
  else if(commandIgnoreAck == COMMAND_READBYTE && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Send tower read byte packet
    success = HandleTowerReadByte();
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
  Packet_Command = 0;
  Packet_Parameter1 = 0;
  Packet_Parameter2 = 0;
  Packet_Parameter3 = 0;
  Packet_Checksum = 0;
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */

  // Initializes the LEDs, UART and FLASH by calling the initialization routines of the supporting software modules.
  if(Flash_Init() && LEDs_Init() && Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ))
  {
  	// Allocates an adress in Flash memory to the tower number
    Flash_AllocateVar((volatile void**)&NvTowerNb, sizeof(*NvTowerNb));
    
  	// Allocates an adress in Flash memory to the tower mode
    Flash_AllocateVar((volatile void**)&NvTowerMd, sizeof(*NvTowerMd));
    
    // Checks if tower number is unset
    if(_FH(NvTowerNb) == 0xFFFF)
    {
      // Sets the tower number to the default number
      Flash_Write16((uint16_t*)NvTowerNb,(uint16_t)1519);
    }
    
    // Checks if tower mode is unset
    if(_FH(NvTowerMd) == 0xFFFF)
    {
      // Sets the tower mode to the default mode
      Flash_Write16((uint16_t*)NvTowerMd,(uint16_t)1);
    }
    
    // Turn on the orange LED to indicate the tower has initialised successfully
    LEDs_On(LED_ORANGE);
  }

  // Send startup packets to PC
  HandleTowerStartup();

  for (;;)
  {
    // Poll UART2 for packets to transmit and receive
    UART_Poll();

    // Check if packet has been received
    if(Packet_Get())
    {
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
