/* ###################################################################
**     Filename    : main.c
**     Project     : Lab2
**     Processor   : MK70FN1M0VMJ12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-07-20, 13:27, # CodeGen: 0
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


// UART baud rate
#define BAUD_RATE 115200

// Listed command bits of a packet
const uint8_t TOWER_STARTUP = 0x04;
const uint8_t TOWER_VER = 0x09;
const uint8_t TOWER_NUM = 0x0B;

// Tower major version and minor version
const uint8_t TOWER_VER_MAJ = 1;
const uint8_t TOWER_VER_MIN = 0;

// Tower number get and set bits of packet parameter 1
const uint8_t TOWER_NUM_GET = 1;
const uint8_t TOWER_NUM_SET = 2;

// Tower number most and least significant bits
uint16union_t Tower_Num_Union;


void leftToRight()
{
  for(uint32_t i = 0; i <320000; i++){}
  LEDs_Toggle(LED_BLUE);
  for(uint32_t i = 0; i <160000; i++){}
  LEDs_Toggle(LED_GREEN);
  for(uint32_t i = 0; i <200000; i++){}
  LEDs_Toggle(LED_YELLOW);
  for(uint32_t i = 0; i <240000; i++){}
  LEDs_Toggle(LED_ORANGE);
  for(uint32_t i = 0; i <280000; i++){}
}

void rightToLeft()
{
  for(uint32_t i = 0; i <280000; i++){}
  LEDs_Toggle(LED_ORANGE);
  for(uint32_t i = 0; i <240000; i++){}
  LEDs_Toggle(LED_YELLOW);
  for(uint32_t i = 0; i <200000; i++){}
  LEDs_Toggle(LED_GREEN);
  for(uint32_t i = 0; i <160000; i++){}
  LEDs_Toggle(LED_BLUE);
  for(uint32_t i = 0; i <320000; i++){}
}



/*! @brief Sends the startup packets to the PC
 *
 *  @return bool - TRUE if all packets are sent
 */
bool HandleTowerStartup(void)
{
  // Sends the tower startup values, version and number to the PC
  return (
    Packet_Put(TOWER_STARTUP,0x00,0x00,0x00) &&
    Packet_Put(TOWER_VER,'v',TOWER_VER_MAJ,TOWER_VER_MIN) &&
    Packet_Put(TOWER_NUM,TOWER_NUM_GET,Tower_Num_Union.s.Lo,Tower_Num_Union.s.Hi)
  );
}

/*! @brief Sends the version packet to the PC
 *
 *  @return bool - TRUE if packet is sent
 */
bool HandleTowerVersion(void)
{
  // Send tower number packet
  return Packet_Put(TOWER_NUM,TOWER_NUM_GET,Tower_Num_Union.s.Lo,Tower_Num_Union.s.Hi);
}

/*! @brief Sends or sets the number packet to the PC
 *
 *  @return bool - TRUE if packet is sent or number is set
 */
bool HandleTowerNumber(void)
{
  // Check if parameters match tower number GET or SET parameters
  if(Packet_Parameter1 == TOWER_NUM_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Send tower number packet
    return Packet_Put(TOWER_NUM,TOWER_NUM_GET,Tower_Num_Union.s.Lo,Tower_Num_Union.s.Hi);
  }
  else if (Packet_Parameter1 == TOWER_NUM_SET)
  {
    // Change tower number
    Tower_Num_Union.s.Lo = Packet_Parameter2;
    Tower_Num_Union.s.Hi = Packet_Parameter3;
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
  // Initializes the success status of the received packet to false
  bool success = false;
  uint8_t commandIgnoreAck = Packet_Command & ~PACKET_ACK_MASK;
  uint8_t commandAck = Packet_Command & PACKET_ACK_MASK;

  // AND the packet command byte with the bitwise inverse ACK MASK to ignore if ACK is requested
  if(commandIgnoreAck == TOWER_STARTUP && Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Send tower startup packets
    success = HandleTowerStartup();
  }
  else if(commandIgnoreAck == TOWER_VER && Packet_Parameter1 == 'v' && Packet_Parameter2 == 'x' && Packet_Parameter3 == 13)
  {
    // Send tower version packet
    success = HandleTowerVersion();
  }
  else if(commandIgnoreAck == TOWER_NUM)
  {
    // Check if parameters match tower number GET or SET parameters
    success = HandleTowerNumber();
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

  // Set Tower Number.
  Tower_Num_Union.l = 1519;

  // Initializes the LEDs by calling the initialization routines of the supporting software modules.
  LEDs_Init();

  // Initializes the packets by calling the initialization routines of the supporting software modules.
  Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ);

  volatile uint16union_t *NvTowerNb; /*!< The non-volatile Tower number. */


  // Send startup packets to PC
  HandleTowerStartup();

  for (;;)
  {
    // Poll UART2 for packets to transmit and receive
    UART_Poll();


    uint64union_t corn;
    corn.l = 0x00000000FFCC00BB;
    Flash_Init();
    WritePhrase(FLASH_DATA_START, corn);

    uint64union_t* cobReturnPtr = (uint64union_t*)FLASH_DATA_START;

    uint64union_t returnOfTheCob;
    returnOfTheCob = *cobReturnPtr;




    // Check if packet has been received
    if(Packet_Get())
    {
      // Execute a command depending on what packet has been received
      ReceivedPacket();
      leftToRight();
      leftToRight();
      //rightToLeft();
      //rightToLeft();
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
