/* ###################################################################
**     Filename    : main.c
**     Project     : Lab1
**     Processor   : MK70FN1M0VMJ12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-04-02, 13:27, # CodeGen: 0
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
** @version 1.0
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


// CPU mpdule - contains low level hardware initialization routines
#include "Cpu.h"
#include "MK70F12.h"
#include "UART.h"
#include "FIFO.h"
#include "packet.h"

// UART baud rate
#define UART_BAUD_RATE 38400

/*! @brief Sends the startup packets to the PC
 *
 *  @return bool - TRUE if all packets are sent
 */
bool HandleTowerStartup(void)
{
  // Sends the tower startup values, version and number to the PC
  return (
    Packet_Put(PACKET_TOWER_STARTUP,0x00,0x00,0x00) &&
    Packet_Put(PACKET_TOWER_VER,'v',PACKET_TOWER_VER_MAJ,PACKET_TOWER_VER_MIN) &&
    Packet_Put(PACKET_TOWER_NUM,PACKET_TOWER_NUM_GET,Packet_Tower_Num_LSB,Packet_Tower_Num_MSB)
  );
}

/*! @brief Sends the version packet to the PC
 *
 *  @return bool - TRUE if packet is sent
 */
bool HandleTowerVersion(void)
{
  // Send tower number packet
  return Packet_Put(PACKET_TOWER_NUM,PACKET_TOWER_NUM_GET,Packet_Tower_Num_LSB,Packet_Tower_Num_MSB);
}

/*! @brief Sends or sets the number packet to the PC
 *
 *  @return bool - TRUE if packet is sent or number is set
 */
bool HandleTowerNumber(void)
{
  // Check if parameters match tower number GET or SET parameters
  if(Packet_Parameter1 == PACKET_TOWER_NUM_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Send tower number packet
    return Packet_Put(PACKET_TOWER_NUM,PACKET_TOWER_NUM_GET,Packet_Tower_Num_LSB,Packet_Tower_Num_MSB);
  }
  else if (Packet_Parameter1 == PACKET_TOWER_NUM_SET)
  {
    // Change tower number
    Packet_Tower_Num_LSB = Packet_Parameter2;
    Packet_Tower_Num_MSB = Packet_Parameter3;
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
  if(commandIgnoreAck == PACKET_TOWER_STARTUP && Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Send tower startup packets
    success = HandleTowerStartup();
  }
  else if(commandIgnoreAck == PACKET_TOWER_VER && Packet_Parameter1 == 'v' && Packet_Parameter2 == 'x' && Packet_Parameter3 == 13)
  {
    // Send tower version packet
    success = HandleTowerVersion();
  }
  else if(commandIgnoreAck == PACKET_TOWER_NUM)
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

  // Initializes the packets by calling the initialization routines of the supporting software modules.
  Packet_Init(UART_BAUD_RATE, CPU_BUS_CLK_HZ);
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

/*
 *
 * TODO
 * fix naming conventions particularly for packet
 * use constants
 * move all variables into correct modules
 * check br formula
 * main functionality lives in main - move startup and handle
 */
