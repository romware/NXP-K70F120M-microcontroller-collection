/* ###################################################################
**     Filename    : main.c
**     Project     : Lab1
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

//UART baud rate
#define UART_BAUD_RATE 38400

#define TOWER_STARTUP 0x04
#define TOWER_VER 0x09
#define TOWER_NUM 0x0B

#define TOWER_VER_MAJ 1
#define TOWER_VER_MIN 0

#define TOWER_NUM_GET 1
#define TOWER_NUM_SET 2

#define TOWER_ACK_MASK 0x80
#define TOWER_NACK_MASK 0x7F

//  ACK 128 - 0b10000000
// NACK 127 - 0b01111111

uint8_t TOWER_NUM_MSB = 0x05;
uint8_t TOWER_NUM_LSB = 0xEF;

// MSB   5 - 0b00000101
// LSB 160 - 0b10100000

// Packet structure
uint8_t Packet_Command,		/*!< The packet's command */
	Packet_Parameter1, 	/*!< The packet's 1st parameter */
	Packet_Parameter2, 	/*!< The packet's 2nd parameter */
	Packet_Parameter3,	/*!< The packet's 3rd parameter */
	Packet_Checksum;	/*!< The packet's checksum */

TFIFO RxFIFO;
TFIFO TxFIFO;

bool Startup_Packets(void)
{
  return (
    Packet_Put(TOWER_STARTUP,0x00,0x00,0x00) && 
    Packet_Put(TOWER_VER,'v',TOWER_VER_MAJ,TOWER_VER_MIN) && 
    Packet_Put(TOWER_NUM,TOWER_NUM_GET,TOWER_NUM_LSB,TOWER_NUM_MSB)
  );
}


void Handle_Packets(void)
{
  bool acknowledgment = FALSE;
  
  switch (Packet_Command & TOWER_NACK_MASK)
  {
    case TOWER_STARTUP:
      if(Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
      {
        acknowledgment = Startup_Packets();
      }
      break;
    
    case TOWER_VER:
      if(Packet_Parameter1 == 'v' && Packet_Parameter2 == 'x' && Packet_Parameter3 == 13)
      {
        acknowledgment = Packet_Put(TOWER_VER,'v',TOWER_VER_MAJ,TOWER_VER_MIN);
      }
      break;
    
    case TOWER_NUM:
      if(Packet_Parameter1 == TOWER_NUM_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
      {
        acknowledgment = Packet_Put(TOWER_NUM,TOWER_NUM_GET,TOWER_NUM_LSB,TOWER_NUM_MSB);
      }
      else if (Packet_Parameter1 == TOWER_NUM_SET)
      {
        TOWER_NUM_LSB = Packet_Parameter2;
        TOWER_NUM_MSB = Packet_Parameter3;
        acknowledgment = TRUE;
      }
      break;

    default:
      acknowledgment = FALSE;
      break;
  }
   
  if(Packet_Command & TOWER_ACK_MASK)
  {
    if(acknowledgment == FALSE)
    {
      Packet_Put(Packet_Command & TOWER_NACK_MASK,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    }
    else
    {
      Packet_Put(Packet_Command,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    }
  }
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
  Packet_Init(UART_BAUD_RATE, CPU_BUS_CLK_HZ);
  
  Startup_Packets();
  
  for (;;)
  {
    UART_Poll();
    if(Packet_Get())
    {
      Handle_Packets();
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
