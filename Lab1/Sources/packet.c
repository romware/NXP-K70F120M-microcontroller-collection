/*
 * packet.c
 *
 *  Created on: 22 Mar 2018
 *      Author: 12403756
 */

#include "packet.h"
#include "UART.h"
/*! @brief Initializes the packets by calling the initialization routines of the supporting software modules.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
 *  @return bool - TRUE if the packet module was successfully initialized.
 */
bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  return UART_Init(baudRate, moduleClk);
}


/*! @brief Attempts to get a packet from the received data.
 *
 *  @return bool - TRUE if a valid packet was received.
 */
bool Packet_Get(void)
{
  UART_InChar(&Packet_Command);
  UART_InChar(&Packet_Parameter1);
  UART_InChar(&Packet_Parameter2);
  UART_InChar(&Packet_Parameter3);
  UART_InChar(&Packet_Checksum);
  
  //Check packet checksum, load in new byte if incorrect
  while(Checksum_Generate(Packet_Command,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3) != Packet_Checksum)
  {
    Packet_Command = Packet_Parameter1;
    Packet_Parameter1 = Packet_Parameter2;
    Packet_Parameter2 = Packet_Parameter3;
    Packet_Parameter3 = Packet_Checksum;
    UART_InChar(&Packet_Checksum);
  }
  
  return true;
}


/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *
 *  @return bool - TRUE if a valid packet was sent.
 */
bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  uint8_t checksum = Checksum_Generate(command, parameter1, parameter2, parameter3);
  return (
    UART_OutChar(command) && 
    UART_OutChar(parameter1) && 
    UART_OutChar(parameter2) && 
    UART_OutChar(parameter3) && 
    UART_OutChar(checksum)
  );
}

uint8_t Checksum_Generate(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  return command ^ parameter1 ^ parameter2 ^ parameter3;
}
