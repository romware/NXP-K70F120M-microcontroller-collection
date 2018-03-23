/*
 * packet.c
 *
 *  Created on: 22 Mar 2018
 *      Author: 12403756
 */


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
    uint8_t packet[PACKET_SIZE];
    for(uint8_t i = 0; i < PACKET_SIZE; i++)
    {
	FIFO_Get(&RxFIFO,packet[i]);
    }

    while(Packet_Error_Check(packet, PACKET_SIZE) == false)
    {
	for(uint8_t i = 0; i < PACKET_SIZE - 1; i++)
	{
	    packet[i] = packet[i + 1];
	}
	FIFO_Get(&RxFIFO,packet[PACKET_SIZE - 1]);
    }
}



/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *
 *  @return bool - TRUE if a valid packet was sent.
 */
bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{

}

bool Packet_Error_Check(const uint8_t packet[], const uint8_t packetLength)
{
    return (Packet_Checksum(packet, packetLength) == packet[packetLength - 1]);
}

uint8_t Packet_Checksum(const uint8_t packet[], const uint8_t packetLength)
{
    uint8_t xorValue = packet[0];
    for(uint8_t i = 1; i < packetLength; i++)
    {
	xorValue = xorValue ^ packet[i];
    }
    return xorValue;
}
