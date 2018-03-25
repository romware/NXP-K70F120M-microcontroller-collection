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
bool Packet_Get(void)		//Should we pass PacketSize??
{
    //Declare variables
    uint8_t packet[PACKET_SIZE];

    // Load PACKET_SIZE bytes in packet[] from RxFIFO
    for(uint8_t i = 0; i < PACKET_SIZE; i++)
    {
	FIFO_Get(&RxFIFO,packet[i]);
    }

    //Check packet checksum, load in new byte if incorrect
    while(Packet_Error_Check(packet, PACKET_SIZE) == false)
    {
	//Shift bytes along packet[]
	for(uint8_t i = 0; i < PACKET_SIZE - 1; i++)
	{
	    packet[i] = packet[i + 1];
	}
	//Load in new last byte into packet[]
	FIFO_Get(&RxFIFO,packet[PACKET_SIZE - 1]);
    }
    return true;
}


/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *
 *  @return bool - TRUE if a valid packet was sent.
 */
bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
    //Declare variables
    uint8_t checkSum;
    uint8_t packet[PACKET_SIZE];

    //Populate the first 4 bytes of the packet
    packet[0] = command;
    packet[1] = parameter1;
    packet[2] = parameter2;
    packet[3] = parameter3;

    //Generate checksum
    checkSum = Packet_Checksum(packet, PACKET_SIZE);

    //Load checksum into packet
    packet[PACKET_SIZE -1] = checkSum;

    //Send packet to TxFIFO
    for(uint8_t i = 0; i < PACKET_SIZE; i++)
        {
    	FIFO_Put(&TxFIFO,packet[i]);
        }
    return true;
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
