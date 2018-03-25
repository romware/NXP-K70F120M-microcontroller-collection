/*
 * UART.c
 *
 *  Created on: 22 Mar 2018
 *      Author: 12403756
 */

/*! @brief Sets up the UART interface before first use.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
 *  @return bool - TRUE if the UART was successfully initialized.
 */
bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
    // enable UART2 by turning the clock on
    SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;

    // set PTE16 to be the UART2_Tx pin
    PORTE_PCR16 = PORT_PCR_MUX(3);

    // set PTE17 to be the UART2_Rx pin
    PORTE_PCR17 = PORT_PCR_MUX(3);

    // set UART2_C2 transmit enable to 1;
    UART2_C2_TE |= UART2_C2_TE_MASK;

    // set UART2_C2 receive enable to 1;
    UART2_C2_RE |= UART2_C2_RE_MASK;

    // page 323
    // example
    // 00010011
    // try and clear bit 4

    // bug! - will clear bits 0 and 1
    // 00010000
    // MY_W1C_REGISTER &= ~0x00010000;

    // 00000000
    // MY_W1C_REGISTER |= 0x00010000;

    // correct:
    // 00000011
    // MY_W1C_REGISTER = 0x00010000;
    // w1c = write 1 to clear
     uint16_t BRFA = 0;
     uint16_t SBR = 0;
     uint8_t BDH =0;
     uint8_t BDL =0;
     BRFA = (moduleClk*2)%32;			//calculate BRFA from %32
     SBR = moduleClk/(16*baudRate) - BRFA;	//calculate the module clock divisor
     BDL = (uint8_t)SBR & 0b11111111;		//mask the BDL
     SBR >> 8;					//shift bits [12:8] to [4:0]
     BDH = (uint8_t)SBR & 0b11111;		//mask the BDH
     UART2_BDH = BDH;				//Load the BDH register
     UART2_BDL = BDL;				//load the BDL register
     return true;
}



/*! @brief Get a character from the receive FIFO if it is not empty.
 *
 *  @param dataPtr A pointer to memory to store the retrieved byte.
 *  @return bool - TRUE if the receive FIFO returned a character.
 *  @note Assumes that UART_Init has been called.
 */
bool UART_InChar(uint8_t * const dataPtr)
{
    return FIFO_Get(&RxFIFO, &dataPtr);
}



/*! @brief Put a byte in the transmit FIFO if it is not full.
 *
 *  @param data The byte to be placed in the transmit FIFO.
 *  @return bool - TRUE if the data was placed in the transmit FIFO.
 *  @note Assumes that UART_Init has been called.
 */
bool UART_OutChar(const uint8_t data)
{
    return FIFO_Put(TxFIFO, data);
}



/*! @brief Poll the UART status register to try and receive and/or transmit one character.
 *
 *  @return void
 *  @note Assumes that UART_Init has been called.
 */
void UART_Poll(void)
{
    if(UART2_S1_RDRF)
    {
		//FIFO_Put(&RxFIFO, UART2_D);
		UART_OutChar(UART2_D);
		//clear flag
    }

    if(UART2_S1_TDRE)
    {
        //FIFO_Get(&TxFIFO, &UART2_D);
        UART_InChar(&UART2_D);
        //Clear flag maybe?
    }
}


