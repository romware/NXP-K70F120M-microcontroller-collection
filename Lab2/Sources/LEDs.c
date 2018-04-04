/*
 * LEDs.c
 *
 *  Created on: 4 Apr 2018
 *      Author: 12403756
 */

#include "MK70F12.h"
#include "LEDs.h"
/*! @brief Sets up the LEDs before first use.
 *
 *  @return bool - TRUE if the LEDs were successfully initialized.
 */
bool LEDs_Init(void)
{

  //GPIOA_PDDR |=  GPIO_PDD_PIN_10;
  //GPIOA_PDDR |=  GPIO_PDD_PIN_11;
  //GPIOA_PDDR |=  GPIO_PDD_PIN_28;
  //GPIOA_PDDR |=  GPIO_PDD_PIN_29;
  PORTA_PCR10 = PORT_PCR_MUX(1);
  PORTA_PCR11 = PORT_PCR_MUX(1);
  PORTA_PCR28 = PORT_PCR_MUX(1);
  PORTA_PCR29 = PORT_PCR_MUX(1);

  PORTA_PCR10 |= PORT_PCR_DSE_MASK;
  PORTA_PCR11 |= PORT_PCR_DSE_MASK;
  PORTA_PCR28 |= PORT_PCR_DSE_MASK;
  PORTA_PCR29 |= PORT_PCR_DSE_MASK;
}

