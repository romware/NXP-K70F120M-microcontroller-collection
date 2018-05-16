/*! @file LEDs.c
 *
 *  @brief Routines to access the LEDs on the TWR-K70F120M.
 *
 *  This contains the functions for operating the LEDs.
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
 */
/*!
**  @addtogroup LEDs_module LEDs module documentation
**  @{
*/
/* MODULE LEDs */

#include "MK70F12.h"
#include "LEDs.h"

/*! @brief Sets up the LEDs before first use.
 *
 *  @return bool - TRUE if the LEDs were successfully initialized.
 */
bool LEDs_Init(void)
{
  // Enable the Port A gate clock via System Clock Gating Control Register 5 
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
  
  // Set multiplexing on LED pins for GPIO
  PORTA_PCR10 = PORT_PCR_MUX(1);
  PORTA_PCR11 = PORT_PCR_MUX(1);
  PORTA_PCR28 = PORT_PCR_MUX(1);
  PORTA_PCR29 = PORT_PCR_MUX(1);

  // Set LED pins to be outputs
  GPIOA_PDDR |= LED_ORANGE;
  GPIOA_PDDR |= LED_YELLOW;
  GPIOA_PDDR |= LED_GREEN;
  GPIOA_PDDR |= LED_BLUE;

  // Turn off all LEDs
  GPIOA_PSOR = LED_ORANGE;
  GPIOA_PSOR = LED_YELLOW;
  GPIOA_PSOR = LED_GREEN;
  GPIOA_PSOR = LED_BLUE;

  // Set Drain Strength Enable of LED pins
  PORTA_PCR10 |= PORT_PCR_DSE_MASK;
  PORTA_PCR11 |= PORT_PCR_DSE_MASK;
  PORTA_PCR28 |= PORT_PCR_DSE_MASK;
  PORTA_PCR29 |= PORT_PCR_DSE_MASK;
  
  return true;
}

/*! @brief Turns an LED on.
 *
 *  @param color The color of the LED to turn on.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_On(const TLED color)
{
  // Clear the GPIOA_PDD by writing to the GPIOA_PCOR with the LED color mask (logic 0 to turn on LEDs)
  GPIOA_PCOR |= color;
}

/*! @brief Turns off an LED.
 *
 *  @param color THe color of the LED to turn off.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_Off(const TLED color)
{
  // Set the GPIOA_PDD by writing to the GPIO_PSOR with the LED color mask  (logic 1 to turn off LEDs)
  GPIOA_PSOR |= color;
}

/*! @brief Toggles an LED.
 *
 *  @param color THe color of the LED to toggle.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_Toggle(const TLED color)
{
  // Toggle the GPIOA_PDD by writing to the GPIOA_PTOR with the LED color mask
  GPIOA_PTOR |= color;
}

/* END LEDs */
/*!
** @}
*/
