/*! @file RTC.c
 *
 *  @brief Routines for controlling the Real Time Clock (RTC) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the real time clock (RTC).
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
 */
/*!
**  @addtogroup RTC_module RTC module documentation
**  @{
*/
/* MODULE RTC */

#include "RTC.h"
#include "MK70F12.h"
#include "Cpu.h"

static void (*UserFunction)(void*); /*!< Callback functions for RTC */
static void* UserArguments;         /*!< Callback parameters for RTC */

/*! @brief Initializes the RTC before first use.
 *
 *  Sets up the control register for the RTC and locks it.
 *  Enables the RTC and sets an interrupt every second.
 *  @param userFunction is a pointer to a user callback function.
 *  @param userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the RTC was successfully initialized.
 */
bool RTC_Init(void (*userFunction)(void*), void* userArguments)
{
  // Store parameters for interrupt routine
  UserFunction = userFunction;
  UserArguments = userArguments;

  // Ensure global interrupts are disabled
  EnterCritical();

  /* Address     | Vector | IRQ  | NVIC non-IPR register | NVIC IPR register | Source module | Source description
   * 0x0000_014C | 83     | 67   | 2                     | 16                | RTC           | Seconds interrupt
   * IRQ modulo 32 = 3
   */

  // Clear any pending interrupts on FTM0
  NVICICPR2 |= (1 << 3);

  // Enable interrupts from FTM0 module
  NVICISER2 |= (1 << 3);

  // Return global interrupts to how they were
  ExitCritical();

  // Enable the Real Time Clock in System Clock Gating Control Register 6
  SIM_SCGC6 |= SIM_SCGC6_RTC_MASK;

  RTC_CR |= RTC_CR_UM_MASK;

  RTC_CR |= RTC_CR_SWR_MASK;

  // Unlock lock register
  RTC_LR |= RTC_LR_LRL_MASK;

  // Unlock control register
  RTC_LR |= (RTC_LR_CRL_MASK | RTC_LR_MCHL_MASK | RTC_LR_MCLL_MASK | RTC_LR_MEL_MASK | RTC_LR_SRL_MASK | RTC_LR_SRL_MASK | RTC_LR_TCL_MASK | RTC_LR_TTSL_MASK);

  // Set internal capacitance for 32.768 kHz oscillator to 18pF (16pF + 2pF) as per sheet 4 of TWR-K70F120M-SCH.pdf
  RTC_CR |= RTC_CR_SC2P_MASK | RTC_CR_SC16P_MASK;

  // Disable 32.768 kHz clock to other peripherals
  RTC_CR |= RTC_CR_CLKO_MASK;

  // Enable the 32.768 kHz clock. NOTE: Wait oscillator startup time to allow XTAL to stabilize
  RTC_CR = RTC_CR_OSCE_MASK;

  // Unlock control register
  RTC_LR &= ~RTC_LR_CRL_MASK;

  // Unlock status register
   RTC_LR |= RTC_LR_SRL_MASK;

  // Enable the time counter
  RTC_SR |= RTC_SR_TCE_MASK;

  // Lock status register
  //RTC_LR &= ~RTC_LR_SRL_MASK;

  // Enable 1 second interrupts from the RTC
  RTC_IER |= RTC_IER_TSIE_MASK;

  // Lock lock register
  RTC_LR &= ~RTC_LR_LRL_MASK;
  return true;
}

/*! @brief Sets the value of the real time clock.
 *
 *  @param hours The desired value of the real time clock hours (0-23).
 *  @param minutes The desired value of the real time clock minutes (0-59).
 *  @param seconds The desired value of the real time clock seconds (0-59).
 *  @note Assumes that the RTC module has been initialized and all input parameters are in range.
 */
void RTC_Set(const uint8_t hours, const uint8_t minutes, const uint8_t seconds)
{
  // Variable to store time in seconds and load with calculated value.
  uint32_t newTime = (seconds + minutes*60 + hours*60*60);

  // Disable the time counter to allow to load new time
  RTC_SR &= ~RTC_SR_TCE_MASK;

  // Load the new time value into the Time Seconds Register
  RTC_TSR = newTime;

  // Enable the time counter
  RTC_SR |= RTC_SR_TCE_MASK;
}

/*! @brief Gets the value of the real time clock.
 *
 *  @param hours The address of a variable to store the real time clock hours.
 *  @param minutes The address of a variable to store the real time clock minutes.
 *  @param seconds The address of a variable to store the real time clock seconds.
 *  @note Assumes that the RTC module has been initialized.
 */
void RTC_Get(uint8_t* const hours, uint8_t* const minutes, uint8_t* const seconds)
{
  // Variable to store the time in seconds from the Time Seconds Register
  uint32_t readTimeSeconds = RTC_TSR;

  // Calculate and set the hours (maximum 23 hrs, days disregarded)
  *hours = (readTimeSeconds / (60*60)) % 24;

  // Calculate and set the minutes
  *minutes = (readTimeSeconds % (60*60)) / 60;

  // Calculate and set the seconds
  *seconds = (readTimeSeconds % (60*60)) % 60;
}

/*! @brief Interrupt service routine for the RTC.
 *
 *  The RTC has incremented one second.
 *  The user callback function will be called.
 *  @note Assumes the RTC has been initialized.
 */
void __attribute__ ((interrupt)) RTC_ISR(void)
{
  // Call user callback function to toggle the yellow LED and send the new time to the PC
  if (UserFunction)
  (*UserFunction)(UserArguments);
}
/* END RTC */
/*!
** @}
*/
