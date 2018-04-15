/*
 * PIT.c
 *
 *  Created on: 15 Apr 2018
 *      Author: 12403756
 */
#include "PIT.h"

// Private gloabl variable for PIT
static uint32_t ModuleClk;
static void (*UserFunction)(void*);
static void* UserArguments;

/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @param userFunction is a pointer to a user callback function.
 *  @param userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
bool PIT_Init(const uint32_t moduleClk, void (*userFunction)(void*), void* userArguments)
{
  // Ensure global interrupts are disabled
  EnterCritical();

  // Enable PIT module in PIT_MCR
  //PIT_MCR &= ~PIT_MCR_MDIS_MASK;

  // Ensure the PIT0 in disabled
  //PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;

  // Enable interrupts flags for PIT0
  //PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;

  // Return global interrupts to how they were
  ExitCritical();

  return true;
}

/*! @brief Sets the value of the desired period of the PIT.
 *
 *  @param period The desired value of the timer period in nanoseconds.
 *  @param restart TRUE if the PIT is disabled, a new value set, and then enabled.
 *                 FALSE if the PIT will use the new value after a trigger event.
 *  @note The function will enable the timer and interrupts for the PIT.
 */
void PIT_Set(const uint32_t period, const bool restart)
{
  if(restart)
  {
    // Disable PIT0
    PIT_Enable(false);

    // Load the new LDVAL
    PIT_LDVAL0 = (((ModuleClk * period)/ 1000000000)-1);

    // Enable PIT0
    PIT_Enable(true);
  }

  else
  {
    // Load the new LDVAL
    PIT_LDVAL0 = (((ModuleClk * period)/ 1000000000)-1);
  }

}

/*! @brief Enables or disables the PIT.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void PIT_Enable(const bool enable)
{
  // Enable PIT0
  if(enable)
  {
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
  }
  // Disable PIT0
  else
  {
      PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;
  }
}

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  The user callback function will be called.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT_ISR(void)
{
  //Clear the timer interrupt flag
  PIT_TFLG0 &= ~PIT_TFLG_TIF_MASK;

  // Call user callback function to toggle green led
  if (UserFunction)
   (*UserFunction)(UserArguments);
}


