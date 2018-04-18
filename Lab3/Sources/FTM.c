/*
 * FTM.c
 *
 *  Created on: 15 Apr 2018
 *      Author: 12403756
 */
#include "FTM.h"
#include "MK70F12.h"
#include "Cpu.h"

// Private global variable for PIT
static void (*UserFunction)(void*);
static void* UserArguments;

/*! @brief Sets up the FTM before first use.
 *
 *  Enables the FTM as a free running 16-bit counter.
 *  @return bool - TRUE if the FTM was successfully initialized.
 */
bool FTM_Init()
{
  // Enable the Flexible Timer Module in System Clock Gating Control Register 6
  SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;

  // Ensure global interrupts are disabled
  EnterCritical();

  // Address     | Vector | IRQ1 | NVIC non-IPR register | NVIC IPR register | Source module | Source description
  // 0x0000_0138 | 78     | 62   | 1                     | 15                | FTM0          | Single interrupt vector for all sources
  // IRQ1 modulo 32 = 30

  // Clear any pending interrupts on FTM0
  NVICICPR1 |= (1 << 30);

  // Enable interrupts from FTM0 module
  NVICISER1 |= (1 << 30);

  // Set timer overflow interrupt enable to 1
  FTM0_SC |= FTM_SC_TOIE_MASK;

  // Set clock source to fixed frequency clock (0b10)
  FTM0_SC |= FTM_SC_CLKS(0b10);



  // Return global interrupts to how they were
  ExitCritical();

  return true;
}

/*! @brief Sets up a timer channel.
 *
 *  @param aFTMChannel is a structure containing the parameters to be used in setting up the timer channel.
 *    channelNb is the channel number of the FTM to use.
 *    delayCount is the delay count (in module clock periods) for an output compare event.
 *    timerFunction is used to set the timer up as either an input capture or an output compare.
 *    ioType is a union that depends on the setting of the channel as input capture or output compare:
 *      outputAction is the action to take on a successful output compare.
 *      inputDetection is the type of input capture detection.
 *    userFunction is a pointer to a user callback function.
 *    userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the timer was set up successfully.
 *  @note Assumes the FTM has been initialized.
 */
bool FTM_Set(const TFTMChannel* const aFTMChannel)
{
  // Store parameters for interrupt routine
  UserFunction = aFTMChannel->userFunction;
  UserArguments = aFTMChannel->userArguments;

  // Set the channel interrupt enable
  FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_CHIE_MASK;

  return true;
}


/*! @brief Starts a timer if set up for output compare.
 *
 *  @param aFTMChannel is a structure containing the parameters to be used in setting up the timer channel.
 *  @return bool - TRUE if the timer was started successfully.
 *  @note Assumes the FTM has been initialized.
 */
bool FTM_StartTimer(const TFTMChannel* const aFTMChannel)
{

}


/*! @brief Interrupt service routine for the FTM.
 *
 *  If a timer channel was set up as output compare, then the user callback function will be called.
 *  @note Assumes the FTM has been initialized.
 */
void __attribute__ ((interrupt)) FTM0_ISR(void)
{
  // Clear interrupt flag
  FTM0_CnSC(0) &= ~FTM_CnSC_CHF_MASK;

  // Call user callback function to toggle the blue LED
  if (UserFunction)
   (*UserFunction)(UserArguments);
}
