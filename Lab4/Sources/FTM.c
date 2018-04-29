/*! @file FTM.c
 *
 *  @brief Routines for setting up the FlexTimer module (FTM) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the FlexTimer module (FTM).
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
 */
/*!
**  @addtogroup FTM_module FTM module documentation
**  @{
*/
/* MODULE FTM */

#include "FTM.h"
#include "MK70F12.h"
#include "Cpu.h"

static void (*UserFunction[8])(void*); /*!< Array to store callback functions for channels 0-7 of FTM0 */
static void* UserArguments[8];         /*!< Array to store callback parameters for channels 0-7 of FTM0 */

/*! @brief Sets up the FTM before first use.
 *
 *  Enables the FTM as a free running 16-bit counter.
 *  @return bool - TRUE if the FTM was successfully initialized.
 */
bool FTM_Init()
{
  // Ensure global interrupts are disabled
  EnterCritical();

  /* Address     | Vector | IRQ  | NVIC non-IPR register | NVIC IPR register | Source module | Source description
   * 0x0000_0138 | 78     | 62   | 1                     | 15                | FTM0          | Single interrupt vector for all sources
   * IRQ modulo 32 = 30
   */

  // Clear any pending interrupts on FTM0
  NVICICPR1 |= (1 << 30);

  // Enable interrupts from FTM0 module
  NVICISER1 |= (1 << 30);

  // Return global interrupts to how they were
  ExitCritical();

  // Enable the Flexible Timer Module in System Clock Gating Control Register 6
  SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;

  // Turn off write protect mode
  FTM0_MODE |= FTM_MODE_WPDIS_MASK;

  // Enable FTM
  FTM0_MODE |= FTM_MODE_FTMEN_MASK;

  // Set the BDM to 3 for functional mode
  FTM0_CONF |= FTM_CONF_BDMMODE(3);

  // Set counter initial value to 0
  FTM0_CNTIN = 0x0000;

  // Set overflow value to 0xFFFF
  FTM0_MOD = 0xFFFF;

  // Write anything to CNT to start it
  FTM0_CNT = 0xFFFF;

  // Set clock source to fixed frequency clock (0b10)
  FTM0_SC |= FTM_SC_CLKS(0b10);

  // Return to write protect mode
  FTM0_FMS |= FTM_FMS_WPEN_MASK;

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
  // Store parameters for interrupt routine in the private global arrays
  UserFunction[aFTMChannel->channelNb] = aFTMChannel->userFunction;
  UserArguments[aFTMChannel->channelNb] = aFTMChannel->userArguments;

  // Turn off write protect mode
  FTM0_MODE |= FTM_MODE_WPDIS_MASK;

  // Create 2 bit mask for Mode Select A and Mode select B to AND with the input detection bits
  FTM0_CnSC(aFTMChannel->channelNb) |= (FTM_CnSC_MSA_MASK | FTM_CnSC_MSB_MASK)
  & (aFTMChannel->timerFunction << FTM_CnSC_MSA_SHIFT);

  // Check if timer function is an input capture or output compare
  if(aFTMChannel->timerFunction == TIMER_FUNCTION_INPUT_CAPTURE)
  {
    // Create 2 bit mask for ELS LSB and MSB to AND with the input detection bits
    FTM0_CnSC(aFTMChannel->channelNb) |= (FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK)
    & (aFTMChannel->ioType.inputDetection << FTM_CnSC_ELSA_SHIFT);
  }
  else if (aFTMChannel->timerFunction == TIMER_FUNCTION_OUTPUT_COMPARE)
  {
    // Create 2 bit mask for ELS LSB and MSB to AND with the output action bits
    FTM0_CnSC(aFTMChannel->channelNb) |= (FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK)
    & (aFTMChannel->ioType.outputAction << FTM_CnSC_ELSB_SHIFT);
  }

  // Return to write protect mode
  FTM0_FMS |= FTM_FMS_WPEN_MASK;

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
  // Set the channel value
  FTM0_CnV(aFTMChannel->channelNb) = (FTM_CNT_COUNT_MASK & FTM0_CNT) + aFTMChannel->delayCount;

  // Clear the channel flag
  FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_CHF_MASK;

  // Enable Channel interrupts
  FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_CHIE_MASK;

  return true;
}

/*! @brief Interrupt service routine for the FTM.
 *
 *  If a timer channel was set up as output compare, then the user callback function will be called.
 *  @note Assumes the FTM has been initialized.
 */
void __attribute__ ((interrupt)) FTM0_ISR(void)
{
  // Check for set channel flags
  for(uint8_t i = 0; i < 8; i++)
  {
    if(FTM0_CnSC(i) & FTM_CnSC_CHF_MASK)
    {
      // Clear the channel flag
      FTM0_CnSC(i) &= ~FTM_CnSC_CHF_MASK;

      // Disable the channel interrupt
      FTM0_CnSC(i) &= ~FTM_CnSC_CHIE_MASK;

      // Call up callback function for channels with set flags
      if(UserFunction[i])
        (*UserFunction[i])(UserArguments[i]);
    }
  }
}
/* END FTM */
/*!
** @}
*/
