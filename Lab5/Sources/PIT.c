/*! @file PIT.c
 *
 *  @brief Routines for controlling Periodic Interrupt Timer (PIT) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the periodic interrupt timer (PIT).
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
 */
/*!
**  @addtogroup PIT_module PIT module documentation
**  @{
*/
/* MODULE PIT */

#include "PIT.h"
#include "MK70F12.h"
#include "Cpu.h"

static uint32_t ModuleClk;          /*!< Module Clock */
//static void (*UserFunction)(void*); /*!< Callback functions for PIT */
//static void* UserArguments;         /*!< Callback parameters for PIT */
static OS_ECB* UserSemaphore;


/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @param userFunction is a pointer to a user callback function.  //TODO: Update
 *  @param userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
bool PIT_Init(const uint32_t moduleClk, OS_ECB* userSemaphore)
{
  // Enable the Periodic Interrupt Timer in System Clock Gating Control Register 6
  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;

  // Store parameters for interrupt routine and PIT enable
  ModuleClk = moduleClk;
//  UserFunction = userFunction;
//  UserArguments = userArguments;
  UserSemaphore = userSemaphore;

  // Ensure global interrupts are disabled
  EnterCritical();

  /* Address     | Vector | IRQ  | NVIC non-IPR register | NVIC IPR register | Source module | Source description
   * 0x0000_0150 | 84     | 68   | 2                     | 17                | PIT           | Channel 0
   * IRQ modulo 32 = 4
   */

  // Enable PIT module in PIT_MCR
  PIT_MCR &= ~PIT_MCR_MDIS_MASK;

  // Clear any pending interrupts on PIT0
  NVICICPR2 |= (1 << 4);

  // Enable interrupts from PIT0 module
  NVICISER2 |= (1 << 4);

  // Enable interrupts flags for PIT0
  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;

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
  // Calculate nanoseconds per tick
  uint32_t nanoSecondPerTick = 1000000000 / ModuleClk;

  // Calculate timer load value
  uint32_t locLDVAL = (period / nanoSecondPerTick) - 1;

  // Check if restarting PIT
  if(restart)
  {
    // Disable PIT0
    PIT_Enable(false);

    // Load the new LDVAL
    PIT_LDVAL0 = locLDVAL;

    // Enable PIT0
    PIT_Enable(true);
  }
  else
  {
    // Load the new LDVAL
    PIT_LDVAL0 = locLDVAL;
  }
}

/*! @brief Enables or disables the PIT.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void PIT_Enable(const bool enable)
{
  if(enable)
  {
    // Enable PIT0
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
  }
  else
  {
    // Disable PIT0
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
  // Notify RTOS of start of ISR
  OS_ISREnter();
  // Clear the timer interrupt flag (W1C)
  PIT_TFLG0 = PIT_TFLG_TIF_MASK;

  // Call user callback function to toggle the green LED
//  if (UserFunction)
//   (*UserFunction)(UserArguments);
  OS_SemaphoreSignal(UserSemaphore);

  OS_ISRExit();
}

/* END PIT */
/*!
** @}
*/
