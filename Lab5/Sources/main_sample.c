/*!
** @file
** @version 1.0
** @brief  Main module.
**
**   This file contains the high-level code for the project.
**   It initialises appropriate hardware subsystems,
**   creates application threads, and then starts the OS.
**
**   An example of two threads communicating via a semaphore
**   is given that flashes the orange LED. These should be removed
**   when the use of threads and the RTOS is understood.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


// CPU module - contains low level hardware initialization routines
#include "Cpu.h"

// Simple OS
#include "OS.h"

/*! @brief LED to pin mapping on the TWR-K70F120M
 *
 */
typedef enum
{
  LED_ORANGE = (1 << 11),
  LED_YELLOW = (1 << 28),
  LED_GREEN = (1 << 29),
  LED_BLUE = (1 << 10)
} TLED;

// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100
#define NB_LEDS 4

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
static uint32_t MyLEDThreadStacks[NB_LEDS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

// ----------------------------------------
// Thread priorities
// 0 = highest priority
// ----------------------------------------
const uint8_t LED_THREAD_PRIORITIES[NB_LEDS] = {1, 2, 3, 4};

/*! @brief Data structure used to pass LED configuration to a user thread
 *
 */
typedef struct LEDThreadData
{
  OS_ECB* semaphore;
  TLED color;
  uint8_t delay;
  struct LEDThreadData* next;
} TLEDThreadData;

/*! @brief LED thread configuration data
 *
 */
static TLEDThreadData MyLEDThreadData[NB_LEDS] =
{
  {
    .semaphore = NULL,
    .color = LED_BLUE,
    .delay = 8,
    .next = &MyLEDThreadData[1],
  },
  {
    .semaphore = NULL,
    .color = LED_GREEN,
    .delay = 15,
    .next = &MyLEDThreadData[2],
  },
  {
    .semaphore = NULL,
    .color = LED_YELLOW,
    .delay = 29,
    .next = &MyLEDThreadData[0],
  },
  {
    .semaphore = NULL,
    .color = LED_ORANGE,
    .delay = 0,
    .next = NULL,
  }
};

void LPTMRInit(const uint16_t count)
{
  // Enable clock gate to LPTMR module
  SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;

  // Disable the LPTMR while we set up
  // This also clears the CSR[TCF] bit which indicates a pending interrupt
  LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;

  // Enable LPTMR interrupts
  LPTMR0_CSR |= LPTMR_CSR_TIE_MASK;
  // Reset the LPTMR free running counter whenever the 'counter' equals 'compare'
  LPTMR0_CSR &= ~LPTMR_CSR_TFC_MASK;
  // Set the LPTMR as a timer rather than a counter
  LPTMR0_CSR &= ~LPTMR_CSR_TMS_MASK;

  // Bypass the prescaler
  LPTMR0_PSR |= LPTMR_PSR_PBYP_MASK;
  // Select the prescaler clock source
  LPTMR0_PSR = (LPTMR0_PSR & ~LPTMR_PSR_PCS(0x3)) | LPTMR_PSR_PCS(1);

  // Set compare value
  LPTMR0_CMR = LPTMR_CMR_COMPARE(count);

  // Initialize NVIC
  // see p. 91 of K70P256M150SF3RM.pdf
  // Vector 0x65=101, IRQ=85
  // NVIC non-IPR=2 IPR=21
  // Clear any pending interrupts on LPTMR
  NVICICPR2 = NVIC_ICPR_CLRPEND(1 << 21);
  // Enable interrupts from LPTMR module
  NVICISER2 = NVIC_ISER_SETENA(1 << 21);

  //Turn on LPTMR and start counting
  LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;
}

/*! @brief Initialises the LEDs.
 *
 */
void LEDInit()
{
  uint32_t gpwd;

  // Enable clock gate for Port A to enable pin routing
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

  // Set up each port pin so that initially the LED is off
  GPIOA_PSOR = LED_ORANGE | LED_YELLOW | LED_GREEN | LED_BLUE;

  // Set up each port pin to be an output
  GPIOA_PDDR |= (LED_ORANGE | LED_YELLOW | LED_GREEN | LED_BLUE);

  // *** for PIN multiplexing, see p. 282 of K70P256M150SF3RM.pdf ***
  // PORTA_PCRx: ISF=0, MUX=1 (see p. 316 of K70P256M150SF3RM.pdf)
  // Use the global pin control registers since the pins are the same

  gpwd = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
  PORTA_GPCLR = ((LED_ORANGE | LED_BLUE) << 16) | gpwd;
  PORTA_GPCHR = (LED_YELLOW | LED_GREEN) | gpwd;
}

/*! @brief Initialises the modules to support the LEDs and low power timer.
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note This thread deletes itself after running for the first time.
 */
static void InitModulesThread(void* pData)
{
  // Initialise the low power timer to tick every 0.5s
  LPTMRInit(500);

  // Initialise the LEDs
  LEDInit();

  // Generate the three global LED semaphores
  for (uint8_t ledNb = 0; ledNb < NB_LEDS; ledNb++)
    MyLEDThreadData[ledNb].semaphore = OS_SemaphoreCreate(0);

  // Signal the first LED to toggle
  (void)OS_SemaphoreSignal(MyLEDThreadData[0].semaphore);

  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

void __attribute__ ((interrupt)) LPTimer_ISR(void)
{
  // Clear interrupt flag
  LPTMR0_CSR |= LPTMR_CSR_TCF_MASK;
  // Signal the orange LED to toggle
  (void)OS_SemaphoreSignal(MyLEDThreadData[3].semaphore);
}

/*! @brief Waits for a signal to toggle the LED, then waits for a specified delay, then signals for the next LED to toggle.
 *
 *  @param pData holds the configuration data for each LED thread.
 *  @note Assumes that LEDInit has been called successfully.
 */
static void LEDThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define ledData ((TLEDThreadData*)pData)

  for (;;)
  {
    // Wait here until signaled that we can turn the LED on
    (void)OS_SemaphoreWait(ledData->semaphore, 0);

    // Toggle LED
    GPIOA_PTOR = ledData->color;

    // Wait for the required toggle time if the delay > 0
    if (ledData->delay)
      OS_TimeDelay(ledData->delay);

    // Signal for the next LED to toggle
    if (ledData->next->semaphore)
      (void)OS_SemaphoreSignal(ledData->next->semaphore);
  }
}

/*! @brief Initialises the hardware, sets up threads, and starts the OS.
 *
 */
/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error;

  // Initialise low-level clocks etc using Processor Expert code
  PE_low_level_init();

  // Initialize the RTOS - without flashing the orange LED "heartbeat"
  OS_Init(CPU_CORE_CLK_HZ, false);

  // Create module initialisation thread
  error = OS_ThreadCreate(InitModulesThread,
                          NULL,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
		          0); // Highest priority

  // Create threads to toggle the LEDS
  for (uint8_t threadNb = 0; threadNb < NB_LEDS; threadNb++)
  {
    error = OS_ThreadCreate(LEDThread,
	                    &MyLEDThreadData[threadNb],
		            &MyLEDThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
			    LED_THREAD_PRIORITIES[threadNb]);
  }

  // Start multithreading - never returns!
  OS_Start();
}

/*!
** @}
*/
