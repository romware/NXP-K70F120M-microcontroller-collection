/* ###################################################################
**     Filename    : main.c
**     Project     : Lab2
**     Processor   : MK70FN1M0VMJ12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-04-12, 13:27, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**     Authors     : 12403756, 12551519
**
** ###################################################################*/
/*!
** @file main.c
** @version 2.0
** @brief
**         Main module.
**         This module contains user's application code.
** @author 12403756, 12551519
** @date 2018-04-13
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */

// Included header files
#include "types.h"
#include "Cpu.h"
#include "OS.h"
#include "Events.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "LEDs.h"
#include "MK70F12.h"
#include "UART.h"
#include "FIFO.h"
#include "packet.h"
#include "Flash.h"
#include "PE_Types.h"
#include "PIT.h"
#include "FTM.h"
#include "RTC.h"
#include "median.h"

// Analog functions
#include "analog.h"

#include <math.h>

#define BAUD_RATE 115200                        /*!< UART2 Baud Rate */

const uint32_t PERIOD_ANALOG_POLL = 1250000;    /*!< Period of the Analog polling (1 seconds) */

const uint8_t COMMAND_STARTUP     = 0x04;       /*!< The serial command byte for tower startup */
const uint8_t COMMAND_VER         = 0x09;       /*!< The serial command byte for tower version */
const uint8_t COMMAND_NUM         = 0x0B;       /*!< The serial command byte for tower number */
const uint8_t COMMAND_PROGRAMBYTE = 0x07;       /*!< The serial command byte for tower program byte */
const uint8_t COMMAND_READBYTE    = 0x08;       /*!< The serial command byte for tower read byte */
const uint8_t COMMAND_MODE        = 0x0D;       /*!< The serial command byte for tower mode */
const uint8_t COMMAND_TIME        = 0x0C;       /*!< The serial command byte for tower time */

const uint8_t COMMAND_TIMING      = 0x10;       /*!< The serial command byte for tower timing */
const uint8_t COMMAND_RAISES      = 0x11;       /*!< The serial command byte for tower raises */
const uint8_t COMMAND_LOWERS      = 0x12;       /*!< The serial command byte for tower lowers */
const uint8_t COMMAND_FREQUENCY   = 0x17;       /*!< The serial command byte for tower frequency */
const uint8_t COMMAND_VOLTAGE     = 0x18;       /*!< The serial command byte for tower voltage */
const uint8_t COMMAND_SPECTRUM    = 0x19;       /*!< The serial command byte for tower spectrum */

const uint8_t PARAM_GET           = 1;          /*!< Get bit of packet parameter 1 */
const uint8_t PARAM_SET           = 2;          /*!< Set bit of packet parameter 1 */

const uint8_t VRR_GET             = 0;          /*!< Get bit of packet parameter 1 */
const uint8_t VRR_DEFINITE        = 1;          /*!< Definite bit of packet parameter 1 */
const uint8_t VRR_INVERSE         = 2;          /*!< Inverse bit of packet parameter 1 */
const uint8_t VRR_RESET           = 1;          /*!< Reset bit of packet parameter 1 */
const uint8_t VRR_PHASE_A         = 1;          /*!< Phase A bit of packet parameter 1 */
const uint8_t VRR_PHASE_B         = 2;          /*!< Phase B bit of packet parameter 1 */
const uint8_t VRR_PHASE_C         = 3;          /*!< Phase C bit of packet parameter 1 */

const uint8_t TOWER_VER_MAJ       = 1;          /*!< Tower major version */
const uint8_t TOWER_VER_MIN       = 0;          /*!< Tower minor version */

volatile uint16union_t* NvTowerNb;              /*!< Tower number union pointer to flash */
volatile uint16union_t* NvTowerMd;              /*!< Tower mode union pointer to flash */

static OS_ECB* LEDOffSemaphore;                 /*!< LED off semaphore for FTM */
static OS_ECB* RTCReadSemaphore;                /*!< Read semaphore for RTC */

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);       /*!< The stack for the Tower Init thread. */
OS_THREAD_STACK(RTCThreadStack, THREAD_STACK_SIZE);               /*!< The stack for the RTC thread. */
OS_THREAD_STACK(FTMLEDsOffThreadStack, THREAD_STACK_SIZE);        /*!< The stack for the FTM thread. */
OS_THREAD_STACK(PITThreadStack, THREAD_STACK_SIZE);               /*!< The stack for the PIT thread. */
OS_THREAD_STACK(PacketThreadStack, THREAD_STACK_SIZE);            /*!< The stack for the Packet thread. */

const float VRR_VOLT = 3276.8;
const float VRR_LIMIT_HIGH = 9830.4;
const float VRR_LIMIT_LOW = 6553.6;
const float VRR_ALARM = 16384;

#define NB_ANALOG_CHANNELS 3

static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t RMSThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphoreRead;
  OS_ECB* semaphoreRMS;
  uint8_t channelNb;
  int16_t sampleData[16]; // TODO: 16 should be constant sample
  uint16_t sampleDataIndex;
} TAnalogThreadData;

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphoreRead = NULL,
    .semaphoreRMS = NULL,
    .channelNb = 0,
    .sampleDataIndex = 0
  },
  {
    .semaphoreRead = NULL,
    .semaphoreRMS = NULL,
    .channelNb = 1,
    .sampleDataIndex = 0
  },
  {
    .semaphoreRead = NULL,
    .semaphoreRMS = NULL,
    .channelNb = 2,
    .sampleDataIndex = 0
  }
};

/*! @brief Calculates the RMS value on an ADC channel.
 *
 */
void CalculateRMSThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

  for (;;)
  {
    (void)OS_SemaphoreWait(analogData->semaphoreRMS, 0);

    int64_t sum = 0;

    for(uint8_t i = 0; i < 16; i++) // TODO: 16 should be constant sample
    {
      sum += (analogData->sampleData[i])*(analogData->sampleData[i]);
    }

    float rms = sqrt((float)(sum/16));

    OS_DisableInterrupts();
    if(rms < VRR_LIMIT_LOW)
    {
      Analog_Put(2, VRR_ALARM); // TODO: alarm number should be constant
    }
    else if(rms > VRR_LIMIT_HIGH)
    {
      Analog_Put(2, VRR_ALARM);
    }
    else
    {
      Analog_Put(2, 0);
    }

    analogData->sampleDataIndex = 0;

    OS_EnableInterrupts();
  }
}

/*! @brief Samples a value on an ADC channel and sends it to the corresponding DAC channel.
 *
 */
void AnalogLoopbackThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

  for (;;)
  {
    int16_t analogInputValue;

    (void)OS_SemaphoreWait(analogData->semaphoreRead, 0);

    OS_DisableInterrupts();
    // Get analog sample
    Analog_Get(analogData->channelNb, &analogInputValue);
    OS_EnableInterrupts();

    if(analogData->sampleDataIndex == 16)
    {
      (void)OS_SemaphoreSignal(AnalogThreadData[analogData->channelNb].semaphoreRMS);
    }
    else
    {
      analogData->sampleData[analogData->sampleDataIndex] = analogInputValue;
      analogData->sampleDataIndex++;
    }

  }
}

/*! @brief Reads analog data every time it is called
 *
 *  @return void
 */
void PITCallback(void* arg)
{
  // Signal the analog channels to take a sample
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphoreRead);
}


/*! @brief Sends the startup packets to the PC
 *
 *  @return bool - TRUE if all packets are sent
 */
bool HandleTowerStartup(void)
{
  // Sends the tower startup values, version, number and mode to the PC
  return (
    Packet_Put(COMMAND_STARTUP,0x00,0x00,0x00) &&
    Packet_Put(COMMAND_VER,'v',TOWER_VER_MAJ,TOWER_VER_MIN) &&
    Packet_Put(COMMAND_NUM,PARAM_GET,NvTowerNb->s.Lo,NvTowerNb->s.Hi) &&
    Packet_Put(COMMAND_MODE,PARAM_GET,NvTowerMd->s.Lo,NvTowerMd->s.Hi)
  );
}

/*! @brief Sends the tower version packet to the PC
 *
 *  @return bool - TRUE if packet is sent
 */
bool HandleTowerVersion(void)
{
  // Sends the tower number packet
  return Packet_Put(COMMAND_NUM,PARAM_GET,NvTowerNb->s.Lo,NvTowerNb->s.Hi);
}

/*! @brief Sets the tower number or sends the packet to the PC
 *
 *  @return bool - TRUE if packet is sent or number is set
 */
bool HandleTowerNumber(void)
{
  // Check if parameters match tower number GET or SET parameters
  if(Packet_Parameter1 == PARAM_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the tower number packet
    return Packet_Put(COMMAND_NUM,PARAM_GET,NvTowerNb->s.Lo,NvTowerNb->s.Hi);
  }
  else if(Packet_Parameter1 == PARAM_SET)
  {
    // Sets the tower number
    return Flash_Write16((uint16_t*)NvTowerNb,(uint16_t)Packet_Parameter23);
  }
  return false;
}

/*! @brief Erases or programs a byte to Flash based on packet received
 *
 *  @return bool - TRUE if Flash was modified
 */
bool HandleTowerProgramByte(void)
{
  // Check if offset is erase or set
  if(Packet_Parameter1 == 0x08 && Packet_Parameter2 == 0)
  {
    // Erase Flash
    return Flash_Erase();
  }
  else if(Packet_Parameter1 >= 0x00 && Packet_Parameter1 <= 0x07 && Packet_Parameter2 == 0)
  {
    // Program byte to Flash
    volatile uint8_t* nvAddress = (uint8_t*)(FLASH_DATA_START + Packet_Parameter1);
    return Flash_Write8( (uint8_t*)nvAddress, Packet_Parameter3 );
  }
  return false;
}

/*! @brief Sends the byte read from Flash packet to the PC
 *
 *  @return bool - TRUE if packet is sent
 */
bool HandleTowerReadByte(void)
{
  // Check if offset is within range
  if(Packet_Parameter1 >= 0x00 && Packet_Parameter1 <= 0x07 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Send read byte packet
    return Packet_Put(COMMAND_READBYTE,Packet_Parameter1,0,_FB(FLASH_DATA_START + (uint32_t)Packet_Parameter1));
  }
  return false;
}

/*! @brief Sets the tower mode or sends the packet to the PC
 *
 *  @return bool - TRUE if packet is sent or mode is set
 */
bool HandleTowerMode(void)
{
  // Check if parameters match tower mode GET or SET parameters
  if(Packet_Parameter1 == PARAM_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the tower mode packet
    return Packet_Put(COMMAND_MODE,PARAM_GET,NvTowerMd->s.Lo,NvTowerMd->s.Hi);
  }
  else if (Packet_Parameter1 == PARAM_SET)
  {
    // Sets the tower mode
    return Flash_Write16((uint16_t*)NvTowerMd,(uint16_t)Packet_Parameter23);
  }
  return false;
}

/*! @brief Sets the Real Time Clock time as requested by PC
 *
 *  @return bool - TRUE if Real Clock Time is set successfully
 */
bool HandleTowerSetTime(void)
{
  // Check if parameters are within tower time limits
  if(Packet_Parameter1 < 24 && Packet_Parameter2 < 60 && Packet_Parameter3 < 60)
  {
      // Sets the Real Time Clock
      RTC_Set(Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
      return true;
  }
  return false;
}

/*! @brief Sets the tower timing mode or sends the packet to the PC
 *
 *  @return bool - TRUE if packet is sent or timing mode is set
 */
bool HandleTowerTiming(void)
{
  // Check if parameters match tower timing GET, DEFINITE or INVERSE parameters
  if(Packet_Parameter1 == VRR_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the tower timing mode packet
  }
  else if(Packet_Parameter1 == VRR_DEFINITE && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the tower timing mode packet
  }
  else if(Packet_Parameter1 == VRR_INVERSE && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the tower timing mode packet
  }
  return false;
}

/*! @brief Resets the number of raises or sends the packet to the PC
 *
 *  @return bool - TRUE if packet is sent or number of raises is reset
 */
bool HandleTowerRaises(void)
{
  // Check if parameters match tower raises GET or RESET parameters
  if(Packet_Parameter1 == VRR_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the number of raises packet
  }
  else if(Packet_Parameter1 == VRR_RESET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Resets the number of raises
  }
  return false;
}

/*! @brief Resets the number of lowers or sends the packet to the PC
 *
 *  @return bool - TRUE if packet is sent or number of lowers is reset
 */
bool HandleTowerLowers(void)
{
  // Check if parameters match tower lowers GET or RESET parameters
  if(Packet_Parameter1 == VRR_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the number of lowers packet
  }
  else if(Packet_Parameter1 == VRR_RESET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Resets the number of lowers
  }
  return false;
}

/*! @brief Sets the tower frequency packet to the PC
 *
 *  @return bool - TRUE if frequency is set
 */
bool HandleTowerFrequency(void)
{
  // Check if parameters frequency low byte and high byte is valid

  return false;
}

/*! @brief Sets the tower voltage
 *
 *  @return bool - TRUE if voltage is set
 */
bool HandleTowerVoltage(void)
{
  // Check half word

  // Check if parameters match phase A, B or C
  if(Packet_Parameter1 == VRR_PHASE_A)
  {
    // Sends the tower voltage packet
  }
  else if(Packet_Parameter1 == VRR_PHASE_B)
  {
    // Sends the tower voltage packet
  }
  else if(Packet_Parameter1 == VRR_PHASE_C)
  {
    // Sends the tower voltage packet
  }
  return false;
}

/*! @brief Sets the tower spectrum
 *
 *  @return bool - TRUE if spectrum is set
 */
bool HandleTowerSpectrum(void)
{
  // Check half word

  // Check if harmonic number is between 0 and 7
  if(Packet_Parameter1 >= 0 && Packet_Parameter1 <= 7)
  {
    // Sends the spectrum packet
  }

  return false;
}

/*! @brief Executes the command depending on what packet has been received
 *
 *  @return void
 */
void ReceivedPacket(void)
{
  bool success = false;                                         /*!< The success status of the received packet */
  uint8_t commandIgnoreAck = Packet_Command & ~PACKET_ACK_MASK; /*!< The command byte ignoring the ACK mask */
  uint8_t commandAck = Packet_Command & PACKET_ACK_MASK;        /*!< The command byte with the ACK mask */

  // AND the packet command byte with the bitwise inverse ACK MASK to ignore if ACK is requested
  if(commandIgnoreAck == COMMAND_STARTUP && Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Send tower startup packets
    success = HandleTowerStartup();
  }
  else if(commandIgnoreAck == COMMAND_VER && Packet_Parameter1 == 'v' && Packet_Parameter2 == 'x' && Packet_Parameter3 == 13)
  {
    // Send tower version packet
    success = HandleTowerVersion();
  }
  else if(commandIgnoreAck == COMMAND_NUM)
  {
    // Check if parameters match tower number GET or SET parameters
    success = HandleTowerNumber();
  }
  else if(commandIgnoreAck == COMMAND_MODE)
  {
    // Check if parameters match tower mode GET or SET parameters
    success = HandleTowerMode();
  }
  else if(commandIgnoreAck == COMMAND_PROGRAMBYTE)
  {
    // Send tower program byte packet
    success = HandleTowerProgramByte();
  }
  else if(commandIgnoreAck == COMMAND_READBYTE)
  {
    // Send tower read byte packet
    success = HandleTowerReadByte();
  }
  else if(commandIgnoreAck == COMMAND_TIME)
  {
    // Set the Real Time Clock time
    success = HandleTowerSetTime();
  }
  else if(commandIgnoreAck == COMMAND_TIMING)
  {
    // Check if parameters match tower timing GET, DEFINITE or INVERSE parameters
    success = HandleTowerTiming();
  }
  else if(commandIgnoreAck == COMMAND_RAISES)
  {
    // Check if parameters match number of raises GET or RESET parameters
    success = HandleTowerRaises();
  }
  else if(commandIgnoreAck == COMMAND_LOWERS)
  {
    // Check if parameters match number of lowers GET or RESET parameters
    success = HandleTowerLowers();
  }
  else if(commandIgnoreAck == COMMAND_FREQUENCY)
  {
    // Send the tower frequency packet
    success = HandleTowerFrequency();
  }
  else if(commandIgnoreAck == COMMAND_VOLTAGE)
  {
    // Send the tower voltage packet for phase A, B or C
    success = HandleTowerVoltage();
  }
  else if(commandIgnoreAck == COMMAND_SPECTRUM)
  {
    // Send the tower spectrum packet
    success = HandleTowerSpectrum();
  }

  // AND the packet command byte with the ACK MASK to check if ACK is requested
  if(commandAck)
  {
    // Check the success status of the packet which was sent
    if(success == false)
    {
      // Return the sent packet with the NACK command if unsuccessful
      Packet_Put(commandIgnoreAck,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    }
    else
    {
      // Return the sent packet with the ACK command if successful
      Packet_Put(Packet_Command,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    }
  }

  // Reset the packet variables to 0
  Packet_Command    = 0;
  Packet_Parameter1 = 0;
  Packet_Parameter2 = 0;
  Packet_Parameter3 = 0;
  Packet_Checksum   = 0;
}

/*! @brief Initializes the main tower components by calling the initialization routines of the supporting software modules.
 *
 *  @return void
 */
bool TowerInit(void)
{
  // Success status of writing default values to Flash and FTM
  bool success = false;

  if (Flash_Init() &&  LEDs_Init() && Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ) && FTM_Init() && RTC_Init(RTCReadSemaphore))
  {
    success = true;

    // Allocates an address in Flash memory to the tower number and tower mode
    if(Flash_AllocateVar((volatile void**)&NvTowerNb, sizeof(*NvTowerNb))
    && Flash_AllocateVar((volatile void**)&NvTowerMd, sizeof(*NvTowerMd)));
    {
      // Checks if tower number is clear
      if(_FH(NvTowerNb) == 0xFFFF)
      {
        // Sets the tower number to the default number
        if(!Flash_Write16((uint16_t*)NvTowerNb,(uint16_t)1519))
        {
          success = false;
        }
      }

      // Checks if tower mode is clear
      if(_FH(NvTowerMd) == 0xFFFF)
      {
        // Sets the tower mode to the default mode
        if(!Flash_Write16((uint16_t*)NvTowerMd,(uint16_t)1))
        {
          success = false;
        }
      }
    }
  }
  return success;
}

/*! @brief Initializes the modules
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note This thread deletes itself after running for the first time.
 */
static void InitModulesThread(void* pData)
{
  // Analog
  (void)Analog_Init(CPU_BUS_CLK_HZ);

  // Generate the global analog semaphores
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
  {
    AnalogThreadData[analogNb].semaphoreRead = OS_SemaphoreCreate(0);
    AnalogThreadData[analogNb].semaphoreRMS = OS_SemaphoreCreate(0);
  }

  //TODO:PIT_Init
  PIT_Init(CPU_BUS_CLK_HZ, PITCallback, NULL);

  // Create semaphores for threads
  LEDOffSemaphore = OS_SemaphoreCreate(0);
  RTCReadSemaphore = OS_SemaphoreCreate(0);

  // Initializes the main tower components and sets the default or stored values
  if(TowerInit())
  {
    // Turn on the orange LED to indicate the tower has initialized successfully
    LEDs_On(LED_ORANGE);
  }
  
  //TODO:PIT_Set
  PIT_Set(PERIOD_ANALOG_POLL, true);

  // Send startup packets to PC
  HandleTowerStartup();

  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

/*! @brief Checks if any packets have been received then handles it based on its contents
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note Assumes that Packet_Init has been called successfully.
 */
static void PacketThread(void* pData)
{
  TFTMChannel receivedPacketTmr;                   /*!< FTM Channel for received packet timer */
  receivedPacketTmr.channelNb                      = 0;
  receivedPacketTmr.delayCount                     = CPU_MCGFF_CLK_HZ_CONFIG_0;
  receivedPacketTmr.ioType.inputDetection          = TIMER_INPUT_ANY;
  receivedPacketTmr.ioType.outputAction            = TIMER_OUTPUT_DISCONNECT;
  receivedPacketTmr.timerFunction                  = TIMER_FUNCTION_OUTPUT_COMPARE;
  receivedPacketTmr.userSemaphore                  = LEDOffSemaphore;

  // Set FTM Channel for received packet timer
  FTM_Set(&receivedPacketTmr);

  for (;;)
  {
    // Check if a packet has been received
    if(Packet_Get())
    {
      // Turn on the blue LED if the 1 second timer is set
      if(FTM_StartTimer(&receivedPacketTmr))
      {
        LEDs_On(LED_BLUE);
      }

      // Execute a command depending on what packet has been received
      ReceivedPacket();
    }
  }
}

/*! @brief Sends the time from the real time clock to the PC
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note Assumes that RTC_Init has been called successfully.
 */
static void RTCThread(void* pData)
{
  for (;;)
  {
    // Wait for RTCRead semaphore
    OS_SemaphoreWait(RTCReadSemaphore,0);

    // Toggle the yellow LED
    LEDs_Toggle(LED_YELLOW);

    // Declare variable for hours, minutes seconds
    uint8_t hours, minutes, seconds;

    // Get the current time values
    RTC_Get(&hours, &minutes, &seconds);

    // Send time to PC
    Packet_Put(COMMAND_TIME, hours, minutes, seconds);
  }
}

/*! @brief Turns the Blue LED off after the timer is complete
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note Assumes that FTM_Init has been called successfully.
 */
static void FTMLEDsOffThread(void* pData)
{
  for (;;)
  {
      // Wait until signaled to turn LED off
     OS_SemaphoreWait(LEDOffSemaphore,0);

     // Turn off blue LED
     LEDs_Off(LED_BLUE);
  }
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */
  OS_ERROR error;

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  // Initialize the RTOS
  OS_Init(CPU_CORE_CLK_HZ, false);

  // Highest priority
  error = OS_ThreadCreate(InitModulesThread,
                          NULL,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
  		                    0);

  uint8_t analogPriority = 3;

  // Create threads for analog loopback channels
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(AnalogLoopbackThread,
                            &AnalogThreadData[threadNb],
                            &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            analogPriority);
    analogPriority++;
  }

  // Create threads for analog RMS channels
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(CalculateRMSThread,
                            &AnalogThreadData[threadNb],
                            &RMSThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            analogPriority);
    analogPriority++;
  }

  // 3rd Highest priority
  error = OS_ThreadCreate(PacketThread,
                          NULL,
                          &PacketThreadStack[THREAD_STACK_SIZE - 1],
                          9);
  // 4th Highest priority
  error = OS_ThreadCreate(FTMLEDsOffThread,
                          NULL,
                          &FTMLEDsOffThreadStack[THREAD_STACK_SIZE - 1],
                          10);
  // 5th Highest priority
  error = OS_ThreadCreate(RTCThread,
                          NULL,
                          &RTCThreadStack[THREAD_STACK_SIZE - 1],
                          11);

  // Start multithreading - never returns!
  OS_Start();
}

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
