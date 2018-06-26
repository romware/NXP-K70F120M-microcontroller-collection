/* ###################################################################
**     Filename    : main.c
**     Project     : VRR
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
**     Authors     : 12551519
**
** ###################################################################*/
/*!
** @file main.c
** @version 2.0
** @brief
**         Main module.
**         This module contains user's application code.
** @author 12551519
** @date 2018-05-13
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
#include "MK70F12.h"
#include "LEDs.h"
#include "UART.h"
#include "FIFO.h"
#include "packet.h"
#include "Flash.h"
#include "PIT.h"
#include "FTM.h"
#include "VRR.h"
#include "kiss_fft.h"
#include "kiss_fftr.h"

// Analog functions
#include "analog.h"

#include <math.h>

#define BAUD_RATE 115200                                     /*!< UART2 Baud Rate */

const uint8_t COMMAND_TIMING       =       0x10;             /*!< The serial command byte for tower timing */
const uint8_t COMMAND_RAISES       =       0x11;             /*!< The serial command byte for tower raises */
const uint8_t COMMAND_LOWERS       =       0x12;             /*!< The serial command byte for tower lowers */
const uint8_t COMMAND_FREQUENCY    =       0x17;             /*!< The serial command byte for tower frequency */
const uint8_t COMMAND_VOLTAGE      =       0x18;             /*!< The serial command byte for tower voltage */
const uint8_t COMMAND_SPECTRUM     =       0x19;             /*!< The serial command byte for tower spectrum */

static uint32_t PeriodAnalogSample =    1250000;             /*!< Period of analog polling (16 samples per cycle, 50 Hz default) */
static uint32_t Frequency          =        500;             /*!< The frequency of the VRR (Default 50Hz) */
static uint32_t Spectrum[8];                                 /*!< The spectrum harmonics of the VRR */
static uint8_t TimingMode          =          1;             /*!< The timing mode of the VRR */

volatile uint8_t* NvCountRaises;                             /*!< Number of raises pointer to flash */
volatile uint8_t* NvCountLowers;                             /*!< Number of lowers pointer to flash */
volatile uint8_t* NvTimingMode;                              /*!< Timing Mode Setting pointer to flash */

static OS_ECB* LEDOffSemaphore;                              /*!< LED off semaphore for FTM */
static OS_ECB* RaisesSemaphore;                              /*!< Raises semaphore for Flash */
static OS_ECB* LowersSemaphore;                              /*!< Lowers semaphore for Flash */
static OS_ECB* FrequencySemaphore;                           /*!< Frequency semaphore for VRR */
static OS_ECB* SpectrumSemaphore;                            /*!< Spectrum semaphore for VRR */

static OS_ECB* FlashAccessSemaphore;                         /*!< Mutex semaphore for Flash */
static OS_ECB* RMSAccessSemaphore;                           /*!< Mutex semaphore for RMS */
static OS_ECB* FrequencyAccessSemaphore;                     /*!< Mutex semaphore for Frequency */
static OS_ECB* SpectrumAccessSemaphore;                      /*!< Mutex semaphore for Spectrum */

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);  /*!< The stack for the Initial thread. */
OS_THREAD_STACK(LEDsOffThreadStack, THREAD_STACK_SIZE);      /*!< The stack for the FTM LED thread. */
OS_THREAD_STACK(PacketThreadStack, THREAD_STACK_SIZE);       /*!< The stack for the Packet thread. */
OS_THREAD_STACK(RaisesThreadStack, THREAD_STACK_SIZE);       /*!< The stack for the Flash Raises thread. */
OS_THREAD_STACK(LowersThreadStack, THREAD_STACK_SIZE);       /*!< The stack for the Flash Lowers thread. */
OS_THREAD_STACK(FrequencyThreadStack, THREAD_STACK_SIZE);    /*!< The stack for the Frequency thread. */
OS_THREAD_STACK(SpectrumThreadStack, THREAD_STACK_SIZE);     /*!< The stack for the Spectrum thread. */

static uint32_t RMSThreadStacks[VRR_NB_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[VRR_NB_CHANNELS] =
{
  {
    .semaphoreRMS = NULL,
    .sampleDataIndex = 0,
    .channelNb = VRR_CHANNEL_1
  },
  {
    .semaphoreRMS = NULL,
    .sampleDataIndex = 0,
    .channelNb = VRR_CHANNEL_2
  },
  {
    .semaphoreRMS = NULL,
    .sampleDataIndex = 0,
    .channelNb = VRR_CHANNEL_3
  }
};

/*! @brief Sets the tower timing mode or sends the packet to the PC
 *
 *  @return bool - TRUE if packet is sent or timing mode is set
 */
static bool HandleTowerTiming(void)
{
  // Check if parameters match timing GET, DEFINITE or INVERSE parameters
  if(Packet_Parameter1 == TIMING_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the timing mode packet
    return Packet_Put(COMMAND_TIMING,TIMING_GET,TimingMode,0);
  }
  else if((Packet_Parameter1 == TIMING_DEFINITE || Packet_Parameter1 == TIMING_INVERSE) && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sets the timing mode
    TimingMode = Packet_Parameter1;
    return Flash_Write((uint32_t*)NvTimingMode,Packet_Parameter1,8);
  }
  return false;
}

/*! @brief Resets the number of raises or sends the number to the PC
 *
 *  @return bool - TRUE if packet is sent or number of raises is reset
 */
static bool HandleTowerRaises(void)
{
  // Check if parameters match tower raises GET or RESET parameters
  if(Packet_Parameter1 == DEVIATION_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the number of raises packet
    return Packet_Put(COMMAND_RAISES,DEVIATION_GET,Flash_Read8(NvCountRaises),0);
  }
  else if(Packet_Parameter1 == DEVIATION_RESET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Resets the number of raises to flash
    return Flash_Write((uint32_t*)NvCountRaises,0,8);
  }
  return false;
}

/*! @brief Resets the number of lowers or sends the number to the PC
 *
 *  @return bool - TRUE if packet is sent or number of lowers is reset
 */
static bool HandleTowerLowers(void)
{
  // Check if parameters match tower raises GET or RESET parameters
  if(Packet_Parameter1 == DEVIATION_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the number of raises packet
    return Packet_Put(COMMAND_LOWERS,DEVIATION_GET,Flash_Read8(NvCountLowers),0);
  }
  else if(Packet_Parameter1 == DEVIATION_RESET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Resets the number of lowers to flash
    return Flash_Write((uint32_t*)NvCountLowers,0,8);
  }
  return false;
}

/*! @brief Sends the tower frequency packet for a selected phase
 *
 *  @return bool - TRUE if frequency packet is sent
 */
static bool HandleTowerFrequency(void)
{
  int16union_t frequency;

  // Selects the frequency based on selected phase
  (void)OS_SemaphoreWait(FrequencyAccessSemaphore, 0);
  frequency.l = (int16_t)Frequency;
  (void)OS_SemaphoreSignal(FrequencyAccessSemaphore);

  // Sends the frequency packet
  return Packet_Put(COMMAND_FREQUENCY,frequency.s.Lo,frequency.s.Hi,0);
}

/*! @brief Sends the tower voltage packet for a selected phase
 *
 *  @return bool - TRUE if voltage packet is sent
 */
static bool HandleTowerVoltage(void)
{
  if(Packet_Parameter1 > VRR_CHANNEL_1 && Packet_Parameter1 <= VRR_NB_CHANNELS && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    int16union_t rms;
    TAnalogThreadData* phase = &AnalogThreadData[Packet_Parameter1-1];

    // Selects the RMS based on received phase
    (void)OS_SemaphoreWait(RMSAccessSemaphore, 0);
    rms.l = phase->currentRMS;
    (void)OS_SemaphoreSignal(RMSAccessSemaphore);

    // Sends the RMS voltage packet
    return Packet_Put(COMMAND_VOLTAGE,Packet_Parameter1,rms.s.Lo,rms.s.Hi);
  }
  return false;
}

/*! @brief Sends the tower spectrum packet for a selected harmonic
 *
 *  @note Tested speed of Math.h sqrt and float FFT speed - 242us whereas this is 26us. Therefore on average 216us faster
 *  @return bool - TRUE if spectrum packet is sent
 */
static bool HandleTowerSpectrum(void)
{
  // Check if harmonic number is between 0 and 7
  if(Packet_Parameter1 >= 0 && Packet_Parameter1 <= 7)
  {
    uint16union_t magnitude;

    // Selects the magnitude based on received harmonic
    (void)OS_SemaphoreWait(SpectrumAccessSemaphore,0);
    magnitude.l = Spectrum[Packet_Parameter1];
    (void)OS_SemaphoreSignal(SpectrumAccessSemaphore);

    // Sends the harmonic magnitude packet
    return Packet_Put(COMMAND_SPECTRUM,Packet_Parameter1,magnitude.s.Lo,magnitude.s.Hi);
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
  if(commandIgnoreAck == COMMAND_TIMING)
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

/*! @brief Reads analog data from the ADC on a specific channel
 *
 *  @return void
 */
void AnalogSampleCallback(void* arg)
{
  static uint8_t channel;
  int16_t analogInputValue;
  TAnalogThreadData* channelData = &AnalogThreadData[channel];

  // Get the voltage value of the selected channel
  Analog_Get(channelData->channelNb, &analogInputValue);

  // Store the voltage value in an array
  channelData->sampleData[channelData->sampleDataIndex] = analogInputValue;
  channelData->sampleDataIndex++;
  if(channelData->sampleDataIndex >= VRR_SAMPLE_SIZE)
  {
    channelData->sampleDataIndex = 0;
  }

  // Signal to calculate RMS every sample
  (void)OS_SemaphoreSignal(AnalogThreadData[channel].semaphoreRMS);

  // Increment selected channel
  channel++;
  if(channel >= VRR_NB_CHANNELS)
  {
    channel = 0;
  }

  // Signal to calculate Frequency and Spectrum every cycle
  if(channel == 0 && channelData->sampleDataIndex == 0)
  {
    (void)OS_SemaphoreSignal(FrequencySemaphore);
    (void)OS_SemaphoreSignal(SpectrumSemaphore);
  }
}

/*! @brief Initializes the main tower components by calling the initialization routines of the supporting software modules.
 *
 *  @return void
 */
bool TowerInit(void)
{
  // Success status of writing default values to Flash and FTM
  bool success = false;

  if(Flash_Init(FlashAccessSemaphore)
  && LEDs_Init()
  && Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ)
  && FTM_Init()
  && VRR_Init()
  && PIT_Init(CPU_BUS_CLK_HZ, AnalogSampleCallback, NULL))
  {
    success = true;

    // Allocates addresses in Flash memory
    if(Flash_AllocateVar((volatile void**)&NvCountRaises, sizeof(*NvCountRaises))
    && Flash_AllocateVar((volatile void**)&NvCountLowers, sizeof(*NvCountLowers))
    && Flash_AllocateVar((volatile void**)&NvTimingMode, sizeof(*NvTimingMode)));
    {
      // Checks if timing mode is clear
      if(Flash_Read8(NvTimingMode) == 0xFF)
      {
        // Sets the tower mode to the default number
        if(!Flash_Write((uint32_t*)NvTimingMode,(uint8_t)TIMING_DEFINITE,8))
        {
          success = false;
        }
      }
      // Checks if number of raises is clear
      if(Flash_Read8(NvCountRaises) == 0xFF)
      {
        // Sets the number of raises to the default number
        if(!Flash_Write((uint32_t*)NvCountRaises,(uint8_t)0,8))
        {
          success = false;
        }
      }
      // Checks if number of lowers is clear
      if(Flash_Read8(NvCountLowers) == 0xFF)
      {
        // Sets the number of lowers to the default number
        if(!Flash_Write((uint32_t*)NvCountLowers,(uint8_t)0,8))
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
  OS_DisableInterrupts();

  // Initialize the analog library
  (void)Analog_Init(CPU_BUS_CLK_HZ);

  // Generate the global analog semaphores
  for (uint8_t analogNb = VRR_CHANNEL_1; analogNb < VRR_NB_CHANNELS; analogNb++)
  {
    AnalogThreadData[analogNb].semaphoreRMS = OS_SemaphoreCreate(0);
  }

  // Create semaphores for mutual exclusions
  FlashAccessSemaphore      = OS_SemaphoreCreate(1);
  RMSAccessSemaphore        = OS_SemaphoreCreate(1);
  FrequencyAccessSemaphore  = OS_SemaphoreCreate(1);
  SpectrumAccessSemaphore   = OS_SemaphoreCreate(1);

  // Create semaphores for threads
  LEDOffSemaphore           = OS_SemaphoreCreate(0);
  RaisesSemaphore           = OS_SemaphoreCreate(0);
  LowersSemaphore           = OS_SemaphoreCreate(0);
  FrequencySemaphore        = OS_SemaphoreCreate(0);
  SpectrumSemaphore         = OS_SemaphoreCreate(0);

  // Initializes the main tower components and sets the default or stored values
  if(TowerInit())
  {
    // Turn on the orange LED to indicate the tower has initialized successfully
    LEDs_On(LED_ORANGE);
  }
  
  // Set the PIT with the analog polling period
  PIT_Set(PeriodAnalogSample / VRR_NB_CHANNELS, true);

  // Put initial zero values to tower output channels
  VRR_ProtectedAnalogPut(VRR_CHANNEL_1, VRR_OUT_ZERO);
  VRR_ProtectedAnalogPut(VRR_CHANNEL_2, VRR_OUT_ZERO);
  VRR_ProtectedAnalogPut(VRR_CHANNEL_3, VRR_OUT_ZERO);

  OS_EnableInterrupts();

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
  TFTMChannel receivedPacketTmr;          /*!< FTM Channel for received packet timer */
  receivedPacketTmr.channelNb             = 0;
  receivedPacketTmr.delayCount            = CPU_MCGFF_CLK_HZ_CONFIG_0;
  receivedPacketTmr.ioType.inputDetection = TIMER_INPUT_ANY;
  receivedPacketTmr.ioType.outputAction   = TIMER_OUTPUT_DISCONNECT;
  receivedPacketTmr.timerFunction         = TIMER_FUNCTION_OUTPUT_COMPARE;
  receivedPacketTmr.userSemaphore         = LEDOffSemaphore;

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

/*! @brief Turns the Blue LED off after the timer is complete
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note Assumes that FTM_Init has been called successfully.
 */
static void LEDsOffThread(void* pData)
{
  for (;;)
  {
      // Wait until signaled to turn LED off
     OS_SemaphoreWait(LEDOffSemaphore,0);

     // Turn off blue LED
     LEDs_Off(LED_BLUE);
  }
}


/*! @brief Calculates the RMS value on an ADC channel
 *
 *  @param pData is not used to store the channels data.
 *  @note Assumes that Analog_Init has been called successfully.
 */
void CalculateRMSThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

  static bool allAlarms[VRR_NB_CHANNELS];
  static bool adjustment;

  int64_t timerDelay;
  int64_t minDelay;
  int64_t timerRate;
  bool alarm;

  for (;;)
  {
    (void)OS_SemaphoreWait(analogData->semaphoreRMS, 0);

    // Protect access for frequency reading
    (void)OS_SemaphoreWait(FrequencyAccessSemaphore, 0);
    uint32_t analogPoll = PeriodAnalogSample;
    (void)OS_SemaphoreSignal(FrequencyAccessSemaphore);

    // Protect access for RMS reading
    (void)OS_SemaphoreWait(RMSAccessSemaphore, 0);
    uint16_t prevRoot = analogData->currentRMS;
    (void)OS_SemaphoreSignal(RMSAccessSemaphore);

    // Protect access for sample data reading
    OS_DisableInterrupts();
    int16_t sampleData[VRR_SAMPLE_SIZE*2];
    for(uint8_t i = 0; i < VRR_SAMPLE_SIZE; i++)
    {
       sampleData[i] = analogData->sampleData[i];
    }
    OS_EnableInterrupts();

    // Calculate the RMS based on sample data
    uint16_t rms = VRR_CalculateRMS(sampleData, VRR_SAMPLE_SIZE, prevRoot);

    // Protect access for RMS writing
    (void)OS_SemaphoreWait(RMSAccessSemaphore, 0);
    analogData->currentRMS = rms;
    (void)OS_SemaphoreSignal(RMSAccessSemaphore);

    // Check if the RMS is within boundary
    if(rms < VRR_VOLT_LIMIT_LOW)
    {
      // Handle the alarm and adjustments for raising
      VRR_CheckAlarm(&timerDelay, &minDelay, &timerRate, analogPoll, TimingMode, (VRR_VOLT_NOMINAL-rms), &alarm, &adjustment, RaisesSemaphore, VRR_CHANNEL_1);
    }
    else if(rms > VRR_VOLT_LIMIT_HIGH)
    {
      // Handle the alarm and adjustments for lowering
      VRR_CheckAlarm(&timerDelay, &minDelay, &timerRate, analogPoll, TimingMode, (rms-VRR_VOLT_NOMINAL), &alarm, &adjustment, LowersSemaphore, VRR_CHANNEL_2);
    }
    else
    {
      // No alarm on this channel
      alarm = false;
    }

    // Update status on this channels alarm
    allAlarms[analogData->channelNb] = alarm;

    // Turn off alarm and adjustment signals if all channels are within boundaries
    if(!VRR_ArrayAnyTrue(allAlarms, VRR_NB_CHANNELS))
    {
      adjustment = false;
      VRR_ProtectedAnalogPut(VRR_CHANNEL_1, VRR_OUT_ZERO);
      VRR_ProtectedAnalogPut(VRR_CHANNEL_2, VRR_OUT_ZERO);
      VRR_ProtectedAnalogPut(VRR_CHANNEL_3, VRR_OUT_ZERO);
    }
  }
}

/*! @brief Calculates the Frequency value on an ADC channel
 *
 *  @param pData is not used.
 *  @note Assumes that Analog_Init has been called successfully.
 */
void CalculateFrequencyThread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(FrequencySemaphore, 0);

    // Protect access for frequency reading
    (void)OS_SemaphoreWait(FrequencyAccessSemaphore, 0);
    uint32_t analogPoll = PeriodAnalogSample;
    (void)OS_SemaphoreSignal(FrequencyAccessSemaphore);

    // Protect access for sample data reading
    OS_DisableInterrupts();
    TAnalogThreadData* channelData = &AnalogThreadData[VRR_CHANNEL_1];
    int16_t sampleData[VRR_SAMPLE_SIZE*2];
    for(uint8_t i = 0; i < VRR_SAMPLE_SIZE*2; i++)
    {
      // Copy array locally from the two most recent samples
      if(i < VRR_SAMPLE_SIZE)
      {
        sampleData[i] = channelData->prevSampleData[i];
      }
      else
      {
        sampleData[i] = channelData->sampleData[i - VRR_SAMPLE_SIZE];
      }
    }
    VRR_ArrayCopy(channelData->sampleData, channelData->prevSampleData, VRR_SAMPLE_SIZE);
    OS_EnableInterrupts();

    // Protect access for RMS reading
    (void)OS_SemaphoreWait(RMSAccessSemaphore, 0);
    uint16_t rms = channelData->currentRMS;
    (void)OS_SemaphoreSignal(RMSAccessSemaphore);

    // Calculate the frequency of this channel
    uint32_t frequency = VRR_CalculateFrequency(sampleData,rms,analogPoll);
    if(frequency)
    {
      // Set the frequency and polling period based on calculated frequency
      VRR_SetFrequency(FrequencyAccessSemaphore,&Frequency,frequency,rms,&PeriodAnalogSample);
    }
  }
}

/*! @brief Calculates the Spectrum on an ADC channel
 *
 *  @param pData is not used.
 *  @note Assumes that Analog_Init has been called successfully.
 */
void CalculateSpectrumThread(void* pData)
{
  // Alocate memory for FFT library configuration
  uint8_t memory[512];
  size_t length = 512;
  kiss_fftr_cfg config = kiss_fftr_alloc(VRR_SAMPLE_SIZE, 0, memory, &length);

  // Keep track of stored harmonic
  static uint8_t harmonic = 0;

  for (;;)
  {
    (void)OS_SemaphoreWait(SpectrumSemaphore, 0);


    kiss_fft_scalar timedata[VRR_SAMPLE_SIZE];

    // Protect access for sample data reading
    OS_DisableInterrupts();
    TAnalogThreadData* channelData = &AnalogThreadData[VRR_CHANNEL_1];
    for(uint8_t i = 0; i < VRR_SAMPLE_SIZE; i++)
    {
      timedata[i] = channelData->prevSampleData[i];
    }
    OS_EnableInterrupts();

    // Calculate spectrum
    kiss_fft_cpx spectrum[(VRR_SAMPLE_SIZE/2)+1];

    // Real Fast Fourier Transform calculation
    kiss_fftr(config, timedata, spectrum);

    // Protect access for spectrum reading/writing
    (void)OS_SemaphoreWait(SpectrumAccessSemaphore,0);

    // Retrieve previously calculated roots for increased speed
    uint32_t prevRoot = Spectrum[harmonic];
    uint32_t prevSquare = prevRoot * prevRoot;

    // Calculate the target square
    int64_t real = spectrum[harmonic].r / (VRR_SAMPLE_SIZE/2);
    int64_t imaginary = spectrum[harmonic].i / (VRR_SAMPLE_SIZE/2);
    int64_t targetSquare = (real * real) + (imaginary * imaginary);

    // Calculate the square root efficiently of selected harmonic
    Spectrum[harmonic] = VRR_QuickSquareRoot(targetSquare,prevRoot,prevSquare,VRR_OUT_ZERO,VRR_OUT_FIVE);

    // Increment harmonic
    harmonic++;
    if(harmonic >= 8)
    {
      harmonic = 0;
    }

    (void)OS_SemaphoreSignal(SpectrumAccessSemaphore);
  }
}

/*! @brief Increments the raises count and writes to Flash.
 *
 *  @param pData is not used.
 *  @note Assumes that Flash_Init has been called successfully.
 */
void RaisesThread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(RaisesSemaphore, 0);
    Flash_Write((uint32_t*)NvCountRaises,Flash_Read8(NvCountRaises)+1,8);
  }
}

/*! @brief Increments the lowers count and writes to Flash.
 *
 *  @param pData is not used.
 *  @note Assumes that Flash_Init has been called successfully.
 */
void LowersThread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(LowersSemaphore, 0);
    Flash_Write((uint32_t*)NvCountLowers,Flash_Read8(NvCountLowers)+1,8);
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

  // Create threads for analog RMS calculations
  for (uint8_t threadNb = VRR_CHANNEL_1; threadNb < VRR_NB_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(CalculateRMSThread,
                            &AnalogThreadData[threadNb],
                            &RMSThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            3+threadNb);
  }

  // 6th Highest priority
  error = OS_ThreadCreate(CalculateFrequencyThread,
                          NULL,
                          &FrequencyThreadStack[THREAD_STACK_SIZE - 1],
                          6);
  // 7th Highest priority
  error = OS_ThreadCreate(CalculateSpectrumThread,
                          NULL,
                          &SpectrumThreadStack[THREAD_STACK_SIZE - 1],
                          7);
  // 8th Highest priority
  error = OS_ThreadCreate(PacketThread,
                          NULL,
                          &PacketThreadStack[THREAD_STACK_SIZE - 1],
                          8);
  // 9th Highest priority
  error = OS_ThreadCreate(LEDsOffThread,
                          NULL,
                          &LEDsOffThreadStack[THREAD_STACK_SIZE - 1],
                          9);
  // 10th Highest priority
  error = OS_ThreadCreate(RaisesThread,
                          NULL,
                          &RaisesThreadStack[THREAD_STACK_SIZE - 1],
                          10);
  // 11th Highest priority
  error = OS_ThreadCreate(LowersThread,
                          NULL,
                          &LowersThreadStack[THREAD_STACK_SIZE - 1],
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
