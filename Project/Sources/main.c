/* ###################################################################
**     Filename    : main.c
**     Project     : VRR Project
**     Processor   : MK70FN1M0VMJ12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-06-17, 13:27, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**     Authors     : 12403756
**
** ###################################################################*/
/*!
** @file main.c
** @version 1.0
** @brief
**         Main module.
**         This module contains user's application code.
** @author 12403756
** @date 2018-06-17
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
#include "accel.h"
#include "I2C.h"
#include "median.h"
#include "analog.h"
#include "math.h"
#include "kiss_fft.h"
#include "kiss_fftr.h"


#define BAUD_RATE 115200                        /*!< UART2 Baud Rate */
#define ADC_SAMPLES_PER_CYCLE 16                /*!< ADC samples per cycle */
#define ADC_BUFFER_SIZE 64                      /*!< ADC buffer size Note: Must be an integral number of ADC_SAMPLES_PER_CYCLE*/
#define NB_ANALOG_CHANNELS 3                    /*!< Number of analog channels */
#define ADC_DEFAULT_FREQUENCY 50                /*!< ADC default frequency */
#define RMS_UPPER_LIMIT 9830                    /*!< VRR RMS upper limit */
#define RMS_LOWER_LIMIT 6554                    /*!< VRR RMS lower limit */
#define RMS_FREQUENCY_MIN 4915                  /*!< VRR RMS frequency measure minimum level */
#define DAC_5V_OUT 16384                        /*!< DAC 5 volts out */
#define DAC_0V_OUT 0                            /*!< DAC 0 volts out */
#define ADC_HALF_VOLT 1638                      /*!< ADC Half Volt */
#define ALARM_TIMER 5000000000                  /*!< Alarm Timer in nanoSeconds */
#define ALARM_TIMER_MIN 1000000000              /*!< Minimum Alarm Timer in nanoSeconds */

/*! @brief Data structure used to hold ADC voltage samples
 *
 */
typedef struct
{
  int16_t ADC_Data[ADC_BUFFER_SIZE];
  uint8_t LatestData;
}TVoltageData;

/*! @brief Data structure for enumerated timing parameters
 *
 */
typedef enum
{
  TIMING_GET,
  TIMING_DEFINITE,
  TIMING_INVERSE
}TTimingMode;

/*! @brief Data structure for enumerated output
 *
 */
typedef enum
{
  OUTPUT_DEBUG,
  OUTPUT_RAISE,
  OUTPUT_LOWER,
  OUTPUT_ALARM
}TOutputMode;

/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
} TAnalogThreadData;

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphore = NULL,
    .channelNb = 0
  },
  {
    .semaphore = NULL,
    .channelNb = 1
  },
  {
    .semaphore = NULL,
    .channelNb = 2
  }
};

// Thread priorities for analog threads
const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {7,8,9}; /*!< The array of analog thread priorities */

// I2C Poll period
const uint32_t PERIOD_I2C_POLL    = 1000000000;                       /*!< Period of the I2C polling in polling mode */

// PC to Tower Commands
const uint8_t COMMAND_STARTUP     = 0x04;                             /*!< The serial command byte for tower startup */
const uint8_t COMMAND_VER         = 0x09;                             /*!< The serial command byte for tower version */
const uint8_t COMMAND_NUM         = 0x0B;                             /*!< The serial command byte for tower number */
const uint8_t COMMAND_PROGRAMBYTE = 0x07;                             /*!< The serial command byte for tower program byte */
const uint8_t COMMAND_READBYTE    = 0x08;                             /*!< The serial command byte for tower read byte */
const uint8_t COMMAND_MODE        = 0x0D;                             /*!< The serial command byte for tower mode */
const uint8_t COMMAND_TIME        = 0x0C;                             /*!< The serial command byte for tower time */
const uint8_t COMMAND_ACCEL       = 0x10;                             /*!< The serial command byte for tower accelerometer */
const uint8_t COMMAND_TIMINGMODE  = 0x10;                             /*!< The serial command byte for tower timing mode */
const uint8_t COMMAND_RAISES      = 0x11;                             /*!< The serial command byte for tower raises */
const uint8_t COMMAND_LOWERS      = 0x12;                             /*!< The serial command byte for tower lowers */
const uint8_t COMMAND_FREQUENCY   = 0x17;                             /*!< The serial command byte for tower frequency */
const uint8_t COMMAND_VOLTAGE     = 0x18;                             /*!< The serial command byte for tower voltage */
const uint8_t COMMAND_SPECTRUM    = 0x19;                             /*!< The serial command byte for tower spectrum */

// Tower packet parameters
const uint8_t PARAM_GET           = 1;                                /*!< Get bit of packet parameter 1 */
const uint8_t PARAM_SET           = 2;                                /*!< Set bit of packet parameter 1 */

// Tower version numbers
const uint8_t TOWER_VER_MAJ       = 1;                                /*!< Tower major version */
const uint8_t TOWER_VER_MIN       = 0;                                /*!< Tower minor version */

// Pointers to non-volatile flash memory
volatile uint16union_t* NvTowerNb;                                    /*!< Tower number union pointer to flash */
volatile uint16union_t* NvTowerMd;                                    /*!< Tower mode union pointer to flash */
volatile uint8_t* NvNbRaises;                                         /*!< Raise count byte pointer to flash */
volatile uint8_t* NvNbLowers;                                         /*!< Lower count byte pointer to flash */
volatile uint8_t* NvTimingMd;                                         /*!< Timing Mode byte pointer to flash */

static TVoltageData VoltageSamples[NB_ANALOG_CHANNELS];               /*!< The array of structs for voltage samples */
static uint16_t RMS[NB_ANALOG_CHANNELS];                              /*!< The RMS of each analog channel */
static uint16_t TimingMode;                                           /*!< The timing mode for the VRR*/
static uint32_t SamplePeriod;                                         /*!< The current sample period of the VRR*/
static uint32_t NewSamplePeriod;                                      /*!< The period to be loaded into the PIT next */
static int64_t LastSumOfSquares[NB_ANALOG_CHANNELS];                  /*!< The array of the last sum of squares for fast RMS calculation */
static int16_t OldestData[NB_ANALOG_CHANNELS];                        /*!< The array of the oldest ADC sample to be removed from sum of squares */
static bool Alarm[NB_ANALOG_CHANNELS];                                /*!< The array of alarm flags indicating individual thread alarm states */
static bool Adjusting[NB_ANALOG_CHANNELS];                            /*!< The array of adjusting flags indicating individual thread adjusting states */
static uint16_t Frequency;                                               /*!< The frequency of phase A */
kiss_fft_cpx FFTOutput[(ADC_SAMPLES_PER_CYCLE / 2) + 1];              /*!< Complex array for output data */

// Semaphores for use by RTOS
static OS_ECB* LEDOffSemaphore;                                       /*!< LED off semaphore for FTM */
static OS_ECB* RTCReadSemaphore;                                      /*!< Read semaphore for RTC */
static OS_ECB* OutOfRangeSemaphore;                                   /*!< Out of range semaphore for RMS  */
static OS_ECB* WithinRangeSemaphore;                                  /*!< Within range semaphore for RMS  */
static OS_ECB* NewADCDataSemaphore;                                   /*!< New ADC data semaphore for ADC Process thread */
static OS_ECB* FrequencyCalculateSemaphore;                           /*!< Frequency Calculate semaphore */
static OS_ECB* FrequencyTrackSemaphore;                               /*!< Frequency track semaphore */
static OS_ECB* LogRaisesSemaphore;                                    /*!< Log Raises semaphore */
static OS_ECB* LogLowersSemaphore;                                    /*!< Log Lowers semaphore */
static OS_ECB* FlashAccessMutex;                                      /*!< Flash Access Mutex */
static OS_ECB* FFTSemaphore;                                          /*!< FFT semaphore semaphore */
static OS_ECB* FFTOutputMutex;                                        /*!< FFTOutput Access Mutex */
static OS_ECB* AlarmMutex;                                            /*!< Alarm Flag Access Mutex */
static OS_ECB* AdjustingMutex;                                        /*!< Adjusting Flag Access Mutex */
static OS_ECB* RMSCalculationDataMutex;                               /*!< RMS Calculation Data Access Mutex */
static OS_ECB* SamplePeriodMutex;                                     /*!< Sample Period Access Mutex */
static OS_ECB* NewSamplePeriodMutex;                                  /*!< New Sample Period Access Mutex */
static OS_ECB* FrequencyMutex;                                        /*!< Frequency Access Mutex */
static OS_ECB* RMSMutex;                                              /*!< RMS Access Mutex */
static OS_ECB* TimingModeMutex;                                       /*!< Timing Mode Access Mutex */

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);           /*!< The stack for the Tower Init thread. */
OS_THREAD_STACK(RTCThreadStack, THREAD_STACK_SIZE);                   /*!< The stack for the RTC thread. */
OS_THREAD_STACK(FTMLEDsOffThreadStack, THREAD_STACK_SIZE);            /*!< The stack for the FTM thread. */
OS_THREAD_STACK(ADCDataProcessThreadStack, THREAD_STACK_SIZE);        /*!< The stack for the ADCDataProcess thread. */
OS_THREAD_STACK(FrequencyCalculateThreadStack, THREAD_STACK_SIZE);    /*!< The stack for the FrequencyCalculate thread. */
OS_THREAD_STACK(FrequencyTrackThreadStack, THREAD_STACK_SIZE);        /*!< The stack for the FrequencyTrack thread. */
OS_THREAD_STACK(PacketThreadStack, THREAD_STACK_SIZE);                /*!< The stack for the Packet thread. */
OS_THREAD_STACK(LogRaisesThreadStack, THREAD_STACK_SIZE);             /*!< The stack for the Log Raises thread. */
OS_THREAD_STACK(LogLowersThreadStack, THREAD_STACK_SIZE);             /*!< The stack for the Log Lowers thread. */
OS_THREAD_STACK(FFTThreadStack, 2000);                                /*!< The stack for the Log Lowers thread. */
static uint32_t RMSThreadStacks[NB_ANALOG_CHANNELS]
                [THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));  /*!< The thread stack array for RMS threads */

/*! @brief Calculates the square root of a number quickly by knowing the previous value of the square.
 *
 *  @param square The number to find the square root of
 *  @param lastRoot The root of the last square
 *  @param accuracy The desired accuracy of the root
 *  @return uint16_t - The square root
 */
uint16_t FastSqrt(uint32_t square, uint16_t lastRoot, uint16_t accuracy)
{
  int32_t error;
  uint32_t last;
  uint32_t root = lastRoot;
  if(root <= 0)
  {
    root = 1;
  }
  if(root <= 0)
  {
    return 0;
  }
  do
  {
    last = root;
    root = (root + (square / root)) / 2;
    error = root - last;
  }while(error > accuracy || error <-accuracy);
  return root;
}

/*! @brief Reads a word with mutex access
 *
 *  @param pVariable The address of the variable.
 *  @param mutex The mutex semaphore to access the variable
 *  @return uint16_t The variable
 */
uint16_t ProtectedUint16Get(uint16* pVariable, OS_ECB* mutex)
{
  uint16_t halfword;
  OS_SemaphoreWait(mutex, 0);
  halfword = *pVariable;
  OS_SemaphoreSignal(mutex);
  return halfword;
}

/*! @brief Puts a halfword into an address with mutex access
 *
 *  @param pAddress The address of the variable.
 *  @param variable The variable to put in the address
 *  @param mutex The mutex semaphore to access the variable to be overwritten
 *  @return void
 */
void ProtectedUint16Put(uint16_t* pAddress, uint16_t variable, OS_ECB* mutex)
{
  OS_SemaphoreWait(mutex, 0);
  *pAddress = variable;
  OS_SemaphoreSignal(mutex);
}

/*! @brief Reads a word with mutex access
 *
 *  @param pVariable The address of the variable.
 *  @param mutex The mutex semaphore to access the variable
 *  @return uint32_t The variable
 */
uint32_t ProtectedUint32Get(uint32* pVariable, OS_ECB* mutex)
{
  uint32_t word;
  OS_SemaphoreWait(mutex, 0);
  word = *pVariable;
  OS_SemaphoreSignal(mutex);
  return word;
}

/*! @brief Puts a word into an address with mutex access
 *
 *  @param pAddress The address of the variable.
 *  @param variable The variable to put in the address
 *  @param mutex The mutex semaphore to access the variable to be overwritten
 *  @return void
 */
void ProtectedUint32Put(uint32_t* pAddress, uint32_t variable, OS_ECB* mutex)
{
  OS_SemaphoreWait(mutex, 0);
  *pAddress = variable;
  OS_SemaphoreSignal(mutex);
}

/*! @brief Calls Analog_Put with interrupts disabled.
 *
 *  @param channelNb The number of the analog output channel to send the value to.
 *  @param value The value to write to the analog channel.
 *  @return bool - true if the analog value was output successfully.
 */
bool ProtectedAnalogPut(uint8_t const channelNb, int16_t const value)
{
  bool success = false;
  OS_DisableInterrupts();
  success = Analog_Put(channelNb, value);
  OS_EnableInterrupts();
  return success;
}

/*! @brief Put FFT data into a global array of frequency bins. Uses Mutex access.
 *
 *  @param fftOutput The new data to put in the global array.
 *  @param size The number of elements in the array
 *  @return void
 */
void ProtectedFFTPut(kiss_fft_cpx* fftOutput, uint8_t size)
{
  // Gain exclusive access to data
  OS_SemaphoreWait(FFTOutputMutex, 0);

  // Copy each element into array
  for(uint8_t i = 0; i < size; i++)
  {
    FFTOutput[i] = fftOutput[i];
  }

  // Release exclusive access to data
  OS_SemaphoreSignal(FFTOutputMutex);
}
/*! @brief Gets a harmonic magnitude from a global array off FFT frequency bins. Uses Mutex access.
 *
 *  @param The harmonic number
 *  @return void
 */

uint16_t ProtectedFFTGet(uint8_t harmonic)
{
  // Gain exclusive access to data
  OS_SemaphoreWait(FFTOutputMutex, 0);

  // Gain a local copy then release Mutex. Note: element 0 of the array is the DC component.
  kiss_fft_cpx localVoltage = FFTOutput[harmonic];

  // Release exclusive access to data
  OS_SemaphoreSignal(FFTOutputMutex);

  // Get the magnitude by taking the square root of the sum of the real and imaginary components squared, dividing values by nfft / 2 to scale correctly.
  return (uint16_t)FastSqrt(((uint32_t)(((int32_t)localVoltage.r  / (ADC_SAMPLES_PER_CYCLE / 2)) * ((int32_t)localVoltage.r) / (ADC_SAMPLES_PER_CYCLE / 2))
         + (((int32_t)localVoltage.i  / (ADC_SAMPLES_PER_CYCLE / 2))* ((int32_t)localVoltage.i / (ADC_SAMPLES_PER_CYCLE / 2)))), 0, 1);
}
/*! @brief Checks that no other analog thread has an output set
 *
 *  @param outputs[] The array of flags
 *  @param nboutputs The number of flags in the array
 *  @param mutex The mutex to the flag/flag array
 *  @return bool - TRUE if all flags are false
 */

bool ProtectedCheckFlagsOff(const bool  flags[], const uint8_t nbFlags, OS_ECB* mutex)
{
  OS_SemaphoreWait(mutex,0);
  bool set = false;
  for (uint8_t i = 0; i < nbFlags; i++)
  {
    set |= flags[i];
  }
  OS_SemaphoreSignal(mutex);
  return !set;
}

/*! @brief Updates a flag with mutex access
 *
 *  @param flag The flag to be updated
 *  @param flagValue The new value of the flag
 *  @param mutex The mutex to the flag/flag array
 *  @return void
 */
void ProtectedFlagUpdate(bool* flag, const bool flagValue, OS_ECB* mutex)
{
  OS_SemaphoreWait(mutex,0);
  *flag = flagValue;
  OS_SemaphoreSignal(mutex);
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
    Packet_Put(COMMAND_MODE,PARAM_GET,NvTowerMd->s.Lo,NvTowerMd->s.Hi));
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
    return Flash_Write((uint16_t*)NvTowerNb,(uint16_t)Packet_Parameter23, (uint8_t)16);
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
    return Flash_Write( (uint8_t*)nvAddress, Packet_Parameter3, (uint8_t)8);
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
    return Packet_Put(COMMAND_READBYTE,Packet_Parameter1,0,_FB((volatile uint8_t*)(FLASH_DATA_START + (uint32_t)Packet_Parameter1)));
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
    return Flash_Write((uint16_t*)NvTowerMd,(uint16_t)Packet_Parameter23, (uint8_t)16);
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

/*! @brief Sets or gets timing mode as requested by PC
 *
 *  @return bool - TRUE if mode is set or sent
 */
bool HandleTowerTimingMode(void)
{
  // Check if parameters are within tower time limits and get or set definite or inverse
  if(Packet_Parameter1 == TIMING_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Send Timing Mode
    Packet_Put(COMMAND_TIMINGMODE, _FB(NvTimingMd), 0, 0);
    return true;
  }
  else if((Packet_Parameter1 == TIMING_DEFINITE || Packet_Parameter1 == TIMING_INVERSE) && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Set Timing Mode
    if (Flash_Write((uint8_t*)NvTimingMd,(uint8_t)Packet_Parameter1, (uint8_t)8))
    {
      ProtectedUint16Put(&TimingMode, _FB(NvTimingMd), TimingModeMutex);
      return true;
    }
  }
  return false;
}

/*! @brief Sets the number of raises to 0 or sends number of raises to PC
 *
 *  @return bool - TRUE if packet is sent or number is set
 */
bool HandleTowerRaises(void)
{
  // Check if parameters match tower number GET or SET parameters
  if(Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the number of raises
    return Packet_Put(COMMAND_RAISES,_FB(NvNbRaises), 0, 0);
  }
  else if(Packet_Parameter1 == 1 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Resets the number of raises
    return Flash_Write((uint8_t*)NvNbRaises,(uint8_t)0, (uint8_t)8);
  }
  return false;
}

/*! @brief Sets the number of lowers to 0 or sends number of lowers to PC
 *
 *  @return bool - TRUE if packet is sent or number is set
 */
bool HandleTowerLowers(void)
{
  // Check if parameters match tower number GET or SET parameters
  if(Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the number of lowers
    return Packet_Put(COMMAND_LOWERS,_FB(NvNbLowers), 0, 0);
  }
  else if(Packet_Parameter1 == 1 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Resets the number of lowers
    return Flash_Write((uint8_t*)NvNbLowers,(uint8_t)0, (uint8_t)8);
  }
  return false;
}

/*! @brief Sends the frequency to the PC
 *
 *  @return bool - TRUE if packet sent
 */
bool HandleTowerFrequency(void)
{
  // Local storage of variable as union
  uint16union_t frequency;

  // Check if parameters are within limits
  if(Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Get a local copy of Frequency
    uint16_t localFrequency = ProtectedUint16Get(&Frequency, FrequencyMutex);

    // Type-cast as a uint16  and convert to dHz to be sent to PC.
    frequency.l = (uint16_t)(localFrequency / 100);

    // Check if we need to round up
    if(localFrequency % 100 >= 50)
    {
      frequency.l ++;
    }

    // Send frequency to PC
    return Packet_Put(COMMAND_FREQUENCY, frequency.s.Hi, frequency.s.Lo, 0);
  }
  return false;
}

/*! @brief Sends the voltage (RMS) of a given channel to the PC
 *
 *  @return bool - TRUE if packet sent
 */
bool HandleTowerVoltage(void)
{
  // Local storage of variable as union
  uint16union_t rms;

  // Check if parameters are within limits
  if(Packet_Parameter1 > 0 && Packet_Parameter1 <= NB_ANALOG_CHANNELS && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Get RMS value
    rms.l = ProtectedUint16Get(&(RMS[Packet_Parameter1 -1]), RMSMutex);

    // Send Voltage to PC
    return Packet_Put(COMMAND_VOLTAGE, Packet_Parameter1, rms.s.Hi, rms.s.Lo);
  }
  return false;
}

/*! @brief Sends the magnitude of a given harmonic to the PC
 *
 *  @return bool - TRUE if packet sent
 *  @note Peak magnitude used.
 */
bool HandleTowerHarmonic(void)
{
  // Local storage of variable as union
  uint16union_t magnitude;

  // Check if parameters are within limits
  if(Packet_Parameter1 < 8 &&Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Get magnitude from complex data
    magnitude.l = ProtectedFFTGet(Packet_Parameter1);

    // Send magnitude to PC
    return Packet_Put(COMMAND_SPECTRUM, Packet_Parameter1, magnitude.s.Hi, magnitude.s.Lo);
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
  else if(commandIgnoreAck == COMMAND_TIMINGMODE)
  {
    // Handle timing mode command
    success = HandleTowerTimingMode();
  }
  else if(commandIgnoreAck == COMMAND_RAISES)
  {
    // Set of Get number of raises
    success = HandleTowerRaises();
  }
  else if(commandIgnoreAck == COMMAND_LOWERS)
  {
    // Set of Get number of lowers
    success = HandleTowerLowers();
  }
  else if(commandIgnoreAck == COMMAND_FREQUENCY)
  {
    // Send Frequency to Tower
    success = HandleTowerFrequency();
  }
  else if(commandIgnoreAck == COMMAND_VOLTAGE)
  {
    // Send Voltage to Tower
    success = HandleTowerVoltage();
  }
  else if(commandIgnoreAck == COMMAND_SPECTRUM)
  {
    // Send Harmonic to Tower
    success = HandleTowerHarmonic();
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

/*! @brief Reads data from the analog channels and signals semaphores for RMS and Frequency treads
 *
 *  @return void
 */
void ADCReadCallback(void* arg)
{
  // Keep track of what phase to read from
  static uint8_t phase = 0;

  switch (phase){
    case 0:

      // Read from the ADC and put data in the correct element of the array.
      Analog_Get(phase, &(VoltageSamples[phase].ADC_Data[VoltageSamples[phase].LatestData]));
      VoltageSamples[phase].LatestData++;

      // Signal FFT Semaphore
      if(VoltageSamples[phase].LatestData == ADC_SAMPLES_PER_CYCLE)
      {
        OS_SemaphoreSignal(FFTSemaphore);
      }

      // If we are at the end of the buffer, signal semaphore for FrequencyCalculate thread, also set LatestData to 0.
      if(VoltageSamples[phase].LatestData == ADC_BUFFER_SIZE)
      {
        VoltageSamples[phase].LatestData = 0;
        OS_SemaphoreSignal(FrequencyCalculateSemaphore);
      }

      // Signal semaphore for RMS thread
      OS_SemaphoreSignal(AnalogThreadData[phase].semaphore);
      phase ++;
      break;
    case 1:

      // Read from the ADC and put data in the correct element of the array.
      Analog_Get(phase, &(VoltageSamples[phase].ADC_Data[VoltageSamples[phase].LatestData]));
      VoltageSamples[phase].LatestData++;

      // If we are at the end of the array, set LatestData back to 0.
      if(VoltageSamples[phase].LatestData == ADC_BUFFER_SIZE)
      {
        VoltageSamples[phase].LatestData = 0;
      }

      // Signal semaphore for RMS thread
      OS_SemaphoreSignal(AnalogThreadData[phase].semaphore);
      phase ++;
      break;
    case 2:

      // Read from the ADC and put data in the correct element of the array.
      Analog_Get(phase, &(VoltageSamples[phase].ADC_Data[VoltageSamples[phase].LatestData]));
      VoltageSamples[phase].LatestData++;

      // If we are one element away from the end of the array, signal FrequencyTrack semaphore for thread to load new PIT value.
      if(VoltageSamples[phase].LatestData == ADC_BUFFER_SIZE - 1)
      {
        OS_SemaphoreSignal(FrequencyTrackSemaphore);
      }

      // If we are at the end of the array, set LatestData back to 0.
      if(VoltageSamples[phase].LatestData == ADC_BUFFER_SIZE)
      {
        VoltageSamples[phase].LatestData = 0;
      }

      // Signal semaphore for RMS thread
      OS_SemaphoreSignal(AnalogThreadData[phase].semaphore);
      phase = 0;
      break;
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

  if (Flash_Init() &&  LEDs_Init() && Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ) && FTM_Init() &&
      RTC_Init(RTCReadSemaphore) && Analog_Init(CPU_BUS_CLK_HZ) && PIT_Init(CPU_BUS_CLK_HZ, ADCReadCallback, NULL)/*&& Accel_Init(&accelSetup)*/)
  {
    success = true;

    // Allocates an address in Flash memory to the tower number, tower mode, timing mode, number of raises and number of lowers
    if(Flash_AllocateVar((volatile void**)&NvTowerNb, sizeof(*NvTowerNb))
    && Flash_AllocateVar((volatile void**)&NvTowerMd, sizeof(*NvTowerMd))
    && Flash_AllocateVar((volatile void**)&NvTimingMd, sizeof(*NvTimingMd))
    && Flash_AllocateVar((volatile void**)&NvNbRaises, sizeof(*NvNbRaises))
    && Flash_AllocateVar((volatile void**)&NvNbLowers, sizeof(*NvNbLowers)));
    {
      // Checks if tower number is clear
      if(_FH((volatile uint16_t*)(NvTowerNb)) == 0xFFFF)
      {
        // Sets the tower number to the default number
        if(!Flash_Write((uint16_t*)NvTowerNb,(uint16_t)3756, (uint8_t)16))
        {
          success = false;
        }
      }

      // Checks if tower mode is clear
      if(_FH((volatile uint16_t*)(NvTowerMd)) == 0xFFFF)
      {
        // Sets the tower mode to the default mode
        if(!Flash_Write((uint16_t*)NvTowerMd,(uint16_t)1, (uint8_t)16))
        {
          success = false;
        }
      }

      // Checks if timing mode is valid
      if((_FB(NvTimingMd) != TIMING_DEFINITE) && (_FB(NvTimingMd) != TIMING_INVERSE))
      {
        // Sets the timing mode to the default mode
        if(!Flash_Write((uint8_t*)NvTimingMd, (uint8_t)TIMING_DEFINITE, (uint8_t)8))
        {
          success = false;
        }
        TimingMode = _FB(NvTimingMd);
      }

      else if((_FB(NvTimingMd) == TIMING_DEFINITE) || (_FB(NvTimingMd) == TIMING_INVERSE))
      {
        TimingMode = _FB(NvTimingMd);
      }

      // Checks if number of raises is clear
      if(_FB(NvNbRaises) == 0xFF)
      {
        // Sets the number of raises to 0
        if(!Flash_Write((uint8_t*)NvNbRaises,(uint8_t)0, (uint8_t)8))
        {
          success = false;
        }
      }

      // Checks if number of lowers is clear
      if(_FB(NvNbLowers) == 0xFF)
      {
        // Sets the number of raises to 0
        if(!Flash_Write((uint8_t*)NvNbLowers,(uint8_t)0, (uint8_t)8))
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
  // Create semaphores and Mutex semaphores for threads
  LEDOffSemaphore             = OS_SemaphoreCreate(0);
  RTCReadSemaphore            = OS_SemaphoreCreate(0);
  FrequencyCalculateSemaphore = OS_SemaphoreCreate(0);
  FrequencyTrackSemaphore     = OS_SemaphoreCreate(0);
  LogRaisesSemaphore          = OS_SemaphoreCreate(0);
  LogLowersSemaphore          = OS_SemaphoreCreate(0);
  FFTSemaphore                = OS_SemaphoreCreate(0);
  FlashAccessMutex            = OS_SemaphoreCreate(1);
  FFTOutputMutex              = OS_SemaphoreCreate(1);
  AlarmMutex                  = OS_SemaphoreCreate(1);
  AdjustingMutex              = OS_SemaphoreCreate(1);
  RMSCalculationDataMutex     = OS_SemaphoreCreate(1);
  SamplePeriodMutex           = OS_SemaphoreCreate(1);
  NewSamplePeriodMutex        = OS_SemaphoreCreate(1);
  FrequencyMutex              = OS_SemaphoreCreate(1);
  RMSMutex                    = OS_SemaphoreCreate(1);
  TimingModeMutex             = OS_SemaphoreCreate(1);

  // Create semaphore for RMS channels
  for (uint8_t i = 0; i < NB_ANALOG_CHANNELS; i++)
  {
    AnalogThreadData[i].semaphore = OS_SemaphoreCreate(0);
  }

  // Initializes the main tower components and sets the default or stored values
  if(TowerInit())
  {
    // Turn on the orange LED to indicate the tower has initialized successfully
    LEDs_On(LED_ORANGE);
  }
  
  // Send startup packets to PC
  HandleTowerStartup();

  // Calculate the initial sample period (in nano-seconds)
  SamplePeriod = (uint32_t)(1000000000/(ADC_DEFAULT_FREQUENCY * ADC_SAMPLES_PER_CYCLE));

  // Set the new sampler period and frequency
  NewSamplePeriod = SamplePeriod;
  Frequency = ADC_DEFAULT_FREQUENCY;

  // Set and enable the PIT
  PIT_Set(SamplePeriod / NB_ANALOG_CHANNELS, false);
  PIT_Enable(true);

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

/*! @brief Writes a logged VRR raise event to non-volatile memory
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note Assumes that Flash_Init has been called successfully.
 */
static void LogRaisesThread(void* pData)
{
  uint8_t events;
  for (;;)
  {
    // Wait for Log Raise semaphore
    OS_SemaphoreWait(LogRaisesSemaphore,0);

    // Gain access to flash
    OS_SemaphoreWait(FlashAccessMutex, 0);

    events = _FB(NvNbRaises);
    if(events != 255)
    {
      events ++;
      Flash_Write((uint8_t*)NvNbRaises, events, (uint8_t)8);
    }

    // Release access to flash
    OS_SemaphoreSignal(FlashAccessMutex);
  }
}

/*! @brief Writes a logged VRR lower event to non-volatile memory
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note Assumes that Flash_Init has been called successfully.
 */
static void LogLowersThread(void* pData)
{
  uint8_t events;
  for (;;)
  {
    // Wait for Log Raise semaphore
    OS_SemaphoreWait(LogLowersSemaphore,0);

    // Gain access to flash
    OS_SemaphoreWait(FlashAccessMutex, 0);

    events = _FB(NvNbLowers);
    if(events != 255)
    {
      events ++;
      Flash_Write((uint8_t*)NvNbLowers, events, (uint8_t)8);
    }
    // Release access to flash
    OS_SemaphoreSignal(FlashAccessMutex);
  }
}

/*! @brief Loads a new value into PIT in order to track phase A frequency
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note Assumes that Flash_Init has been called successfully.
 */
static void FrequencyTrackThread(void* pData)
{
  uint8_t events;

  for (;;)
  {
    // Wait for Frequency Track semaphore
    OS_SemaphoreWait(FrequencyTrackSemaphore,0);

    uint32_t newSamplePeriod = ProtectedUint32Get(&NewSamplePeriod, NewSamplePeriodMutex);

    // Set New PIT sample period
    OS_DisableInterrupts();
    PIT_Set((uint32_t)(newSamplePeriod / NB_ANALOG_CHANNELS), false);
    OS_EnableInterrupts();
  }
}

/*! @brief Calculates the frequency for an analog channel
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note Assumes that Flash_Init has been called successfully.
 */
static void FrequencyCalculateThread(void* pData)
{
  TVoltageData localSamples;
  uint32_t newSamplePeriod;
  uint32_t samplePeriod;
  uint16_t rms;
  uint8_t crossingCount;
  uint32_t risingCrossings[10];
  uint32_t lastCrossing;
  uint32_t frequency;
  uint32_t period;

  for (;;)
  {
    // Wait for FrequencyTrack semaphore
    OS_SemaphoreWait(FrequencyCalculateSemaphore,0);

    // Get a local copy of phase A RMS,
    rms = ProtectedUint16Get(&(RMS[0]), RMSMutex);

    samplePeriod = ProtectedUint32Get(&SamplePeriod, SamplePeriodMutex);

    // Check if RMS is in the frequency reading range
    if(rms > RMS_FREQUENCY_MIN)
    {
      // Get a local copy that cannot be modified by and interrupt
      OS_DisableInterrupts();
      for(uint8_t i = 0; i < ADC_BUFFER_SIZE; i++)
      {
        localSamples.ADC_Data[i] = VoltageSamples[0].ADC_Data[i];
      }
      OS_EnableInterrupts();

      // Reset array element counter
      crossingCount = 0;

      // Find rising crossings from sample data and store interpolated result in array (units are in sample periods)
      for(uint8_t i = 0; i < ADC_BUFFER_SIZE - 1; i++)
      {
        if((localSamples.ADC_Data[i + 1] > 0) && (localSamples.ADC_Data[i] <= 0))
        {
          uint16_t m = (uint16_t)(localSamples.ADC_Data[i + 1]) - (localSamples.ADC_Data[i]);
          int16_t b = localSamples.ADC_Data[i];
          risingCrossings[crossingCount] = (i * 1000) + ((((-b) * 1000) / m));
          crossingCount ++;
        }
      }

      // If we have a buffer size equal to the number of samples per cycle, we could get only one crossing and need to reference last cycles crossing
      if(crossingCount == 1)
      {
        // Calculate the period
        period = ((ADC_SAMPLES_PER_CYCLE + risingCrossings[crossingCount -1]) - lastCrossing);

        // Update the last crossing
        lastCrossing = risingCrossings[crossingCount - 1] -((crossingCount -1) * ADC_SAMPLES_PER_CYCLE);
      }

      // If we have a larger buffer, or if it is equal but the frequency has increased we could get more than 1 rising crossing from the sample
      else if(crossingCount > 1)
      {
        period = ((risingCrossings[crossingCount - 1]) - risingCrossings[0]) / (crossingCount - 1);

        // Update the last crossing
        lastCrossing = risingCrossings[crossingCount - 1] -((crossingCount -1) * (ADC_SAMPLES_PER_CYCLE));
      }

      // If we have a buffer size equal to the number of samples per cycle, we could get no crossings if the frequency has dropped.
      else if(crossingCount == 0)
      {
        //Update the last crossing as a negative number so it can be used normally next time around. Do not update period in this round.
        lastCrossing -= ADC_SAMPLES_PER_CYCLE;
      }

      // Find the period in nanoseconds.
      newSamplePeriod = (uint32_t)(uint32_t)((((uint64_t)period * samplePeriod) /(1000 * ADC_SAMPLES_PER_CYCLE)));

      // From the period (in number of sample periods) and the sample period, work out the frequency in mHz.
      frequency = (uint16_t)((uint64_t)1000000000000 / ((period * (samplePeriod / 1000))));

      // Update the Frequency
      ProtectedUint16Put(&Frequency, (uint16)frequency, FrequencyMutex);

      // Update sample periods (for use by Frequency track thread and this thread)
      uint32_t dummyNewSamplePeriod = ProtectedUint32Get(&NewSamplePeriod, NewSamplePeriodMutex);
      ProtectedUint32Put(&SamplePeriod, dummyNewSamplePeriod, SamplePeriodMutex);
      ProtectedUint32Put(&NewSamplePeriod, newSamplePeriod, NewSamplePeriodMutex);
    }
  }
}

/*! @brief Updates the RMS using the previous sum of squares to save time. From testing it take around 64us vs 112us from scratch.
 *
 *  @param pRemoveData The pointer to the data to remove from the sum of squares
 *  @param pPreviousSumOfSquares The pointer to the previous sum of squares
 *  @param latestData The latest data sample
 *  @param oldestData The oldest data sample to be removed next time
 *  @param dataSize The number of data samples for average of sum of squares
 *  @param lastRMS The last RMS value of the data
 *  @return uint16_t - The RMS value
 */
uint16_t UpdateRMSFast(int16_t* const pRemoveData, int64_t* const pPreviousSumOfSquares, const int16_t latestData,
                       const int16_t oldestData, const uint8_t dataSize, const uint16_t lastRMS, OS_ECB* mutex)
{
  // Gain exclusive access to global variables
  OS_SemaphoreWait(mutex, 0);

  // Cast to int32_t to avoid calculations later
  int32_t newestData = (int32_t)latestData;

  // Update the sum of squares, removing old data and adding new data.
  int64_t newSumOfSquares = *pPreviousSumOfSquares;

  newSumOfSquares += (newestData * newestData);

  newSumOfSquares -= (int64_t)((int32_t)(*pRemoveData) * (int32_t)(*pRemoveData));

  // Ensure the new sum is positive
  if(newSumOfSquares < 0)
  {
    newSumOfSquares = ((*pRemoveData) * (*pRemoveData));
  }

  // Update the removed data and sum of squares for next time.
  *pRemoveData = oldestData;
  *pPreviousSumOfSquares = newSumOfSquares;

  // Release access to global variables
  OS_SemaphoreSignal(mutex);

  // Return the calculated RMS
  return (uint16_t)FastSqrt(newSumOfSquares /dataSize, lastRMS, 1);
}

/*! @brief Gets the RMS, checks limits and handles alarms and raise/lower timing
 *
 *  @param pData The pointer to the analog thread data
 *  @note Assumes that semaphores have been created.
 */
void RMSThread(void* pData)  //TODO: commenting from here on. Also create enumerated type for DAC channel numbers
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

  uint16_t rms;
  uint16_t rmsTest;
  uint16_t deltaVoltage;
  uint32_t minTimeCheck;
  int64_t OutOfRangeTimer = ALARM_TIMER;
  bool alarm;
  bool alarmSet;
  bool overVoltage;
  bool underVoltage;
  bool adjusting;

  for (;;)
  {
    // Wait for channel semaphore
    (void)OS_SemaphoreWait(analogData->semaphore, 0);

    // Get a local copy of the latest data sample
    int16_t latestSample;
    int16_t oldestSample;

    // Disable interrupts to get local copy. Cannot use mutex as interrupts update this variable
    OS_DisableInterrupts();

    if(VoltageSamples[analogData->channelNb].LatestData == 0)
    {
      latestSample = VoltageSamples[analogData->channelNb].ADC_Data[ADC_BUFFER_SIZE - 1];
    }
    else
    {
      latestSample = VoltageSamples[analogData->channelNb].ADC_Data[VoltageSamples[analogData->channelNb].LatestData - 1];
    }
    oldestSample = VoltageSamples[analogData->channelNb].ADC_Data[VoltageSamples[analogData->channelNb].LatestData];

    OS_EnableInterrupts();

    // Calculate the new RMS
    rms = UpdateRMSFast(&(OldestData[analogData->channelNb]), &(LastSumOfSquares[analogData->channelNb]),
                            latestSample, oldestSample, ADC_BUFFER_SIZE, RMS[analogData->channelNb], RMSCalculationDataMutex);

    // Update global RMS variable
    ProtectedUint16Put(&(RMS[analogData->channelNb]), rms, RMSMutex);

    // Check if data is in limits
    if(rms > RMS_UPPER_LIMIT)
    {
      alarm = true;
      overVoltage = true;
    }
    else if(rms < RMS_LOWER_LIMIT)
    {
      alarm = true;
      underVoltage = true;
    }

    // Set alarm if it is not set
    if(alarm && !alarmSet)
    {
      if(ProtectedCheckFlagsOff(Alarm, (uint8_t)NB_ANALOG_CHANNELS, AlarmMutex))
      {
        ProtectedAnalogPut(OUTPUT_ALARM, DAC_5V_OUT);
        ProtectedFlagUpdate(&Alarm[analogData->channelNb], true, AlarmMutex);
      }
      alarmSet = true;
    }

    if(alarm && !adjusting)
    {
      uint32_t samplePeriod = ProtectedUint32Get(&SamplePeriod, SamplePeriodMutex);
      if(ProtectedUint16Get(&TimingMode, TimingModeMutex) == TIMING_DEFINITE)
      {
        OutOfRangeTimer -= samplePeriod;

        // Keep track of actual time
        minTimeCheck += samplePeriod;
      }
      else if(ProtectedUint16Get(&TimingMode, TimingModeMutex) == TIMING_INVERSE)
      {
        if(overVoltage)
        {
          deltaVoltage = rms - RMS_UPPER_LIMIT;
        }
        else if(underVoltage)
        {
          deltaVoltage = RMS_LOWER_LIMIT - rms;
        }

        // Decrement counter by inverse amount (1638 represents 0.5V)
        OutOfRangeTimer -= (((uint64_t)deltaVoltage * (uint64_t)samplePeriod) / ADC_HALF_VOLT);

        // Keep track of actual time
        minTimeCheck += samplePeriod;
      }

      // Check if raise or lower signals should be set
      if(OutOfRangeTimer <= 0 && !adjusting && minTimeCheck >= ALARM_TIMER_MIN)
      {
        // Set lower signal and store count in Nv if voltage high and not already set by another channel.
        if(overVoltage)
        {
          if(ProtectedCheckFlagsOff(Adjusting, (uint8_t)NB_ANALOG_CHANNELS, AdjustingMutex))
          {
            ProtectedAnalogPut(OUTPUT_LOWER, DAC_5V_OUT);
            ProtectedFlagUpdate(&Adjusting[analogData->channelNb], true, AdjustingMutex);
            OS_SemaphoreSignal(LogLowersSemaphore);
          }
          adjusting = true;
        }

        // Set raise signal and store count in Nv if voltage low and not already set by another channel.
        else if(underVoltage)
        {
          if(ProtectedCheckFlagsOff(Adjusting, (uint8_t)NB_ANALOG_CHANNELS, AdjustingMutex))
          {
            ProtectedAnalogPut(OUTPUT_RAISE, DAC_5V_OUT);
            ProtectedFlagUpdate(&Adjusting[analogData->channelNb], true, AdjustingMutex);
            OS_SemaphoreSignal(LogRaisesSemaphore);
          }
          adjusting = true;
        }
      }
    }

    // Check if raise / lower should be decremented
    else if(!alarm)
    {
      bool switchOffAdjusting;

      ProtectedFlagUpdate(&Alarm[analogData->channelNb], false, AlarmMutex);
      if(ProtectedCheckFlagsOff(Alarm, (uint8_t)NB_ANALOG_CHANNELS, AlarmMutex))
      {
        ProtectedAnalogPut(OUTPUT_ALARM, DAC_0V_OUT);
      }

      ProtectedFlagUpdate(&Adjusting[analogData->channelNb], false, AdjustingMutex);
      if(ProtectedCheckFlagsOff(Adjusting, (uint8_t)NB_ANALOG_CHANNELS, AdjustingMutex))
      {
        switchOffAdjusting = true;
      }

      if(switchOffAdjusting)
      {
        ProtectedAnalogPut(OUTPUT_RAISE, DAC_0V_OUT);
        ProtectedAnalogPut(OUTPUT_LOWER, DAC_0V_OUT);
      }
      OutOfRangeTimer = ALARM_TIMER;
      adjusting = false;
      alarmSet = false;
    }

    // Reset variables
    alarm = false;
    overVoltage = false;
    underVoltage = false;
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

/*! @brief Performs a Forward Fast Fourier Transform on the given time data.
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note Uses a real only FFT which saves close to 45% of the time required for a complex FFT
 */
static void FFTThread(void* pData)
{
   uint8_t memory[500];                                               /*!< Memory space allocated for kiss_fftr_cfg*/
   size_t size = 500;                                                 /*!< Size of memory allocated for kiss_fftr_cfg*/
   kiss_fft_scalar fftInput[ADC_SAMPLES_PER_CYCLE];                   /*!< Scalar array for input data*/
   kiss_fft_cpx fftOutput[(ADC_SAMPLES_PER_CYCLE / 2) + 1];           /*!< Complex array for output data */

  for (;;)
  {
    // Wait for Frequency Track semaphore
    OS_SemaphoreWait(FFTSemaphore,0);

    kiss_fftr_cfg config = kiss_fftr_alloc(16, 0, memory, &size);

    // Load time data into array. Note: interrupts are not disabled as after this threads semaphore is signaled,
    // it will be a long time (3 * 16 * 1.25 ms for the default sampling rate and buffer size) before the
    // PIT interrupt will modify any data in this part of the array.

    for(uint8_t i = 0; i < ADC_SAMPLES_PER_CYCLE; i++)
    {
      fftInput[i] = VoltageSamples[0].ADC_Data[i];
    }

    // Run forward FFT
    kiss_fftr(config, fftInput, fftOutput);

    // Update global array
    ProtectedFFTPut(fftOutput, (uint8_t)((ADC_SAMPLES_PER_CYCLE / 2) + 1));
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
  // Create threads for RMS channels
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(RMSThread,
                            &AnalogThreadData[threadNb],
                            &RMSThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            ANALOG_THREAD_PRIORITIES[threadNb]);
  }
  // 6th Highest priority
  error = OS_ThreadCreate(FrequencyCalculateThread,
                          NULL,
                          &FrequencyCalculateThreadStack[THREAD_STACK_SIZE - 1],
                          6);
  // 2nd Highest priority
  error = OS_ThreadCreate(FrequencyTrackThread,
                          NULL,
                          &FrequencyTrackThreadStack[THREAD_STACK_SIZE - 1],
                          2);
  // 16th Highest priority
  error = OS_ThreadCreate(FFTThread,
                          NULL,
                          &FFTThreadStack[THREAD_STACK_SIZE - 1],
                          16);

  // 10th Highest priority
  error = OS_ThreadCreate(PacketThread,
			  NULL,
                          &PacketThreadStack[THREAD_STACK_SIZE - 1],
                          10);
  // 17th Highest priority
  error = OS_ThreadCreate(LogRaisesThread,
        NULL,
                          &LogRaisesThreadStack[THREAD_STACK_SIZE - 1],
                          17);
  // 18th Highest priority
  error = OS_ThreadCreate(LogLowersThread,
        NULL,
                          &LogLowersThreadStack[THREAD_STACK_SIZE - 1],
                          18);
  // 29th Highest priority
  error = OS_ThreadCreate(FTMLEDsOffThread,
                          NULL,
                          &FTMLEDsOffThreadStack[THREAD_STACK_SIZE - 1],
                          29);
  // 30th Highest priority
  error = OS_ThreadCreate(RTCThread,
                          NULL,
                          &RTCThreadStack[THREAD_STACK_SIZE - 1],
                          30);

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
