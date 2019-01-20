#pragma once

///////////////////////////////////////////////////////////////////////////////
// Global variables
//
// Basically anything that might need to be shared between different classes
// in this project. 
//
// The Utilities.cpp file has EXTERN defined to nothing, so that including
// this file causes the variables to be defined rather than merely declared.
///////////////////////////////////////////////////////////////////////////////

#ifndef EXTERN
#define EXTERN extern
#endif

//#ifndef byte
//#define byte char
//#endif

EXTERN void* null;

const unsigned DisplayWidth = 16;

// Extra characters allow for padding values with trailing spaces
EXTERN char DisplayLine1[DisplayWidth + 1];
EXTERN char DisplayLine2[DisplayWidth + 1];

// For display usage
EXTERN char CurrentModeName[DisplayWidth];
EXTERN char ErrorMessage[DisplayWidth];
EXTERN char LastErrorMessage[DisplayWidth];

// For unit testing
EXTERN char FailureMessage[100];

// For debugging/diagnostics
EXTERN unsigned InitializationErrorCount;
EXTERN unsigned ErrorCount;
EXTERN unsigned DebugLeft;
EXTERN unsigned DebugCrank;
EXTERN unsigned DebugRight;

EXTERN unsigned DebugLong1;
EXTERN unsigned DebugLong2;

// To measure update rate
EXTERN unsigned IterationsPerSecond;

// Raw sensor state
EXTERN unsigned MapSensorState;
EXTERN unsigned KnobState;

// Centigrade (160F = 71C)
EXTERN unsigned OilTemperature;

EXTERN unsigned PlxPacketCount;
EXTERN unsigned PlxByteCount;

// Angles
EXTERN float CamTargetAngle;
EXTERN float LeftCamError;
EXTERN float RightCamError;

// Actuator Duty Cycle
EXTERN unsigned LeftSolenoidDutyCycle;
EXTERN unsigned RightSolenoidDutyCycle;

/* PLX receive testing
EXTERN unsigned ReceivedCrankRpm;
EXTERN unsigned ReceivedLeftRpm;
EXTERN unsigned ReceivedRightRpm;
EXTERN unsigned ReceivedFluidPressureAddress;
EXTERN unsigned ReceivedFluidPressure;
EXTERN unsigned ReceivedSensorCount;
*/

// Constants for turning timer ticks into RPM and degrees.
//
// This one is defined in InterruptHandlers.cpp because it depends on which
// type of timer is being used. (There is currently one type proven and another
// in development.)
extern const unsigned TicksPerSecond;
extern const unsigned TicksPerMinute;

bool initialisationComplete = false; //Tracks whether the setup() function has run completely
#define _SAM3XA_
//Handy bitsetting macros
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_CHECK(var,pos) !!((var) & (1<<(pos)))

#define interruptSafe(c)  noInterrupts(); c interrupts(); //Wraps any code between nointerrupt and interrupt calls

#define MS_IN_MINUTE 60000
#define US_IN_MINUTE 60000000
#define TOOTH_LOG_BUFFER    256
unsigned long revolutionTime; //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
#define MAX_RPM 10000 //This is the maximum rpm that the ECU will attempt to run at. It is NOT related to the rev limiter, but is instead dictates how fast certain operations will be allowed to run. Lower number gives better performance
volatile uint16_t toothHistory[TOOTH_LOG_BUFFER];
volatile unsigned int toothHistoryIndex = 0;
byte primaryTriggerEdge;
byte secondaryTriggerEdge;
int CRANK_ANGLE_MAX = 720;
#define SIZE_BYTE           8

//The status struct contains the current values for all 'live' variables
//In current version this is 64 bytes
struct statuses {
  volatile bool hasSync;
  uint16_t RPM;
  long longRPM;
  int mapADC;
  int baroADC;
  long MAP; //Has to be a long for PID calcs (Boost control)
  int16_t EMAP;
  int16_t EMAPADC;
  byte baro; //Barometric pressure is simply the inital MAP reading, taken before the engine is running. Alternatively, can be taken from an external sensor
  byte TPS; //The current TPS reading (0% - 100%)
  byte TPSlast; //The previous TPS reading
  unsigned long TPS_time; //The time the TPS sample was taken
  unsigned long TPSlast_time; //The time the previous TPS sample was taken
  byte tpsADC; //0-255 byte representation of the TPS
  byte tpsDOT;
  volatile int rpmDOT;
  byte VE;
  byte O2;
  byte O2_2;
  int coolant;
  int cltADC;
  int IAT;
  int iatADC;
  int batADC;
  int O2ADC;
  int O2_2ADC;
  int dwell;
  byte dwellCorrection; //The amount of correction being applied to the dwell time.
  byte battery10; //The current BRV in volts (multiplied by 10. Eg 12.5V = 125)
  int8_t advance; //Signed 8 bit as advance can now go negative (ATDC)
  byte corrections;
  int16_t TAEamount; //The amount of accleration enrichment currently being applied
  byte egoCorrection; //The amount of closed loop AFR enrichment currently being applied
  byte wueCorrection; //The amount of warmup enrichment currently being applied
  byte batCorrection; //The amount of battery voltage enrichment currently being applied
  byte iatCorrection; //The amount of inlet air temperature adjustment currently being applied
  byte launchCorrection; //The amount of correction being applied if launch control is active
  byte flexCorrection; //Amount of correction being applied to compensate for ethanol content
  int8_t flexIgnCorrection; //Amount of additional advance being applied based on flex. Note the type as this allows for negative values
  byte afrTarget;
  byte idleDuty;
  bool idleUpActive;
  bool fanOn; //Whether or not the fan is turned on
  volatile byte ethanolPct; //Ethanol reading (if enabled). 0 = No ethanol, 100 = pure ethanol. Eg E85 = 85.
  unsigned long TAEEndTime; //The target end time used whenever TAE is turned on
  volatile byte status1;
  volatile byte spark;
  volatile byte spark2;
  byte engine;
  unsigned int PW1; //In uS
  unsigned int PW2; //In uS
  unsigned int PW3; //In uS
  unsigned int PW4; //In uS
  unsigned int PW5; //In uS
  unsigned int PW6; //In uS
  unsigned int PW7; //In uS
  unsigned int PW8; //In uS
  volatile byte runSecs; //Counter of seconds since cranking commenced (overflows at 255 obviously)
  volatile byte secl; //Continous
  volatile unsigned int loopsPerSecond;
  boolean launchingSoft; //True when in launch control soft limit mode
  boolean launchingHard; //True when in launch control hard limit mode
  uint16_t freeRAM;
  unsigned int clutchEngagedRPM;
  bool flatShiftingHard;
  volatile uint32_t startRevolutions; //A counter for how many revolutions have been completed since sync was achieved.
  uint16_t boostTarget;
  byte testOutputs;
  bool testActive;
  uint16_t boostDuty; //Percentage value * 100 to give 2 points of precision
  byte idleLoad; //Either the current steps or current duty cycle for the idle control.
  uint16_t canin[16];   //16bit raw value of selected canin data for channel 0-15
  uint8_t current_caninchannel = 0; //start off at channel 0
  uint16_t crankRPM = 400; //The actual cranking RPM limit. Saves us multiplying it everytime from the config page
  volatile byte status3;
  int16_t flexBoostCorrection; //Amount of boost added based on flex
  byte nitrous_status;
  byte nSquirts;
  byte nChannels; //Number of fuel and ignition channels
  int16_t fuelLoad;
  int16_t ignLoad;
  bool fuelPumpOn; //The current status of the fuel pump
  byte syncLossCounter;
  byte knockRetard;
  bool knockActive;
  bool toothLogEnabled;
  bool compositeLogEnabled;

  //Helpful bitwise operations:
  //Useful reference: http://playground.arduino.cc/Code/BitMath
  // y = (x >> n) & 1;    // n=0..15.  stores nth bit of x in y.  y becomes 0 or 1.
  // x &= ~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
  // x |= (1 << n);       // forces nth bit of x to be 1.  all other bits left alone.

};

struct statuses currentStatus; //The global status object

//Page 4 of the config - See the ini file for further reference
//This mostly covers off variables that are required for ignition
struct config4 {

  int16_t triggerAngle;
  int8_t FixAng; //Negative values allowed
  byte CrankAng;
  byte TrigAngMul; //Multiplier for non evenly divisible tooth counts.

  byte TrigEdge : 1;
  byte TrigSpeed : 1;
  byte IgInv : 1;
  byte TrigPattern : 5;

  byte TrigEdgeSec : 1;
  byte fuelPumpPin : 6;
  byte useResync : 1;

  byte sparkDur; //Spark duration in ms * 10
  byte trigPatternSec; //Mode for Missing tooth secondary trigger.  Either single tooth cam wheel or 4-1
  uint8_t bootloaderCaps; //Capabilities of the bootloader over stock. e.g., 0=Stock, 1=Reset protection, etc.

  byte resetControl : 2; //Which method of reset control to use (0=None, 1=Prevent When Running, 2=Prevent Always, 3=Serial Command)
  byte resetControlPin : 6;

  byte StgCycles; //The number of initial cycles before the ignition should fire when first cranking

  byte boostType : 1; //Open or closed loop boost control
  byte useDwellLim : 1; //Whether the dwell limiter is off or on
  byte sparkMode : 3; //Spark output mode (Eg Wasted spark, single channel or Wasted COP)
  byte triggerFilter : 2; //The mode of trigger filter being used (0=Off, 1=Light (Not currently used), 2=Normal, 3=Aggressive)
  byte ignCranklock : 1; //Whether or not the ignition timing during cranking is locked to a CAS pulse. Only currently valid for Basic distributor and 4G63.

  byte dwellCrank; //Dwell time whilst cranking
  byte dwellRun; //Dwell time whilst running
  byte triggerTeeth; //The full count of teeth on the trigger wheel if there were no gaps
  byte triggerMissingTeeth; //The size of the tooth gap (ie number of missing teeth)
  byte crankRPM; //RPM below which the engine is considered to be cranking
  byte floodClear; //TPS value that triggers flood clear mode (No fuel whilst cranking)
  byte SoftRevLim; //Soft rev limit (RPM/100)
  byte SoftLimRetard; //Amount soft limit retards (degrees)
  byte SoftLimMax; //Time the soft limit can run
  byte HardRevLim; //Hard rev limit (RPM/100)
  byte taeBins[4]; //TPS based acceleration enrichment bins (%/s)
  byte taeValues[4]; //TPS based acceleration enrichment rates (% to add)
  byte wueBins[10]; //Warmup Enrichment bins (Values are in configTable1)
  byte dwellLimit;
  byte dwellCorrectionValues[6]; //Correction table for dwell vs battery voltage
  byte iatRetBins[6]; // Inlet Air Temp timing retard curve bins
  byte iatRetValues[6]; // Inlet Air Temp timing retard curve values
  byte dfcoRPM; //RPM at which DFCO turns off/on at
  byte dfcoHyster; //Hysteris RPM for DFCO
  byte dfcoTPSThresh; //TPS must be below this figure for DFCO to engage

  byte ignBypassEnabled : 1; //Whether or not the ignition bypass is enabled
  byte ignBypassPin : 6; //Pin the ignition bypass is activated on
  byte ignBypassHiLo : 1; //Whether this should be active high or low.

  byte ADCFILTER_TPS;
  byte ADCFILTER_CLT;
  byte ADCFILTER_IAT;
  byte ADCFILTER_O2;
  byte ADCFILTER_BAT;
  byte ADCFILTER_MAP; //This is only used on Instantaneous MAP readings and is intentionally very weak to allow for faster response
  byte ADCFILTER_BARO;

  byte unused2_64[57];

#if defined(CORE_AVR)
  };
#else
  } __attribute__((__packed__)); //The 32 bit systems require all structs to be fully packed
#endif

struct config4 configPage4;
