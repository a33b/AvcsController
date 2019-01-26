/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
This file contains the various crank and cam wheel decoder functions.

Each decoder must have the following 4 functions (Where xxxx is the decoder name):

* triggerSetup_xxx - Called once from within setup() and configures any required variables
* triggerPri_xxxx - Called each time the primary (No. 1) crank/cam signal is triggered (Called as an interrupt, so variables must be declared volatile)
* triggerSec_xxxx - Called each time the secondary (No. 2) crank/cam signal is triggered (Called as an interrupt, so variables must be declared volatile)
* getRPM_xxxx - Returns the current RPM, as calculated by the decoder
* getCrankAngle_xxxx - Returns the current crank angle, as calculated b the decoder

And each decoder must utlise at least the following variables:
toothLastToothTime - The time (In uS) that the last primary tooth was 'seen'
*

*/
//#include <limits.h>
#include "Globals.h"
#include "decoder.h"
//#include "scheduledIO.h"
//#include "scheduler.h"
#include "crankMaths.h"

static inline uint16_t stdGetRPM(uint16_t degreesOver)
{
  uint16_t tempRPM = 0;

  if( currentStatus.hasSync == true )
  {
    if( (currentStatus.RPM < currentStatus.crankRPM) && (currentStatus.startRevolutions == 0) ) { tempRPM = 0; } //Prevents crazy RPM spike when there has been less than 1 full revolution
    else if( (toothOneTime == 0) || (toothOneMinusOneTime == 0) ) { tempRPM = 0; }
    else
    {
      noInterrupts();
      revolutionTime = (toothOneTime - toothOneMinusOneTime); //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
      interrupts();
      if(degreesOver == 720) { revolutionTime = revolutionTime / 2; }
      tempRPM = (US_IN_MINUTE / revolutionTime); //Calc RPM based on last full revolution time (Faster as /)
      if(tempRPM >= MAX_RPM) { tempRPM = currentStatus.RPM; } //Sanity check
    }
  }
  else { tempRPM = 0; }

  return tempRPM;
}

/*
 * Sets the new filter time based on the current settings.
 * This ONLY works for even spaced decoders
 */
static inline void setFilter(unsigned long curGap)
{
   if(configPage4.triggerFilter == 1) { triggerFilterTime = curGap >> 2; } //Lite filter level is 25% of previous gap
   else if(configPage4.triggerFilter == 2) { triggerFilterTime = curGap >> 1; } //Medium filter level is 50% of previous gap
   else if (configPage4.triggerFilter == 3) { triggerFilterTime = (curGap * 3) >> 2; } //Aggressive filter level is 75% of previous gap
   else { triggerFilterTime = 0; } //trigger filter is turned off.
}

/*
This is a special case of RPM measure that is based on the time between the last 2 teeth rather than the time of the last full revolution
This gives much more volatile reading, but is quite useful during cranking, particularly on low resolution patterns.
It can only be used on patterns where the teeth are evenly spaced
It takes an argument of the full (COMPLETE) number of teeth per revolution. For a missing tooth wheel, this is the number if the tooth had NOT been missing (Eg 36-1 = 36)
*/
static inline int crankingGetRPM(byte totalTeeth)
{
  uint16_t tempRPM = 0;
  if( (currentStatus.startRevolutions >= configPage4.StgCycles) && (currentStatus.hasSync == true) )
  {
    if( (toothLastToothTime > 0) && (toothLastMinusOneToothTime > 0) && (toothLastToothTime > toothLastMinusOneToothTime) )
    {
      noInterrupts();
      revolutionTime = (toothLastToothTime - toothLastMinusOneToothTime) * totalTeeth;
      interrupts();
      tempRPM = (US_IN_MINUTE / revolutionTime);
      if( tempRPM >= MAX_RPM ) { tempRPM = currentStatus.RPM; } //Sanity check. This can prevent spiking caused by noise on individual teeth. The new RPM should never be above 4x the cranking setting value (Remembering that this function is only called is the current RPM is less than the cranking setting)
    }
  }

  return tempRPM;
}


void triggerSetup_NissanVQ()
{
  triggerToothAngle = 10; //The number of degrees that passes from tooth to tooth (primary). This is the maximum gap
  triggerActualTeeth = 30; //The number of physical teeth on the wheel. Doing this here saves us a calculation each time in the interrupt
  triggerFilterTime = (int)(1000000 / (MAX_RPM / 60 * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise
  secondDerivEnabled = false;
  decoderIsSequential = true;
  checkSyncToothCount = (configPage4.triggerTeeth) >> 1; //50% of the total teeth.
  toothLastMinusOneToothTime = 0;
  gapPriCurrentRev= 0;
  toothCurrentCount = 0;
  toothOneTime = 0;
  teethSinceTDC = 0;
  toothOneMinusOneTime = 0;
  secondaryToothCount = 0; //Needed for the cam tooth tracking
  MAX_STALL_TIME = (3333UL * triggerToothAngle * 2); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  
  triggerSecFilterTime = (int)(US_IN_MINUTE / (MAX_RPM * 18)) / 2; //Same as above, but fixed at 18 teeth (if none were missing) on the secondary input and divided by 2 (for cam speed)
  triggerEXHFilterTime = (int)(US_IN_MINUTE / (MAX_RPM * 3)) / 2; //Same as above, but fixed at 3 teeth on the secondary input and divided by 2 (for cam speed)
  //decoderHasFixedCrankingTiming = true;
}

void triggerPri_NissanVQ()
{
   curTime = micros();
   curGap = curTime - toothLastToothTime;
   if ( curGap >= triggerFilterTime ) //Pulses should never be less than triggerFilterTime, so if they are it means a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
   {
     toothCurrentCount++; //Increment the tooth counter
     validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

     //Begin the missing tooth detection
     //If the time between the current tooth and the last is greater than 2x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after a gap
     targetGap = ((toothLastToothTime - toothLastMinusOneToothTime)) * 2; //Multiply by 2 

     if( (toothLastToothTime == 0) || (toothLastMinusOneToothTime == 0) ) { curGap = 0; }

     if ( (curGap > targetGap) )
     {
        //We've seen a missing tooth set
        toothCurrentCount += 2 ;
        gapPriCurrentRev++;
        triggerToothAngleIsCorrect = false; //The tooth angle is triple at this point
        if(teethSinceTDC != 1)
        {
          currentStatus.syncLossCounter++;
          currentStatus.hasSync = false;
        }
        teethSinceTDC = 4;
       // triggerFilterTime = 0; //This is used to prevent a condition where serious intermitent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
      }
     else
      {
        teethSinceTDC++;
        if(teethSinceTDC == 11) //shift counts down and reset current counts
        {
          secondaryToothCountMinus2 = secondaryToothCountMinus1;
          intake2ToothCountMinus2 = intake2ToothCountMinus1;
          secondaryToothCountMinus1 = secondaryToothCount;
          intake2ToothCountMinus1 = intake2ToothCount;
          secondaryToothCount = 0;
          intake2ToothCount = 0;
        }
        if((gapPriCurrentRev == 0) && (toothCurrentCount == 9)) {teethSinceTDC = 0;}  //possible only at startup
        
        if(teethSinceTDC > 11) {teethSinceTDC = 0;}
        
       //Filter can only be recalc'd for the regular teeth, not the missing one.
       //setFilter(curGap);
       triggerToothAngleIsCorrect = true;
      }

     if(gapPriCurrentRev > 3) //Means a complete rotation has occured.
     {
        gapPriCurrentRev = 1;
        revolutionOne = !revolutionOne; //Flip sequential revolution tracker
        toothOneMinusOneTime = toothOneTime;  // unless it is an issue, toothOne is going to be the first tooth found after any gap (1 of 3)
        toothOneTime = curTime;
        currentStatus.startRevolutions++; //Counter
     }
/* Intake cams have identical pattern phased 360 crank degrees apart
A single cam can establish sync in 360 (or less) crank degrees, 
with both cams working sync can be established with 120 crank degrees
To determine sync, we need to count the number of falling edges of each cam after a TDC event
Cams can advance up to 40 degrees which puts the first falling edge right at TDC

    B1 Edges  | 2 | 0 | 1 | 0 | 1 | 2 |
    B2 Edges  | 0 | 1 | 2 | 2 | 0 | 1 |
Next Inj cyl  | 1 | 2 | 3 | 4 | 5 | 6 |
B1 Failsafe   |201|010|101|012|122|220| These are the last 3 counts during each 120 degree crank rotation by bank 2
B2 Failsafe   |012|122|220|201|010|101| These are the last 3 counts during each 120 degree crank rotation by bank 1
              |34 \ 10\ 22\ 34\ 10\ 22|
*/
     if(teethSinceTDC == 9) //count the number of secondary tooth pulses after a TDC event within 90 degrees crank  
      {
        if((toothCurrentCount > checkSyncToothCount) && (currentStatus.hasSync == false))
        {
         switch(secondaryToothCount)
         {
          case 2:
           if(secondaryToothCountMinus1 == 2 && intake2ToothCount == 0) // evaluate whether cams signals are synced
           {
            toothCurrentCount = 34;
            currentStatus.hasSync = true;
            break;
           }
           if(secondaryToothCountMinus1 == 1 && intake2ToothCount == 1) // evaluate whether cams signals are synced
           {
            toothCurrentCount = 22;
            currentStatus.hasSync = true;
            break;
           }
           currentStatus.hasSync = false; //either the cams mismatch or the previous tooth count is invalid
           break;
          
          case 1:
           if(secondaryToothCountMinus2 == 2 && intake2ToothCount == 2) 
           { 
             toothCurrentCount = 22;
             currentStatus.hasSync = true;
             break; 
            }
           if(secondaryToothCountMinus2 == 1 && intake2ToothCount == 0) 
           { 
             toothCurrentCount = 10;
             currentStatus.hasSync = true;
             break; 
            }
           currentStatus.hasSync = false;
           break;
          
          case 0:
           if(secondaryToothCountMinus1 == 2 && intake2ToothCount == 1)
           {
            toothCurrentCount = 10;
            currentStatus.hasSync = true;
            break;
           }
           if (secondaryToothCountMinus1 == 1 && intake2ToothCount == 2)
           {
             toothCurrentCount = 34;
             currentStatus.hasSync = true;
            break;
           }
           currentStatus.hasSync = false;
         }
        }
      }
    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;
   }
}


void trigger_SecNissanVQ()  

{
  curTime2 = micros();
  curGap2 = curTime2 - secondaryLastToothTime;
  if ( curGap2 < triggerSecFilterTime ) { return; }
  secondaryLastToothTime1 = secondaryLastToothTime;
  secondaryLastToothTime = curTime2;
  secondaryToothCount++;
}

void trigger_IN2NissanVQ()
{
  curTime3 = micros();
  curGap3 = curTime3 - intake2LastToothTime;
  if ( curGap3 < triggerSecFilterTime ) { return; }
  intake2LastToothTime1 = intake2LastToothTime;
  intake2LastToothTime = curTime3;
  intake2ToothCount++;
}


void trigger_EX1NissanVQ()  //this will be just for phase of exhaust cams, not for sync
// exhaust cams have 3 teeth per revolution evenly sized and spaced 
{
  curTime4 = micros();
  curGap4 = curTime4 - exhaust1LastToothTime;
  if ( (curGap4 < triggerEXHFilterTime) ) { return; }
  exhaust1LastToothTime1 = exhaust1LastToothTime;
  exhaust1LastToothTime = curTime4;
}

void trigger_EX2NissanVQ()  //this will be just for phase of exhaust cams, not for sync
// exhaust cams have 3 teeth per revolution evenly sized and spaced 
{
  curTime5 = micros();
  curGap5 = curTime5 - exhaust2LastToothTime;
  if ( (curGap5 < triggerEXHFilterTime) ) { return; }
  exhaust2LastToothTime1 = exhaust2LastToothTime;
  exhaust2LastToothTime = curTime4;
}

uint16_t getRPM_NissanVQ()
{
  uint16_t tempRPM = 0;
  if( currentStatus.hasSync == true )
  {
    if(currentStatus.RPM < currentStatus.crankRPM) 
    { 
      tempRPM = crankingGetRPM(configPage4.triggerTeeth);
      if (teethSinceTDC = 4) { tempRPM *= 3; }
    }
    else { tempRPM = stdGetRPM(360); }
  }
  return tempRPM;
}

int getCrankAngle_NissanVQ()
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    bool tempRevolutionOne;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempRevolutionOne = revolutionOne;
    tempToothLastToothTime = toothLastToothTime;
    interrupts();

    int crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    
    //Sequential check (simply sets whether we're on the first or 2nd revoltuion of the cycle)
    if ( (tempRevolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) ) { crankAngle += 360; }

    lastCrankAngleCalc = micros();
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_TOOTH);

    if (crankAngle >= 720) { crankAngle -= 720; }
    else if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

void triggerSetEndTeeth_NissanVQ()
{
  lastToothCalcAdvance = currentStatus.advance;
}
