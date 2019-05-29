//LoadControl.c
#define __LoadControl_c__

#include "load_control.h"
#include "outback.h"
#include "usb_arduino.h"
#include <wiringPi.h>
#include <softPwm.h>
#include <unistd.h>
#include <ncurses.h>
#include <time.h>

extern int compressor (int SwOn);
extern WINDOW *InvWin, *CCWin, *FNDCWin, *ScrollWin;
extern void logMesg (const char *fmt, ...);
extern struct tm *tm_p;
extern bool preferHeatPumpOn;
extern void SoundAlarm (int ms);
extern enum AirCondPwrSrcModes AirCondPwrSrc;
extern int airJordanManOff;
extern void cmdMate (char *cmd, char *v, int ibookmark);
extern void AlterCurrentLimit (int NewLimit, int ibookmark, int direction);
extern float MC_On_Temp;

#define VACATION_VSET		55
#define LE_SETPOINT_START	120
#define	LE_DIVISOR			10
#define UE_SETPOINT_START	115
#define	UE_DIVISOR			10
#define AC_SETPOINT_START	135
#define	AC_DIVISOR			20


#define WHMAXTEMP (WHmaxAnyTemp-(100-FNDC_SOC))
#define PRETURN	{WMVPRINTW(FNDCWin,11,22,"%6d",__LINE__);return;}

#define DEFAULT_DELAY	5
#define WH_TOP_NORMAL_TEMP	120.0

#define DCamps(ACamps)	((float)(((ACamps*L1_INVERTER_VOUT)/AFNDCV)-3))
#define TEMPSENSOR(ID)	(readSensorF(WH_SENSOR_##ID##_INDX, 666))


enum inRange
{ irBelow = (-1), irIn = 0, irAbove = 1 };


struct loadAmt
{
  int L1;
  int L2;
};

/*	lower element LV is L2  10A
 * HV is L1 and L2 at 20A
 * upper element is L1 at 10A
 * compressor is 11A on L1
*/

int EstL1A, EstL2A, NeedToShedAmpsL1, NeedToShedAmpsL2;
static int lcDelay = DEFAULT_DELAY;
static int modeChangeDelay = 4;


void
syncACPS (void)
{
  AirCondPwrSrc =
    (digitalRead (MRCOOL2KHP_SRC_GPIO) ==
     0) ? acpsGrid : ((digitalRead (MRCOOL2KHP_PWR_GPIO) ==
		       1) ? acpsInverter : acpsNone);
}

int
setACPS (enum AirCondPwrSrcModes newMode)
{
  static time_t acpsOffTime;

  if (newMode == AirCondPwrSrc)
    return 0;
  if (difftime (time (NULL), acpsOffTime) < 250)
    {
      if ((newMode != acpsNone) && (AirCondPwrSrc == acpsNone))
	return 0;
    }
  switch (newMode)
    {
    case acpsGrid:
      //digitalWrite (MRCOOL2KHP_PWR_GPIO, 0);
      digitalWrite (MRCOOL2KHP_SRC_GPIO, 0);
      break;
    case acpsInverter:
      if (((TOT_LOAD (L1) + 10) < MAX_LOAD_AMPS)
	  && ((TOT_LOAD (L2) + 10) < MAX_LOAD_AMPS))
	{
	  digitalWrite (MRCOOL2KHP_SRC_GPIO, 1);
	  digitalWrite (MRCOOL2KHP_PWR_GPIO, 1);
	}
      else
	{
	  return 0;
	}
      break;
    case acpsNone:
      if (AirCondPwrSrc != acpsNone)
	time (&acpsOffTime);
      //digitalWrite (MRCOOL2KHP_PWR_GPIO, 0);
      digitalWrite (MRCOOL2KHP_SRC_GPIO, 1);
      break;
    case acpsOn:
      if (((TOT_LOAD (L1) + 10) < MAX_LOAD_AMPS)
	  && ((TOT_LOAD (L2) + 10) < MAX_LOAD_AMPS)
	  && ((INVERTER_AC_MODE != 0) || (AFNDCV > 54.0)))
	{
	  digitalWrite (MRCOOL2KHP_SRC_GPIO, 1);
	  digitalWrite (MRCOOL2KHP_PWR_GPIO, 1);
	}
      else
	{
	  //digitalWrite (MRCOOL2KHP_PWR_GPIO, 0);
	  digitalWrite (MRCOOL2KHP_SRC_GPIO, 0);
	}
      break;
    }
  syncACPS ();
  return ((AirCondPwrSrc == newMode)
	  || ((newMode == acpsOn) && (AirCondPwrSrc != acpsNone)));
}

#define MIN_SECS_BETWEEN_CHANGES	10
#define ACC_ON_NOW	3
#define ACC_OFF_NOW	2

int
airCondControl (int targetState)
{
  static time_t tChangeTime = 0;


  if ((difftime (time (NULL), tChangeTime) > MIN_SECS_BETWEEN_CHANGES)
      || (targetState > ON))
    {
      if ((targetState == ON) || (targetState == ACC_ON_NOW))	//ON=turn On if time has elapsed /3 means now
	{
	  if ((digitalRead (AIR_COND_GPIO_PIN) == OFF) && (readSensorF(3,666)<MC_On_Temp))
	    {
	      digitalWrite (AIR_COND_GPIO_PIN, ON);
	      time (&tChangeTime);
	      return TRUE;
	    }
	}
      else if ((targetState == OFF) || (targetState == ACC_OFF_NOW))	// OFF is turn Off if time has elapsed / 2 means now
	{
	  if (digitalRead (AIR_COND_GPIO_PIN) == ON)
	    {
	      digitalWrite (AIR_COND_GPIO_PIN, OFF);
	      time (&tChangeTime);
	      return TRUE;
	    }
	}
      return FALSE;
    }
  return FALSE;
}

int
AirJorPrefered (int desired)	//1 for on, 0 for off, -1 for immediate off (regurdless of timer)
{
  int AJ = digitalRead (AIR_JORDAN_SRC_PIN);
  static time_t AirJordanSwitchTime = 0;

  if (airJordanManOff == TRUE)
    {
      if (AJ)
	digitalWrite (AIR_JORDAN_SRC_PIN, OFF);
      return 0;
    }
  if ((AJ == desired) || ((desired == (-1)) && (AJ == 0)))
    return 0;
  if ((desired == (-1)) && (difftime (time (NULL), AirJordanSwitchTime) > 30))
    {
      digitalWrite (AIR_JORDAN_SRC_PIN, OFF);
      time (&AirJordanSwitchTime);
      return desired;
    }
  if (difftime (time (NULL), AirJordanSwitchTime) > 300) //3 sec is low because no compressor is affected at this time.
    {
      if (((desired == 1) && ((TOT_LOAD (L1) + AIR_COND_AMPS) < MAX_LOAD_AMPS)
//	   && ((TOT_LOAD (L2) + AIR_COND_AMPS) < MAX_LOAD_AMPS)
	   && ((TEMPSENSOR (TOP) > 120.0)
	       || ((UE_PWM.percent > 80) && LE_IS_ON) || (vacation == TRUE)
	       || ((INVERTER_AC_MODE == iacmUse)
		   && (InvInputMode == GridTied)))) || (desired == 0))
	{
	  digitalWrite (AIR_JORDAN_SRC_PIN, desired);
	  time (&AirJordanSwitchTime);
	  return 1;
	}
    }
  return 0;
}

//load shed will seek overload conditions and reduce loads
void
loadShed (void)
{
  static int LastL1A = 0, LastL2A = 0;

  syncACPS ();
  if ((INVERTER_AC_MODE != iacmDrop) && !GT_NoDrop
      && (INVERTER_AC_MODE != iacmNoGr))
    modeChangeDelay = 4;
  else
    modeChangeDelay--;
  if (lcDelay > 0)
    {
      lcDelay--;
      return;
    }
  EstL1A = TOT_LOAD (L1);
  EstL2A = TOT_LOAD (L2);
  if ((EstL1A > MAX_LOAD_AMPS) && (EstL1A < (LastL1A + 5)))
    {				//got to shed something on L1 (lower element is L2)
      NeedToShedAmpsL1 = EstL1A - MAX_LOAD_AMPS;
      if ((UE_PWM.percent > 0) && (NeedToShedAmpsL1 > 0))
	{
	  NeedToShedAmpsL1 -= ((UPPER_L1_DIFF * UE_PWM.percent) / 100);
	  EstL1A -= ((UPPER_L1_DIFF * UE_PWM.percent) / 100);
	  UEOFF;
	  logMesg ("LC @ %d\n", __LINE__);
	  lcDelay = DEFAULT_DELAY;
	}
      if ((NeedToShedAmpsL1 > 0) && (AirCondPwrSrc == acpsInverter))	//heat pump--we don't know if a load exists nor the size of it
	{
	  setACPS ((vacation == false) ? acpsGrid : acpsNone);
	  lcDelay = DEFAULT_DELAY;
	  NeedToShedAmpsL1 -= 5;	//We don't know what, if anything, was reduced. Maybe the 5A guess will delay futher triggers
	  logMesg ("LC @ %d  ACPS %s NTSAL1 %d EL1A %d\n", __LINE__,
		   acpsModeDesc[AirCondPwrSrc], NeedToShedAmpsL1, EstL1A);
	}
      if ((NeedToShedAmpsL1 > 13) && (compressor (ASK) != 0))	//compressor is ok to run -- switch off change bak to 3 when aux pump is protected
	{
	  compressor (OFF);
	  logMesg ("LC @ %d\n", __LINE__);
	  NeedToShedAmpsL1 -= COMPRESSOR_L1_DIFF;
	  EstL1A -= COMPRESSOR_L1_DIFF;
	  lcDelay = DEFAULT_DELAY;
	  SoundAlarm (100);
	}
      if ((NeedToShedAmpsL1 > 0))	//Jordan's room'
	{
	  if (AirJorPrefered (-1) != 0)
	    {
	      logMesg ("LC @ %d\n", __LINE__);
	      NeedToShedAmpsL1 -= AIR_COND_AMPS;
	      EstL1A -= AIR_COND_AMPS;
	      lcDelay = DEFAULT_DELAY;
	    }
	}
      if ((NeedToShedAmpsL1 > 0) && (digitalRead (AIR_COND_GPIO_PIN) == ON))	//this is the window unit in old living room
	{
	  airCondControl (ACC_OFF_NOW);
	  logMesg ("LC @ %d\n", __LINE__);
	  NeedToShedAmpsL1 -= AIR_COND_AMPS;
	  EstL1A -= AIR_COND_AMPS;
	  lcDelay = DEFAULT_DELAY;
	}
    }
  if ((EstL2A > MAX_LOAD_AMPS) && (EstL2A < (LastL2A + 5)))
    {				//got to shed something on L2
      NeedToShedAmpsL2 = EstL2A - MAX_LOAD_AMPS;

      if ((NeedToShedAmpsL2 > 0) && LE_IS_ON)
	{
	  LEOFF;
	  logMesg ("LC @ %d\n", __LINE__);
	  NeedToShedAmpsL2 -= (LOWER_LV_L2_DIFF * LE_PWM.percent / 100);
	  EstL2A -= (LOWER_LV_L2_DIFF * LE_PWM.percent / 100);
	  lcDelay = DEFAULT_DELAY;
	}
      if ((NeedToShedAmpsL2 > 0) && (AirCondPwrSrc == acpsInverter))
	{
	  setACPS ((vacation == false) ? acpsGrid : acpsNone);
	  lcDelay = DEFAULT_DELAY;
	  NeedToShedAmpsL2 -= 5;	//We don't know what, if anything, was reduced. Maybe the 5A guess will delay futher triggers
	  logMesg ("LC @ %d  ACPS %s NTSL2A %d EL2A %d\n", __LINE__,
		   acpsModeDesc[AirCondPwrSrc], NeedToShedAmpsL2, EstL2A);
	}
      if (NeedToShedAmpsL2 > 0)	//Jordan's room'
	{
	  if (AirJorPrefered (-1) != 0)
	    {
	      logMesg ("LC @ %d\n", __LINE__);
	      NeedToShedAmpsL1 -= AIR_COND_AMPS;
	      EstL1A -= AIR_COND_AMPS;
	      lcDelay = DEFAULT_DELAY;
	    }
	}
      if ((NeedToShedAmpsL2 > 0) && (digitalRead (AIR_COND_GPIO_PIN) == ON))
	{
	  airCondControl (ACC_OFF_NOW);
	  logMesg ("LC @ %d\n", __LINE__);
	  NeedToShedAmpsL2 -= AIR_COND_AMPS;
	  EstL2A -= AIR_COND_AMPS;
	  lcDelay = DEFAULT_DELAY;
	}
    }
  LastL1A = EstL1A;
  LastL2A = EstL2A;
  if (lcDelay == 0)
    LoadControl ();
}

#define TURN_LE_ON {digitalWrite(WH_LOWER_ELEMENT_SRC,1);LEON;}

void
LoadControl (void)
{
  int overTemp = (MAX (MAX (TEMPSENSOR (TOP), TEMPSENSOR (BOTTOM)),
		       MAX (TEMPSENSOR (CENTER_RIGHT),
			    TEMPSENSOR (CENTER_LEFT))) > MAX (WHMAXTEMP,
							      MAX
							      (WHtopMinTemp +
							       1,
							       WHCenterMinTemp
							       + 1)));
  float midTankTemp =
    (((TEMPSENSOR (CENTER_LEFT) * 55.0) + (TEMPSENSOR (BOTTOM) * 20.0) +
      (TEMPSENSOR (CENTER_RIGHT) * 7.5) + (TEMPSENSOR (TOP) * 17.5)) / 100.0);

  enum inRange topTank =
    (TEMPSENSOR (TOP) <
     WHtopMinTemp ? irBelow : ((TEMPSENSOR (TOP) > WHtopMaxTemp) ? irAbove :
			       irIn));
  enum inRange midTank =
    (midTankTemp <
     WHCenterMinTemp ? irBelow : (midTankTemp > WHMAXTEMP ? irAbove : irIn));
  static enum InverterACModes lastIACMode = iacmNoGr;

  if (TEMPSENSOR (TOP) > WHMAXTEMP)
    digitalWrite (WH_LOWER_ELEMENT_SRC, 1);
  if (compressor (ASK) == 0)	//Compressor switched off (presumably due to previous overload condition
    {
      if ((TOT_LOAD (L1) + COMPRESSOR_L1_DIFF) < MAX_LOAD_AMPS)
	{
	  compressor (ON);
	  logMesg ("LC @ %d\n", __LINE__);
	  lcDelay = DEFAULT_DELAY;
	  EstL1A += COMPRESSOR_L1_DIFF;
	}
    }
  if (overTemp)
    {
      //turn off all elements , set delay and return
      if (LE_IS_ON || (UE_PWM.percent > 0))
	{
	  if (midTankTemp >= WHCenterMinTemp)
	    LEOFF;		//if min not met keep on heating till popoff blows !!!
	  UEOFF;
	  lcDelay = DEFAULT_DELAY;
	  logMesg ("LC @ %d\n", __LINE__);
	  PRETURN;
	}
    }
  if ((!LE_IS_ON_FULL) && (midTank == irBelow)
      && (INVERTER_AC_MODE != iacmNoGr)
      && ((TOT_LOAD (L2) + LOWER_LV_L2_DIFF) < MAX_LOAD_AMPS))
    {
      //lower tank below min temp -- turn on lower element, set delay, and return
      AlterCurrentLimit ((TOT_LOAD (L2) + LOWER_LV_L2_DIFF), (0 - __LINE__),
			 1);

      LEON;
      lcDelay = DEFAULT_DELAY;
      logMesg ("LC @ %d\n", __LINE__);
      PRETURN;
    }
  if ((UE_PWM.percent < 100) && (topTank == irBelow)
      && (INVERTER_AC_MODE != iacmNoGr)
      && ((TOT_LOAD (L1) + UPPER_L1_DIFF) < MAX_LOAD_AMPS))
    {
      //upper tank temp below range -- turn on upper element, set delay, and return
      AlterCurrentLimit ((TOT_LOAD (L1) + UPPER_L1_DIFF), (0 - __LINE__), 1);
      UEON;
      lcDelay = DEFAULT_DELAY;
      logMesg ("LC @ %d\n", __LINE__);
      PRETURN;
    }
  if ((digitalRead (AIR_COND_GPIO_PIN) == FALSE) && (preferHeatPumpOn == TRUE)
      && (INVERTER_AC_MODE != iacmNoGr)
      && ((TOT_LOAD (L1) + AIR_COND_AMPS) < MAX_LOAD_AMPS)
      && ((TOT_LOAD (L2) + AIR_COND_AMPS) < MAX_LOAD_AMPS))
    {
      if (airCondControl (ON) == TRUE)
	{
	  lcDelay = DEFAULT_DELAY;
	  logMesg ("LC @ %d\n", __LINE__);
	  PRETURN;
	}
    }

  if ((INVERTER_AC_MODE == iacmNoGr))	//grid down 
    {
      if (lastIACMode != iacmNoGr)
	{
	  lastIACMode = iacmNoGr;
	  if ((AirCondPwrSrc == acpsInverter) && (AFNDCV < fSellV))
	    setACPS (acpsGrid);
	}
      if (AFNDCV < fSellV)
	{
	  //turn one element off, set delay , and return
	  if LE_IS_ON		//favoring upper element to get some hot quick
	    {
	      LE_DOWN;
	      lcDelay = DEFAULT_DELAY;
	      logMesg ("LC @ %d\n", __LINE__);
	      PRETURN;
	    }
	  if (UE_PWM.percent > 0)
	    {
	      UE_PWM.percent -= (100 / UE_PWM.resolution);
	      lcDelay = DEFAULT_DELAY;
	      logMesg ("LC @ %d  UE_PWM.percent=%d\n", __LINE__,
		       UE_PWM.percent);
	      PRETURN;
	    }
	  if (AFNDCV < (fSellV - 1.8))
	    {
	      if (AirJorPrefered (0) != 0)
		{
		  lcDelay = DEFAULT_DELAY;
		  logMesg ("LC @ %d\n", __LINE__);
		  PRETURN;
		}
	    }
	  if (digitalRead (AIR_COND_GPIO_PIN))
	    {
	      if (airCondControl (ACC_OFF_NOW) == TRUE)
		{
		  lcDelay = DEFAULT_DELAY;
		  logMesg ("LC @ %d\n", __LINE__);
		  PRETURN;
		}
	    }
	}
      if ((AirCondPwrSrc == acpsInverter) && (AFNDCV < (fSellV - 2.2)))
	{
	  if (setACPS (acpsGrid) != 0)
	    {
	      lcDelay = DEFAULT_DELAY;
	      logMesg ("LC @ %d  ACPS %s\n", __LINE__,
		       acpsModeDesc[AirCondPwrSrc]);
	      return;
	    }
	}

      else if (AFNDCV > (fSellV + 1.0))
	{
	  if ((digitalRead (AIR_COND_GPIO_PIN) == FALSE)
	      && (preferHeatPumpOn == TRUE)
	      && ((TOT_LOAD (L1) + AIR_COND_AMPS) < MAX_LOAD_AMPS)
	      && ((TOT_LOAD (L2) + AIR_COND_AMPS) < MAX_LOAD_AMPS))
	    {
	      if (airCondControl (ON) == TRUE)
		{
		  lcDelay = DEFAULT_DELAY;
		  logMesg ("LC @ %d\n", __LINE__);
		  PRETURN;
		}
	    }
	  if ((AirCondPwrSrc == acpsGrid)
	      && ((TOT_LOAD (L1) + 10) < MAX_LOAD_AMPS)
	      && ((TOT_LOAD (L2) + 10) < MAX_LOAD_AMPS))
	    {
	      if (setACPS (acpsInverter) != 0)
		{
		  lcDelay = DEFAULT_DELAY;
		  logMesg ("LC @ %d  ACPS %s\n", __LINE__,
			   acpsModeDesc[AirCondPwrSrc]);
		  return;
		}
	    }
	  if (AirJorPrefered (1) != 0)
	    {
	      lcDelay = DEFAULT_DELAY;
	      logMesg ("LC @ %d\n", __LINE__);
	      PRETURN;
	    }
	  if ((UE_PWM.percent < 100) && ((topTank == irBelow) || LE_IS_ON))
	    //favoring upper element to get some hot quick  
	    {

	      UE_PWM.percent += (100 / UE_PWM.resolution);
	      lcDelay = DEFAULT_DELAY;
	      logMesg ("LC @ %d   UE_PWM.percent=%d\n", __LINE__,
		       UE_PWM.percent);
	      PRETURN;
	    }
	  if ((!LE_IS_ON_FULL) && (midTank == irBelow))
	    {
	      LE_UP;
	      lcDelay = DEFAULT_DELAY;
	      logMesg ("LC @ %d\n", __LINE__);
	      PRETURN;
	    }
	}
      PRETURN;
    }
  else if ((INVERTER_AC_MODE == iacmUse) && (!GT_NoDrop))	//using grid -- minimize use unless grid tied and no drop selected
    {
      lastIACMode = iacmUse;
      if (AirCondPwrSrc != acpsNone)
	{
	  if (vacation == TRUE)
	    {
	      logMesg ("ACPS %d\n", AirCondPwrSrc);
	      logMesg ("LC @ %d  ACPS = %s\n", __LINE__,
		       acpsModeDesc[AirCondPwrSrc]);
	      if (setACPS (acpsNone))
		logMesg ("LC @ %d acps off\n", __LINE__);
	    }
	  else			//vacation==FALSE
	    {
	      if (AirCondPwrSrc != acpsInverter)
		{
		  if (setACPS (acpsInverter))
		    logMesg ("LC @ %d  ACPS = %s\n", __LINE__,
			     acpsModeDesc[AirCondPwrSrc]);
		}
	    }
	}
      if (digitalRead (AIR_COND_GPIO_PIN) && (preferHeatPumpOn == FALSE))
	{
	  if (airCondControl (ACC_OFF_NOW) == TRUE)
	    {
	      lcDelay = DEFAULT_DELAY;
	      logMesg ("LC @ %d\n", __LINE__);
	      PRETURN;
	    }
	}
      if (topTank != irBelow)
	{
	  //turn off top element
	  if (UE_PWM.percent > 0)
	    {
	      logMesg ("LC @ %d\n", __LINE__);
	      UEOFF;
	    }
	}
      else
	{
	  //turn on top element
	  if ((UE_PWM.percent < 100)
	      && ((EstL1A + UPPER_L1_DIFF) <= MAX_LOAD_AMPS)
	      && (INVERTER_AUX_OUT != 0))
	    {
	      logMesg ("LC @ %d\n", __LINE__);
	      UEON;
	    }
	}
      if (midTank != irBelow)
	{
	  // turn off bottom element
	  if LE_IS_ON
	    {
	      logMesg ("LC @ %d\n", __LINE__);
	      LEOFF;
	    }
	}
      else
	{
	  //turn on bottom element
	  if ((!LE_IS_ON_FULL)
	      && ((EstL2A + LOWER_LV_L2_DIFF) <= MAX_LOAD_AMPS))
	    {
	      logMesg ("LC @ %d\n", __LINE__);
	      LEON;
	    }
	}
      if (vacation == TRUE)
	{
	  setACPS (acpsNone);
	  if (AirJorPrefered (-1) != 0)
	    {
	      lcDelay = DEFAULT_DELAY;
	      logMesg ("LC @ %d\n", __LINE__);
	      PRETURN;
	    }

	}
      else			//(vacation==FALSE))
	{
	  if (INVERTER_OP_MODE != 10 /* not PassTh */ )
	    {
	      if (AirJorPrefered (ON) != 0)
		{
		  lcDelay = DEFAULT_DELAY;
		  logMesg ("LC @ %d\n", __LINE__);
		  PRETURN;
		}
	    }
	}
      PRETURN;
    }
  else if ((INVERTER_AC_MODE == iacmUse) && (GT_NoDrop))
    {
      // Put Grid Tied with no drop selected  code in here and return if something 
      // changes.  Will fall through to dropped grid mode below if no return.
      if ((!LE_IS_ON_FULL) && (midTank != irAbove)
	  && ((TOT_LOAD (L2) + LOWER_LV_L2_DIFF) < MAX_LOAD_AMPS)
	  && (loadBalance == -1))
	{
	  //lower tank temp in range -- turn on lower element, set delay, and return

	  if (!overTemp)
	    {
	      LE_UP;
	      loadBalance = 0;
	      lcDelay = 1 /*DEFAULT_DELAY*/;
	      logMesg ("LC @ %d LE==%d %4.2f %4.2f %4.1fF\n", __LINE__,
		       LE_PWM.percent, AFNDCV,
		       MAX (50.8 +
			    MAX (0,
				 ((TEMPSENSOR (CENTER_LEFT) -
				   LE_SETPOINT_START) / LE_DIVISOR)),
			    MIN (MAX (fSellV, 57), fSellV + 0.8)),
		       TEMPSENSOR (CENTER_LEFT));
	      PRETURN;
	    }
	}
      if ((UE_PWM.percent < 100) && (topTank != irAbove) && (!overTemp) &&
	  ((TOT_LOAD (L1) + UPPER_L1_DIFF) < MAX_LOAD_AMPS)
	  && (loadBalance == -1))
	{
	  //upper tank temp in range -- turn on upper element, set delay, and return
	  UE_UP;
	  lcDelay = 1/*DEFAULT_DELAY*/;
	  logMesg ("LC @ %d UE at %d, %4.2f %4.2f\n", __LINE__,
		   UE_PWM.percent, AFNDCV,
		   MAX (51.2 +
			MAX (0,
			     ((TEMPSENSOR (TOP) -
			       UE_SETPOINT_START) / UE_DIVISOR)),
			fSellV + 1.2));
	  loadBalance = 0;
	  PRETURN;
	}
    }
//Dropped Grid Mode
  lastIACMode = INVERTER_AC_MODE;
  if (modeChangeDelay > 0)
    {
      PRETURN;
    }

  if ((loadBalance == -1) || (UnderUtilization)
      || ((vacation == FALSE) && (ANBA > DCamps (LOWER_LV_L2_DIFF)))
      || ((vacation == FALSE) && (AFNDCV > MAX (54.0, fSellV)))
      || ((vacation == TRUE) && (AFNDCV > VACATION_VSET)))
    {				//excess power. Find something to turn on
      if ((vacation == TRUE) && (AirCondPwrSrc != acpsInverter))
	{
	  if (setACPS (acpsInverter))
	    {
	      lcDelay = 30;
	      logMesg ("LC @ %d acps inverter\n", __LINE__);
	      loadBalance = 0;
	      return;
	    }
	}
      if ((digitalRead (AIR_COND_GPIO_PIN) == FALSE)
	  && (preferHeatPumpOn == TRUE)
	  && ((TOT_LOAD (L1) + AIR_COND_AMPS) < MAX_LOAD_AMPS)
	  && ((TOT_LOAD (L2) + AIR_COND_AMPS) < MAX_LOAD_AMPS))
	{
	  if (airCondControl (ON) == TRUE)
	    {
	      lcDelay = DEFAULT_DELAY;
	      logMesg ("LC @ %d AC on (prefered)\n", __LINE__);
	      loadBalance = 0;
	      PRETURN;
	    }
	}
      if ((AirCondPwrSrc == acpsGrid)
	  && ((TOT_LOAD (L1) + 10) < MAX_LOAD_AMPS)
	  && ((TOT_LOAD (L2) + 10) < MAX_LOAD_AMPS))
	{
	  setACPS (acpsInverter);
	  lcDelay = DEFAULT_DELAY;
	  logMesg ("LC @ %d  ACPS = %s\n", __LINE__,
		   acpsModeDesc[AirCondPwrSrc]);
	  loadBalance = 0;
	}
      if (AirJorPrefered (1) != 0)
	{
	  lcDelay = DEFAULT_DELAY;
	  logMesg ("LC @ %d\n", __LINE__);
	  loadBalance = 0;
	  PRETURN;
	}
      //Top<120 start here
      if ((TEMPSENSOR (TOP) < WH_TOP_NORMAL_TEMP) &&
	  UE_PWM.percent < 100 && (topTank != irAbove) && (!overTemp) &&
	  (((TOT_LOAD (L1) + (UPPER_L1_DIFF / MAX (1, UE_PWM.resolution))) <
	    MAX_LOAD_AMPS) || (AFNDCV > 56.0))
	  && ((AFNDCV > MAX (50.7, fSellV + 0.6))
	      || (((100 - FNDC_SOC) * 2) < ANBA)) && (!GT_NoDrop))
	{
	  //upper tank temp in range -- turn on upper element, set delay, and return
	  UE_PWM.percent += (100 / UE_PWM.resolution);
	  loadBalance = 0;
	  lcDelay = DEFAULT_DELAY;
	  logMesg ("LC @ %d UE at %d, %4.2f %4.2f\n", __LINE__,
		   UE_PWM.percent, AFNDCV, MAX (50.7, fSellV + 0.6));
	  PRETURN;
	}
      //top >120 or we got more juce them bring up bottom temp
      if ((!LE_IS_ON_FULL) && (midTank != irAbove)
	  && ((TOT_LOAD (L2) + LOWER_LV_L2_DIFF) < MAX_LOAD_AMPS)
	  && ((AFNDCV > fSellV) || (((100 - FNDC_SOC) * 2) < ANBA))
	  && (AFNDCV >
	      MAX (50.8 +
		   MAX (0,
			((TEMPSENSOR (CENTER_LEFT) -
			  LE_SETPOINT_START) / LE_DIVISOR)), MIN (MAX (fSellV,
								       57),
								  fSellV +
								  0.8))))
	{
	  //lower tank temp in range -- turn on lower element, set delay, and return

	  if ((!overTemp) && (!GT_NoDrop))
	    {
	      LE_UP;
	      loadBalance = 0;
	      lcDelay = DEFAULT_DELAY;
	      logMesg ("LC @ %d LE==%d %4.2f %4.2f\n", __LINE__,
		       LE_PWM.percent, AFNDCV,
		       MAX (50.8 +
			    MAX (0,
				 ((TEMPSENSOR (CENTER_LEFT) -
				   LE_SETPOINT_START) / LE_DIVISOR)),
			    MIN (MAX (fSellV, 57), fSellV + 0.8)));
	      PRETURN;
	    }
	}
      //still more juce, turn on both elements
      if ((UE_PWM.percent < 100) && (topTank != irAbove) && (!overTemp) &&
	  ((TOT_LOAD (L1) + UPPER_L1_DIFF) < MAX_LOAD_AMPS)
	  && ((AFNDCV > 56.0) || (((100 - FNDC_SOC) * 2) < ANBA))
	  && (AFNDCV >
	      MAX (51.2 +
		   MAX (0,
			((TEMPSENSOR (TOP) -
			  UE_SETPOINT_START) / UE_DIVISOR)), fSellV + 1.2))
	  && (!GT_NoDrop))
	{
	  //upper tank temp in range -- turn on upper element, set delay, and return
	  UE_PWM.percent += (100 / MAX (1, UE_PWM.resolution));
	  lcDelay = DEFAULT_DELAY;
	  logMesg ("LC @ %d UE at %d, %4.2f %4.2f\n", __LINE__,
		   UE_PWM.percent, AFNDCV,
		   MAX (51.2 +
			MAX (0,
			     ((TEMPSENSOR (TOP) - UE_SETPOINT_START) / MAX (1,
									    UE_DIVISOR))),
			fSellV + 1.2));
	  loadBalance = 0;
	  PRETURN;
	}
      if ((digitalRead (AIR_COND_GPIO_PIN) == FALSE)
	  && (preferHeatPumpOn == FALSE)
	  && ((TOT_LOAD (L1) + AIR_COND_AMPS) < MAX_LOAD_AMPS)
	  && ((TOT_LOAD (L2) + AIR_COND_AMPS) < MAX_LOAD_AMPS)
	  && (AFNDCV >= 58.0) && (UnderUtilization))
	{
	  if (airCondControl (ON) == TRUE)
	    {
	      lcDelay = DEFAULT_DELAY;
	      logMesg ("LC @ %d AC_ON L1:%2dA L2:%2dA vgoal %3.1f\n",
		       __LINE__, TOT_LOAD (L1), TOT_LOAD (L2),
		       MIN (MAX
			    (5.2 + MAX (1, ((TEMPSENSOR (TOP) - 130) / 20)),
			     fSellV + 2.4), 56.8));
	      loadBalance = 0;
	      PRETURN;
	    }
	}
    }
  //check to see if using too much power and lower
//      else because it returns if any above is true
  if ((((!UnderUtilization) && (AFNDCV < MAX (51.2, MIN (57, (fSellV + 2.0))))
	&& (preferHeatPumpOn == FALSE)) && (!GT_NoDrop))
      || (loadBalance == 1))
    {
      if ((digitalRead (AIR_COND_GPIO_PIN)))
	{
	  if (airCondControl (OFF) == TRUE)
	    {
	      lcDelay = DEFAULT_DELAY;
	      logMesg ("LC @ %d AC_OFF %3.1f %4.2f\n", __LINE__, AFNDCV,
		       MAX (51.2, MIN (57, (fSellV + 2.0))));
	      loadBalance = 0;
	      PRETURN;
	    }
	}
    }
  if (((!UnderUtilization) /*&& (ANBA < 0) */  &&
       (((AFNDCV < VACATION_VSET) && (vacation == TRUE)) ||
	(AFNDCV <
	 MAX (51.2 +
	      MAX (0, ((TEMPSENSOR (TOP) - UE_SETPOINT_START) / UE_DIVISOR)),
	      MIN (57, fSellV + 1.2)))) && (!GT_NoDrop))
      || (loadBalance == 1))
    {
      if ((UE_PWM.percent > 0) && (topTank != irBelow))
	{
	  //upper tank temp in range -- turn off upper element, set delay, and return
	  UE_PWM.percent -= (100 / UE_PWM.resolution);
	  lcDelay = 1/*DEFAULT_DELAY*/;
	  logMesg ("LC @ %d UE_PWM.percent=%d %4.2f %4.2f\n", __LINE__,
		   UE_PWM.percent, AFNDCV,
		   MAX (51.2 +
			MAX (0,
			     ((TEMPSENSOR (TOP) -
			       UE_SETPOINT_START) / UE_DIVISOR)), MIN (57,
								       fSellV
								       +
								       1.2)));
	  loadBalance = 0;
	  PRETURN;
	}
    }
  if (((!UnderUtilization) /*&& (ANBA < 0) */  &&
       (((AFNDCV < (VACATION_VSET - 0.4)) && (vacation == TRUE)) ||
	(AFNDCV <
	 MAX (50.8 +
	      MAX (0,
		   ((TEMPSENSOR (CENTER_LEFT) -
		     LE_SETPOINT_START) / LE_DIVISOR)), MIN (MAX (fSellV, 57),
							     fSellV + 0.8))))
       && (!GT_NoDrop)) || (loadBalance == 1))
    {
      if (LE_IS_ON && (midTank != irBelow))
	{
	  if (ANBA >= ((-0.2) * LE_PWM.percent))
	    {
	      LE_DOWN;
	    }
	  else
	    {
	      LEOFF;
	    }
	  lcDelay = 1 /*DEFAULT_DELAY*/;
	  logMesg ("LC @ %d LE_PWM.percent==%d %4.2f %4.2f\n", __LINE__,
		   LE_PWM.percent, AFNDCV,
		   MAX (50.8 +
			MAX (0,
			     ((TEMPSENSOR (CENTER_LEFT) -
			       LE_SETPOINT_START) / LE_DIVISOR)),
			MIN (MAX (fSellV, 57), fSellV + 0.8)));
	  loadBalance = 0;
	  PRETURN;
	}
    }
  if (((!UnderUtilization) && (AFNDCV < fSellV) && (preferHeatPumpOn == TRUE)
       && (!GT_NoDrop)) || (loadBalance == 1))
    {
      if ((digitalRead (AIR_COND_GPIO_PIN)))
	{
	  if (airCondControl (OFF) == TRUE)
	    {
	      lcDelay = DEFAULT_DELAY;
	      logMesg ("LC @ %d AC_OFF %4.2f %4.2f\n", __LINE__, AFNDCV,
		       fSellV);
	      loadBalance = 0;
	      PRETURN;
	    }
	}
    }
  if ((vacation == TRUE) && (AFNDCV < (VACATION_VSET - 4.0)))
    setACPS (acpsNone);
  if (((!UnderUtilization)
       && ((AFNDCV < MAX ((fSellV - 2.0), 46.0))
	   || ((vacation == TRUE) && (AFNDCV < (VACATION_VSET - 3.0))))
       && (!GT_NoDrop) && (INVERTER_AC_MODE == iacmDrop
			   || INVERTER_AC_MODE == iacmNoGr))
      || ((loadBalance == 1) && (airJordanManOff == TRUE)))
    {
      if (AirJorPrefered (0) != 0)
	{
	  lcDelay = DEFAULT_DELAY;
	  logMesg ("LC @ %d  V=%6.3f  fSellv=%6.3f  loadBal=%d\n", __LINE__,
		   AFNDCV, fSellV, loadBalance);
	  loadBalance = 0;
	  PRETURN;
	}
    }
  PRETURN;
}
