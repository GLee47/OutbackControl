//LoadControl.c
#define __LoadControl_c__

#include "load_control.h"
#include "outback.h"
#include "usb_arduino.h"
#include <wiringPi.h>
#include <unistd.h>
#include <ncurses.h>
#include <time.h>

extern void compressor(int SwOn);
extern WINDOW * InvWin,* CCWin,* FNDCWin,* ScrollWin;
extern struct tm *tm_p;
extern bool preferHeatPumpOn;

#define	MAX_LOAD_AMPS	34
#define LOWER_HV_L1_DIFF	20
#define LOWER_HV_L2_DIFF	10
#define LOWER_LV_L2_DIFF	10
#define LOWER_LV_L1_DIFF	0
#define UPPER_L1_DIFF		10
#define UPPER_L2_DIFF		0
#define COMPRESSOR_L1_DIFF	11
#define COMPRESSOR_L2_DIFF	0
#define AIR_COND_AMPS		6

#define VACATION_VSET		56
#define LE_SETPOINT_START	120
#define	LE_DIVISOR			10
#define UE_SETPOINT_START	115
#define	UE_DIVISOR			10
#define AC_SETPOINT_START	135
#define	AC_DIVISOR			20


#define WHMAXTEMP (WHmaxAnyTemp-(100-FNDC_SOC))

#define DEFAULT_DELAY	3
#define WH_TOP_NORMAL_TEMP	120.0

#define DCamps(ACamps)	((float)(((ACamps*L1_INVERTER_VOUT)/FNDC_BATT_VOLTS)-3))
#define TEMPSENSOR(ID)	(readSensorF(WH_SENSOR_##ID##_INDX, 666))
#ifndef ON
	#define ON	1
#endif

#ifndef OFF
	#define OFF	0
#endif

enum inRange{irBelow=(-1), irIn=0, irAbove=1};

/*class device{
	char *name;
	int	L1load;
	int L2load;
};*/	
struct loadAmt{
	int	L1;
	int	L2;
};
/*	lower element LV is L2  10A
 * HV is L1 and L2 at 20A
 * upper element is L1 at 10A
 * compressor is 11A on L2
*/

int EstL1A, EstL2A, NeedToShedAmpsL1, NeedToShedAmpsL2;
static int lcDelay=DEFAULT_DELAY;
static int modeChangeDelay=4;

void syncACPS(void)
{
	AirCondPwrSrc=(digitalRead(MRCOOL2KHP_SRC_GPIO)==0)?acpsGrid:((digitalRead(MRCOOL2KHP_PWR_GPIO)==1)?acpsInverter:acpsNone);
}
int setACPS(enum AirCondPwrSrcModes newMode)
{
	static time_t acpsOffTime;
	if (difftime(time(NULL),acpsOffTime)<30)
	{ 
		if((newMode!=acpsNone) && (AirCondPwrSrc==acpsNone))return 0;
	}
	switch (newMode)
	{
		case acpsGrid:
			digitalWrite(MRCOOL2KHP_PWR_GPIO,0);
			digitalWrite(MRCOOL2KHP_SRC_GPIO,0);
			break;
		case acpsInverter:
			if(((TOT_LOAD(L1)+10)< MAX_LOAD_AMPS) && ((TOT_LOAD(L2)+10)< MAX_LOAD_AMPS))
			{
				digitalWrite(MRCOOL2KHP_SRC_GPIO,1);
				digitalWrite(MRCOOL2KHP_PWR_GPIO,1);
			}
			else
			{
				return 0;
			}
			break;
		case acpsNone:
			if(AirCondPwrSrc!=acpsNone)time(&acpsOffTime);
			digitalWrite(MRCOOL2KHP_PWR_GPIO,0);
			digitalWrite(MRCOOL2KHP_SRC_GPIO,1);
			break;
		case acpsOn:
			if(((TOT_LOAD(L1)+10)< MAX_LOAD_AMPS) && ((TOT_LOAD(L2)+10)< MAX_LOAD_AMPS))
			{
				digitalWrite(MRCOOL2KHP_SRC_GPIO,1);
				digitalWrite(MRCOOL2KHP_PWR_GPIO,1);
			}
			else
			{
				digitalWrite(MRCOOL2KHP_PWR_GPIO,0);
				digitalWrite(MRCOOL2KHP_SRC_GPIO,0);
			}
			break;
	}
	syncACPS();
	return ((AirCondPwrSrc==newMode)||((newMode==acpsOn) && (AirCondPwrSrc!=acpsNone)));
}

#define ACC_ON_NOW	3	
#define ACC_OFF_NOW	2	
int airCondControl(int targetState)
{
	static time_t tChangeTime=0;
//	time_t tNow=time(NULL);
	
	if ((difftime(time(NULL),tChangeTime)>200.0) || (targetState>ON))
	{
		if((targetState==ON) || (targetState==ACC_ON_NOW))  //ON=turn On if time has elapsed /3 means now
		{
			if (digitalRead(AIR_COND_GPIO_PIN)==OFF)
			{
				digitalWrite(AIR_COND_GPIO_PIN,ON);
				time(&tChangeTime);
				return TRUE;
			}
		}
		else if((targetState==OFF) || (targetState==ACC_OFF_NOW)) // OFF is turn Off if time has elapsed / 2 means now
		{
			if(digitalRead(AIR_COND_GPIO_PIN)==ON)
			{
				digitalWrite(AIR_COND_GPIO_PIN,OFF);
				time(&tChangeTime);
				return TRUE;
			}
		}
		return FALSE;
	}
	return FALSE;
}

//load shed will seek overload conditions and reduce loads
void loadShed(void)
{
	syncACPS();
	if((INVERTER_AC_MODE!=iacmDrop) && (INVERTER_AC_MODE!=iacmNoGr))
		modeChangeDelay=4; 
	else 
		modeChangeDelay--;
	if(lcDelay>0)
	{
		lcDelay--;
		return;
	}
	EstL1A = TOT_LOAD(L1);//(L1_INVERTER_AMPS+L1_BUY_AMPS);
	EstL2A = TOT_LOAD(L2);//(L2_INVERTER_AMPS+L2_BUY_AMPS);
	if(EstL1A > MAX_LOAD_AMPS){ //got to shed something on L1 (lower element is L2)
		NeedToShedAmpsL1 = EstL1A - MAX_LOAD_AMPS;
/*		if((digitalRead(WH_LOWER_ELEMENT)==ON) && (NeedToShedAmpsL1>LOWER_LV_L1_DIFF)){
			if((digitalRead(WH_LOWER_ELEMENT_HV)==ON)&&(digitalRead(WH_LOWER_ELEMENT)==ON)){
				digitalWrite(WH_LOWER_ELEMENT_HV,OFF); wprintw(ScrollWin,"LC @ %d\n",__LINE__);
				NeedToShedAmpsL1-=LOWER_HV_L1_DIFF;
				EstL1A-= LOWER_LV_L1_DIFF;
				lcDelay=DEFAULT_DELAY;
			}
		}***this is HV control and needs fixed if I get the hardware hooked up*/
		if((digitalRead(WH_UPPER_ELEMENT)==ON) && (NeedToShedAmpsL1>0))
		{
				digitalWrite(WH_UPPER_ELEMENT,OFF); wprintw(ScrollWin,"LC @ %d\n",__LINE__);
				NeedToShedAmpsL1-=UPPER_L1_DIFF;
				EstL1A-= UPPER_L1_DIFF;
				lcDelay=DEFAULT_DELAY;
		}
		if((NeedToShedAmpsL1>0) && (AirCondPwrSrc==acpsInverter))//heat pump--we don't know if a load exists nor the size of it
		{
			setACPS((vacation==false)?acpsGrid:acpsNone);lcDelay=DEFAULT_DELAY;
			NeedToShedAmpsL1-=5;//We don't know what, if anything, was reduced. Maybe the 5A guess will delay futher triggers
			wprintw(ScrollWin,"LC @ %d  ACPS %s NTSAL1 %d EL1A %d\n",__LINE__,acpsModeDesc[AirCondPwrSrc],NeedToShedAmpsL1,EstL1A);
		}
		if((NeedToShedAmpsL1>0) && (INVERTER_AUX_OUT!=0))
		{
			compressor(OFF); wprintw(ScrollWin,"LC @ %d\n",__LINE__);					
			NeedToShedAmpsL1-= COMPRESSOR_L1_DIFF;
			EstL1A-= COMPRESSOR_L1_DIFF;
			lcDelay=DEFAULT_DELAY;
		}	
		if((NeedToShedAmpsL1>0) && (digitalRead(AIR_COND_GPIO_PIN)==ON))//this is the window unit in old living room
		{
			airCondControl(ACC_OFF_NOW); wprintw(ScrollWin,"LC @ %d\n",__LINE__);					
			NeedToShedAmpsL1-= AIR_COND_AMPS;
			EstL1A-= AIR_COND_AMPS;
			lcDelay=DEFAULT_DELAY;
		}	
	}
	if(EstL2A > MAX_LOAD_AMPS)
	{ //got to shed something on L2
		NeedToShedAmpsL2 = EstL2A - MAX_LOAD_AMPS;
/*		if((digitalRead(WH_LOWER_ELEMENT_HV)==ON)&&(digitalRead(WH_LOWER_ELEMENT)==ON))
 		{
			digitalWrite(WH_LOWER_ELEMENT_HV,OFF); wprintw(ScrollWin,"LC @ %d\n",__LINE__);
			NeedToShedAmpsL2-=LOWER_HV_L2_DIFF;
			EstL2A-= LOWER_HV_L2_DIFF;
			lcDelay=DEFAULT_DELAY;
		}*/
		if((NeedToShedAmpsL2>0) && (digitalRead(WH_LOWER_ELEMENT)==ON))
		{
			digitalWrite(WH_LOWER_ELEMENT,OFF); wprintw(ScrollWin,"LC @ %d\n",__LINE__);
			NeedToShedAmpsL2-=LOWER_LV_L2_DIFF;
			EstL2A-= LOWER_LV_L2_DIFF;
			lcDelay=DEFAULT_DELAY;
		}
		if((NeedToShedAmpsL2>0) && (AirCondPwrSrc==acpsInverter))
		{
			setACPS((vacation==false)?acpsGrid:acpsNone);lcDelay=DEFAULT_DELAY;
			NeedToShedAmpsL2-=5;//We don't know what, if anything, was reduced. Maybe the 5A guess will delay futher triggers
			wprintw(ScrollWin,"LC @ %d  ACPS %s NTSL2A %d EL2A %d\n",__LINE__,acpsModeDesc[AirCondPwrSrc],NeedToShedAmpsL2,EstL2A);
		}	
		if((NeedToShedAmpsL2>0) && (digitalRead(AIR_COND_GPIO_PIN)==ON))
		{
			airCondControl(ACC_OFF_NOW); wprintw(ScrollWin,"LC @ %d\n",__LINE__);					
			NeedToShedAmpsL2-= AIR_COND_AMPS;
			EstL2A-= AIR_COND_AMPS;
			lcDelay=DEFAULT_DELAY;
		}	
	}
	if(lcDelay==0) LoadControl();
}
	
void LoadControl(void)
{
	int overTemp=(MAX(MAX(TEMPSENSOR(TOP),TEMPSENSOR(BOTTOM)),MAX(TEMPSENSOR(CENTER_RIGHT),TEMPSENSOR(CENTER_LEFT))) > WHMAXTEMP);
	float midTankTemp=MAX(TEMPSENSOR(CENTER_RIGHT),TEMPSENSOR(CENTER_LEFT));
	enum inRange topTank=(TEMPSENSOR(TOP) < WHtopMinTemp ? irBelow : ((TEMPSENSOR(TOP) > WHtopMaxTemp) ? irAbove :irIn));
	enum inRange midTank=(midTankTemp < WHCenterMinTemp ? irBelow : (midTankTemp > WHMAXTEMP ? irAbove :irIn));
	static enum InverterACModes lastIACMode=iacmNoGr;
//	wprintw(ScrollWin,"%d %d %d %d\n",INVERTER_AUX_OUT,TOT_LOAD(L2),COMPRESSOR_L2_DIFF,MAX_LOAD_AMPS);
//	wprintw(ScrollWin,"%5.1f %5.1f %d %d\n",netbattamps, DCamps(LOWER_LV_L2_DIFF),(digitalRead(WH_LOWER_ELEMENT)==OFF),midTank);
	if(INVERTER_AUX_OUT==0)
	{
		if((TOT_LOAD(L1) + COMPRESSOR_L1_DIFF) < MAX_LOAD_AMPS)
		{
			compressor(ON); wprintw(ScrollWin,"LC @ %d\n",__LINE__);
			lcDelay=DEFAULT_DELAY;
			EstL2A += COMPRESSOR_L1_DIFF;
		}
	}
	if(overTemp)
	{
		//turn off all elements , set delay and return
		if((digitalRead(WH_LOWER_ELEMENT)==ON) || (digitalRead(WH_UPPER_ELEMENT)==ON))
		{
			digitalWrite(WH_LOWER_ELEMENT,OFF);
			digitalWrite(WH_UPPER_ELEMENT,OFF);
			lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d\n",__LINE__);
			return;
		}
	}
	if((digitalRead(WH_LOWER_ELEMENT)==OFF) && (midTank == irBelow) && (INVERTER_AC_MODE!=iacmNoGr)
							&& ((TOT_LOAD(L2)+LOWER_LV_L2_DIFF)< MAX_LOAD_AMPS))
	{
		//lower tank below min temp -- turn on lower element, set delay, and return
		if(digitalRead(WH_LOWER_ELEMENT_HV)==ON)
		{
			digitalWrite(WH_LOWER_ELEMENT_HV,OFF); wprintw(ScrollWin,"LC @ %d\n",__LINE__);
			usleep(35000);
		}
		digitalWrite(WH_LOWER_ELEMENT,ON);
		lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d\n",__LINE__);
		return;
	}
	if((digitalRead(WH_UPPER_ELEMENT)==OFF) && (topTank == irBelow) && (INVERTER_AC_MODE!=iacmNoGr) 
							&& ((TOT_LOAD(L1)+UPPER_L1_DIFF)< MAX_LOAD_AMPS))
	{
		//upper tank temp below range -- turn on upper element, set delay, and return
		digitalWrite(WH_UPPER_ELEMENT,ON);
		lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d\n",__LINE__);
		return; 
	}
	if ((digitalRead(AIR_COND_GPIO_PIN)==FALSE) && (preferHeatPumpOn==TRUE) && (INVERTER_AC_MODE!=iacmNoGr) 
				&& ((TOT_LOAD(L1)+AIR_COND_AMPS)< MAX_LOAD_AMPS) && ((TOT_LOAD(L2)+AIR_COND_AMPS)< MAX_LOAD_AMPS))
	{
		if(airCondControl(ON)==TRUE)
		{ 
			lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d\n",__LINE__);		
			return;	
		}
	}
	if((INVERTER_AC_MODE==iacmNoGr)) //grid down 
	{ 
		if(lastIACMode!=iacmNoGr)
		{
			lastIACMode=iacmNoGr;
			if(AirCondPwrSrc==acpsInverter)	setACPS(acpsGrid);
		}
		if(FNDC_BATT_VOLTS < 54)
		{
		//turn one element off, set delay , and return
			if(digitalRead(WH_LOWER_ELEMENT)==ON)	//favoring upper element to get some hot quick
			{
				digitalWrite(WH_LOWER_ELEMENT,OFF);
				lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d\n",__LINE__);
				return;
			}
			if(digitalRead(WH_UPPER_ELEMENT)==ON)
			{
				digitalWrite(WH_UPPER_ELEMENT,OFF);
				lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d\n",__LINE__);
				return;
			}
			if (digitalRead(AIR_COND_GPIO_PIN))
			{
				if(airCondControl(ACC_OFF_NOW)==TRUE)
				{
					lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d\n",__LINE__);		
					return;	
				}
			}
		}
		else if (FNDC_BATT_VOLTS > 56)
		{
			if ((digitalRead(AIR_COND_GPIO_PIN)==FALSE) && (preferHeatPumpOn==TRUE) 
				&& ((TOT_LOAD(L1)+AIR_COND_AMPS)< MAX_LOAD_AMPS) && ((TOT_LOAD(L2)+AIR_COND_AMPS)< MAX_LOAD_AMPS))
			{
				if(airCondControl(ON)==TRUE)
				{ 
					lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d\n",__LINE__);		
					return;	
				}
			}
			if(digitalRead(WH_UPPER_ELEMENT)!=ON)	//favoring upper element to get some hot quick	
			{
				digitalWrite(WH_UPPER_ELEMENT,ON);
				lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d\n",__LINE__);
				return;
			}
			if(digitalRead(WH_LOWER_ELEMENT)!=ON)
			{
				digitalWrite(WH_LOWER_ELEMENT,ON);
				lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d\n",__LINE__);
				return;
			}			
		}
		return;
	}
	else if(INVERTER_AC_MODE==iacmUse)  //using grid -- minimize use
	{
		lastIACMode=iacmUse;
		if((vacation==TRUE)&&(AirCondPwrSrc!=acpsNone))
		{
			if(setACPS(acpsNone))wprintw(ScrollWin,"LC @ %d acps off\n",__LINE__);
		}
		if (digitalRead(AIR_COND_GPIO_PIN) && (preferHeatPumpOn==FALSE))
		{
			if(airCondControl(ACC_OFF_NOW)==TRUE)
			{
				lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d\n",__LINE__);		
				return;	
			}
		}
		if(topTank != irBelow)
		{
			//turn off top element
			if(digitalRead(WH_UPPER_ELEMENT)==ON)
			{
				wprintw(ScrollWin,"LC @ %d\n",__LINE__);
				digitalWrite(WH_UPPER_ELEMENT,OFF);
			}
		}else{
			//turn on top element
			if((digitalRead(WH_UPPER_ELEMENT)==OFF) && ((EstL1A+UPPER_L1_DIFF) <= MAX_LOAD_AMPS) && (INVERTER_AUX_OUT!=0))
			{
				wprintw(ScrollWin,"LC @ %d\n",__LINE__);
				digitalWrite(WH_UPPER_ELEMENT,ON);
			}
		}
		if(midTank != irBelow)
		{
			// turn off bottom element
			if(digitalRead(WH_LOWER_ELEMENT)==ON)
			{
				wprintw(ScrollWin,"LC @ %d\n",__LINE__);
				digitalWrite(WH_LOWER_ELEMENT,OFF);
			}
		}else{
			//turn on bottom element
			if((digitalRead(WH_LOWER_ELEMENT)==OFF) && ((EstL2A+LOWER_LV_L2_DIFF) <= MAX_LOAD_AMPS))
			{
				wprintw(ScrollWin,"LC @ %d\n",__LINE__);
				digitalWrite(WH_LOWER_ELEMENT,ON); 
			}
		}
		if(vacation==TRUE)setACPS(acpsNone);
		return;
	}
	
	//we haven't returned yet we must be in dropped grid mode -- try to manage charge volts
	//(INVERTER_AC_MODE==iacmDrop)
	
	lastIACMode=INVERTER_AC_MODE;
	if(modeChangeDelay>0)
	{ /*wprintw(ScrollWin,"LC @ %d\n",__LINE__);*/
		return;
	}
	
	if((UnderUtilization)||((vacation==FALSE)&&(netbattamps > DCamps(LOWER_LV_L2_DIFF)))
				||((vacation==FALSE)&&(FNDC_BATT_VOLTS > MAX(54.0,fSellV)))||((vacation==TRUE)&&(FNDC_BATT_VOLTS>VACATION_VSET)))
	{								//excess power. Find something to turn on
		if((vacation==TRUE)&&(AirCondPwrSrc!=acpsInverter))
		{
			if(setACPS(acpsInverter))
			{
				lcDelay=30;wprintw(ScrollWin,"LC @ %d acps inverter\n",__LINE__);
			}
		}
		if ((digitalRead(AIR_COND_GPIO_PIN)==FALSE) && (preferHeatPumpOn==TRUE) 
				&& ((TOT_LOAD(L1)+AIR_COND_AMPS)< MAX_LOAD_AMPS) && ((TOT_LOAD(L2)+AIR_COND_AMPS)< MAX_LOAD_AMPS))
		{
			if(airCondControl(ON)==TRUE)
			{ 
				lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d AC on (prefered)\n",__LINE__);		
				return;	
			}
		}
		if ((AirCondPwrSrc==acpsGrid) && ((TOT_LOAD(L1)+10)< MAX_LOAD_AMPS) && ((TOT_LOAD(L2)+10)< MAX_LOAD_AMPS))
		{
			setACPS(acpsInverter);
			lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d  ACPS %s\n",__LINE__,acpsModeDesc[AirCondPwrSrc]);
		}
		//Top<120 start here
		if((TEMPSENSOR(TOP) < WH_TOP_NORMAL_TEMP) && (digitalRead(WH_UPPER_ELEMENT)==OFF) && (topTank != irAbove) && (!overTemp) && 
				(((TOT_LOAD(L1)+UPPER_L1_DIFF)< MAX_LOAD_AMPS) || (FNDC_BATT_VOLTS > 56.0)) && (FNDC_BATT_VOLTS > MAX(50.7,fSellV+0.6)))
		{
			//upper tank temp in range -- turn on upper element, set delay, and return
			digitalWrite(WH_UPPER_ELEMENT,ON);
			lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d UE_ON %4.2f %4.2f\n",__LINE__,FNDC_BATT_VOLTS,MAX(50.7,fSellV+0.6));
			return;
		}
		//top >120 or we got more juce them bring up bottom temp
		if((digitalRead(WH_LOWER_ELEMENT)==OFF) && (midTank != irAbove) && 
				(((TOT_LOAD(L2)+LOWER_LV_L2_DIFF)< MAX_LOAD_AMPS) || (FNDC_BATT_VOLTS > 56.0)) && 
				(FNDC_BATT_VOLTS > MAX(50.8 + MAX(0,((TEMPSENSOR(CENTER_LEFT)-LE_SETPOINT_START)/LE_DIVISOR)),MIN(57,fSellV+0.8)/*fSellV+0.4*/)))
		{
			//lower tank temp in range -- turn on lower element, set delay, and return
			if(digitalRead(WH_LOWER_ELEMENT_HV)==ON)
			{
				digitalWrite(WH_LOWER_ELEMENT_HV,OFF); wprintw(ScrollWin,"LC @ %d\n",__LINE__);
				usleep(35000);
			}
			if (!overTemp){
				digitalWrite(WH_LOWER_ELEMENT,ON);
				lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d LE_ON %4.2f %4.2f\n",__LINE__,FNDC_BATT_VOLTS,
							MAX(50.8+ MAX(0,((TEMPSENSOR(CENTER_LEFT)-LE_SETPOINT_START)/LE_DIVISOR)),MIN(57,fSellV+0.8)));
				return;
			}
		}
		//still more juce, turn on both elements
		if((digitalRead(WH_UPPER_ELEMENT)==OFF) && (topTank != irAbove) && (!overTemp) &&
				(((TOT_LOAD(L1)+UPPER_L1_DIFF)< MAX_LOAD_AMPS) || (FNDC_BATT_VOLTS > 56.0)) && 
				(FNDC_BATT_VOLTS > MAX(51.2 + MAX(0,((TEMPSENSOR(TOP)-UE_SETPOINT_START)/UE_DIVISOR)),fSellV+1.2)))
		{
			//upper tank temp in range -- turn on upper element, set delay, and return
			digitalWrite(WH_UPPER_ELEMENT,ON);
			lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d UE_ON %4.2f %4.2f\n",__LINE__,FNDC_BATT_VOLTS,
							MAX(51.2 + MAX(0,((TEMPSENSOR(TOP)-UE_SETPOINT_START)/UE_DIVISOR)),fSellV+1.2));
			return;
		}
		if ((digitalRead(AIR_COND_GPIO_PIN)==FALSE) && (preferHeatPumpOn==FALSE)
				&& ((TOT_LOAD(L1)+AIR_COND_AMPS)< MAX_LOAD_AMPS) && ((TOT_LOAD(L2)+AIR_COND_AMPS)< MAX_LOAD_AMPS)
				&& (FNDC_BATT_VOLTS >= 58.0/*MIN(MAX(53.2 + MAX(1,((TEMPSENSOR(TOP)-130)/12)),fSellV+2.4),56.8)*/))
		{
			if(airCondControl(ON)==TRUE)
			{ 
				lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d AC_ON L1:%2dA L2:%2dA vgoal %3.1f\n",__LINE__,TOT_LOAD(L1),TOT_LOAD(L2),
								MIN(MAX(5.2 + MAX(1,((TEMPSENSOR(TOP)-130)/20)),fSellV+2.4),56.8));		
				return;	
			}
		}
	}
	//check to see if using too much power and lower
//	else 
	if((!UnderUtilization) && (FNDC_BATT_VOLTS < MAX(51.2, MIN(57,(fSellV+2.0)))) && (preferHeatPumpOn==FALSE)) 
	{
		if ((digitalRead(AIR_COND_GPIO_PIN)))
		{
			if(airCondControl(OFF)==TRUE)
			{
				lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d AC_OFF %3.1f %4.2f\n",__LINE__,FNDC_BATT_VOLTS,
												MAX(51.2, MIN(57,(fSellV+2.0))));		
				return;	
			}
		}
	}
	if((!UnderUtilization) /*&& (netbattamps < 0)*/ && 
					(((FNDC_BATT_VOLTS<VACATION_VSET)&&(vacation==TRUE))||
					(FNDC_BATT_VOLTS < MAX(51.2 + MAX(0,((TEMPSENSOR(TOP)-UE_SETPOINT_START)/UE_DIVISOR)),MIN(57,fSellV+1.2))))) 
	{
		if((digitalRead(WH_UPPER_ELEMENT)==ON) && (topTank != irBelow))
		{
			//upper tank temp in range -- turn off upper element, set delay, and return
			digitalWrite(WH_UPPER_ELEMENT,OFF);
			lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d UE_OFF %4.2f %4.2f\n",__LINE__,FNDC_BATT_VOLTS,
						MAX(51.2 + MAX(0,((TEMPSENSOR(TOP)-UE_SETPOINT_START)/UE_DIVISOR)),MIN(57,fSellV+1.2)));
			return;
		}
	}
	if((!UnderUtilization) /*&& (netbattamps < 0)*/ && 
				(((FNDC_BATT_VOLTS<(VACATION_VSET-0.4))&&(vacation==TRUE))||
				(FNDC_BATT_VOLTS < MAX(50.8+ MAX(0,((TEMPSENSOR(CENTER_LEFT)-LE_SETPOINT_START)/LE_DIVISOR)),MIN(57,fSellV+0.8))))) 
	{
		if((digitalRead(WH_LOWER_ELEMENT)==ON) && (midTank != irBelow))
		{
			//lower tank temp in range -- turn off lower element, set delay, and return
			if(digitalRead(WH_LOWER_ELEMENT_HV)==ON)
			{
				digitalWrite(WH_LOWER_ELEMENT_HV,OFF); wprintw(ScrollWin,"LC @ %d LEHV_OFF %4.2f %4.2f\n",__LINE__,FNDC_BATT_VOLTS,
							MAX(50.8+ MAX(0,((TEMPSENSOR(CENTER_LEFT)-LE_SETPOINT_START)/LE_DIVISOR)),MIN(57,fSellV+0.8)));
				usleep(35000);
			}
			digitalWrite(WH_LOWER_ELEMENT,OFF);
			lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d LE_OFF %4.2f %4.2f\n",__LINE__,FNDC_BATT_VOLTS,
							MAX(50.8+ MAX(0,((TEMPSENSOR(CENTER_LEFT)-LE_SETPOINT_START)/LE_DIVISOR)),MIN(57,fSellV+0.8))); 
			return;
		}
	}
	if((!UnderUtilization) && (FNDC_BATT_VOLTS < fSellV) && (preferHeatPumpOn==TRUE)) 
	{
		if ((digitalRead(AIR_COND_GPIO_PIN)))
		{
			if(airCondControl(OFF)==TRUE)
			{
				lcDelay=DEFAULT_DELAY; wprintw(ScrollWin,"LC @ %d AC_OFF %4.2f %4.2f\n",__LINE__,FNDC_BATT_VOLTS,fSellV);		
				return;	
			}
		}
	}
	if((vacation==TRUE)&&(FNDC_BATT_VOLTS<(VACATION_VSET-1.6)))setACPS(acpsNone);
}
