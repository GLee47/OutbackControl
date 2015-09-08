//wh.c  water heater

#include "usb_arduino.h"
#include <wiringPi.h>

#define MAXLOAD	45
#define WHAUXLOAD 10
#define WHRELAYLOAD	20
#define BotElem120vLegLoad	L2Load
#define TopElem120vLegLoad	L1Load

#define WH_LOWER_ELEMENT	26
#define WH_LOWER_ELEMENT_HV	12
#define WH_UPPER_ELEMENT	16

#ifndef ON
	#define ON	1
#endif

#ifndef OFF
	#define OFF	0
#endif

#define MAX(ARG1,ARG2)		(((ARG1)>(ARG2)) ? (ARG1) : (ARG2))
#define MIN(ARG1,ARG2)		(((ARG1)<(ARG2)) ? (ARG1) : (ARG2))

extern float WHtopMaxTemp,WHtopMinTemp,WHCenterMinTemp;										

enum whSwitch{ whsOff, whsOn, whsUnset, whsInquiry};
enum whsFlags{ whsManual, whsAuto, whsTimer, whsDisableTimer, whsConserve};
enum whsElement{ whsTop, whsBottom };
enum whSwitch Manual=whsUnset, Auto=whsUnset, Timer=whsUnset, DisableTimer=whsUnset, Conserve=whsUnset;//, OverLoad=whsOff; 
enum whSwitch whsSetFlags(enum whsFlags Flag, enum whSwitch Setting){
	switch (Flag){
		case whsManual:
			if (Setting==whsInquiry) return (Manual); else Manual=Setting;
			break;
		case whsAuto:
			if (Setting==whsInquiry) return (Auto); else Auto=Setting;
			break;
		case whsTimer:
			if (Setting==whsInquiry) return (Timer); else Timer=Setting;
			break;
		case whsDisableTimer:
			if (Setting==whsInquiry) return (DisableTimer); else DisableTimer=Setting;
			break;
		case whsConserve:
			if (Setting==whsInquiry) return (Conserve); else Conserve=Setting;
	}
	return (Setting);
}	

int IsOverLoad(/*int Aux, int Relay,*/ int L1Load, int L2Load, enum whsElement Ele){
//int	L1WHLoad=Aux*((Relay==0) ? WHAUXLOAD:WHRELAYLOAD);
//int	L2WHLoad=Relay*WHRELAYLOAD;
	if ((L1Load>MAXLOAD) || (L2Load>MAXLOAD)) return 2;
	if (digitalRead(WH_LOWER_ELEMENT)==0){
		if(Ele==whsBottom){
			if ((BotElem120vLegLoad+WHAUXLOAD)>MAXLOAD){
				return 1;
			}else if ((digitalRead(WH_LOWER_ELEMENT_HV)!=0) && ((((L1Load+WHRELAYLOAD)>MAXLOAD)) || ((L2Load+WHRELAYLOAD)>MAXLOAD))){
				return 1;
			}
		}else{	//top
			if ((TopElem120vLegLoad+WHAUXLOAD)>MAXLOAD){
				return 1;
			}else if ((digitalRead(WH_LOWER_ELEMENT_HV)!=0) && ((((L1Load+WHRELAYLOAD)>MAXLOAD)) || ((L2Load+WHRELAYLOAD)>MAXLOAD))){
				return 1;
			}
		}
	}
	return 0;
}
enum whSwitch whGetDesiredState(/*int Aux, int Relay,*/ int L1Load, int L2Load){
static enum whSwitch LastStatus=whsOff;	
	if ((Conserve==whsOn) || (MAX(MAX(readSensorF(2,666),readSensorF(0,666)),MAX(readSensorF(1,666),readSensorF(4,666))) > 170.0)){
		//digitalWrite(WH_UPPER_ELEMENT,OFF);
		return (LastStatus=whsOff);
	}
	if(readSensorF(2,666) < WHtopMinTemp){ //top sensor < min req
		if (IsOverLoad(/*digitalRead(WH_LOWER_ELEMENT),digitalRead(WH_LOWER_ELEMENT_HV),*/L1Load,L2Load,whsTop)==0){
			//digitalWrite(WH_UPPER_ELEMENT,ON);
			return (LastStatus=whsOff);
		}else{
			//digitalWrite(WH_UPPER_ELEMENT,OFF);
			return (LastStatus=whsOn);
		}
	}
	switch (Manual){
		case whsUnset:
			if (IsOverLoad(/*digitalRead(WH_LOWER_ELEMENT),digitalRead(WH_LOWER_ELEMENT_HV),*/L1Load,L2Load,whsBottom)==0){
//				if((Relay!=0)&&(LastStatus==whsOn))return (LastStatus=whsOn); //will hold aux on if relay is on
				if ((Timer!=whsOn)||(DisableTimer==whsOn)){
					if (Auto!=whsOn){	
						if(readSensorF(2,666) < WHtopMinTemp){ //top sensor < min req
							if (IsOverLoad(/*digitalRead(WH_LOWER_ELEMENT),digitalRead(WH_LOWER_ELEMENT_HV),*/L1Load,L2Load,whsTop)==0){
								//digitalWrite(WH_UPPER_ELEMENT,ON);
								return (LastStatus=whsOff);
							}else{
								//digitalWrite(WH_UPPER_ELEMENT,OFF);
								return (LastStatus=whsOn);
							}
						}else{ //top sensor > min requirement
							//digitalWrite(WH_UPPER_ELEMENT,OFF);
						}	
						if(MAX(MAX(readSensorF(1,666),readSensorF(4,666)),readSensorF(0,666)) < WHCenterMinTemp) return (LastStatus=whsOn);
						return (LastStatus=whsOff);
					}	
					else {	// Auto is on
						return (LastStatus=whsOn); 
					}
				}else { //Timer is On
					if (DisableTimer!=whsOn){ //Timer is not disabled
						return (LastStatus=whsOn);
					}else{ //Timer is disabled
						return (LastStatus=whsOff);
					}
				}
			}else { //OverLoad is On
				return (LastStatus=whsOff);
			}
			break;
		case whsOn:
			if (IsOverLoad(/*digitalRead(WH_LOWER_ELEMENT),digitalRead(WH_LOWER_ELEMENT_HV),*/L1Load,L2Load,whsBottom)==0){ 
				return (LastStatus=whsOn);
			}
			return (LastStatus=whsOff);
			break;
		case whsOff:	
			return (LastStatus=whsOff);
			break;	
		case whsInquiry:
			return (LastStatus=whsInquiry);
	}
	return (LastStatus=whsUnset);
}
	
