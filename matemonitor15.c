//MateMonitor.c
#define __MateMonitor_c__

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ncurses.h>
#include <stdarg.h>
#include "outback.h"
#include "usb_com.h"
#include "usb_arduino.h"
#include "gdlTimer.h"
#include <termios.h>
#include <fcntl.h>
#include <sqlite3.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <wiringPi.h>
#include <softPwm.h>
#include "load_control.h"

//these need to be an include file for wh.c
enum whSwitch{ whsOff, whsOn, whsUnset, whsInquiry};
enum whsFlags{ whsManual, whsAuto, whsTimer, whsDisableTimer, whsConserve};
enum whSwitch whsSetFlags(enum whsFlags Flag, enum whSwitch Setting);
enum whSwitch whGetDesiredState(/*int Aux, int Relay,*/ int L1Load, int L2Load);
//**************

//needs to be include file for SMS.c
extern int SMS_SendMsg(char * PhNum, char * Msg);
struct SMS_Message_Properties{
	char	PhoneNum[0x10];
	char	MsgText[0x100];
	time_t	SentTime;
	int		MinSecRepeat;
};
extern char *SMS_Rec_Mesg(/*char*/ struct SMS_Message_Properties *buff);
extern int airCondControl(int targetState);
//
 
//#include <sqlite3ext.h>
//sqlite3.c /lib/i386-linux-gnu/libdl.so.2
#define ALARM_TIMMER(DH,reAWT,reqTrigs,reqTWT)	static struct timer_data DH;\
												static int timerInitilized=0;\
												if(timerInitilized!=1) \
													TimerInit(&DH, reAWT,reqTrigs,reqTWT);\
												timerInitilized=1;\
												if (TriggerTimer(&DH))

int SMS_doSendStatRpt=NO;
#define SEND_SMS_STAT_RPT	if(SMS_doSendStatRpt!=NO){\
					sprintf(strtmp,"Hr:%d V:%3.1f\nSOC:%4.1d\nWH:%3.1f\nBuy:%2.0f IKWH:%2.0f\n"\
					"CC %4.1f %4.1f %4.1f\nDrp Min %4d\nNoGr Min %4d\nabs Min %4d"\
					"\nSOC S %3d E %3d"\
					,hour, FNDC_BATT_VOLTS, FNDC_SOC,WaterHeaterKWH,BuyWH/1000.0,InvWH/1000.0\
					,CC1_KWHH,CC2_KWHH,CC1_KWHH+CC2_KWHH, (DroppedSecs/60),(PowerOutageSecs/60)\
					,(UnderUtilizationSecs/60), SOCstart, FNDC_SOC);\
					logMesg("%d Send returned %d - %s\n",__LINE__,SMS_SendMsg("2814509680",strtmp),strtmp);}

enum CC_Modes{ Silent=0, Float=1, Bulk=2, Absorb=3, Equlize=4 };
enum InvInputModes /*{	Gen, Support, GridTied, UPS, Backup, MiniGrid }*/ InvInputMode=Support;
const char * InvInputModeLabels[] = {"Gen", "Support","GridTied","UPS","Backup","MiniGrid"};
const char * ACMODES[] = {"NoGr","Drop","Use "};
char * OPMODES[93] = {"InOff","Searc","InvOn","Chrge","Silnt","Float","Eqlze","ChOff","Suppt","SelEn"\
							,"PasTh","11","12","13","OffSt"};
const char * CCModes[] = {"Silnt","Float","Bulk","Absrb","Equlz"};
const char * Programs[] = {"Conserve        ","Storm           ","90% 04:00-16:00 ","Const SOC 80%   "
							,"70% 02:00-16:00 ","Volt Test 50.0  ","Volt Test 48.8  ","Volt Test 48.0  "
							,"Volt fac2 49.0  ","Mild Dmnd&Sun Md","Mild Demand&Sun ","11 Modified 3   "};
int ProgramIndx=0;
#define	PrgMaxIndx	11
#define	INVERTER	1
#define	GRID		0
unsigned long Ticks=0/*roughly = seconds since program comenced*/,auxRelayOffTicks=0;
const float ArrayRatedWatts = 245.0*30.0, BatRatedAmpHr = 225.0/*rating*/ * 1.5/*# of strings*/;
#define ADJNETBATTAMPS	(netbattamps*(450/BatRatedAmpHr))

char  strtmp[256];

WINDOW * InvWin,* CCWin,* FNDCWin,* ScrollWin;
int ch;
volatile bool terminate=false;

bool initilized=NO, EnableSystem=NO, DischargeAllowed=YES, AllowDaytimeInvCharge=FALSE,BathDone=FALSE,wh_le_src=INVERTER,
				preferHeatPumpOn=FALSE,InvChrgEnabled=NO, MateCommResponseExpected=NO, InvIgnLowVolt=false;
int CurrentDay, DroppedSecs=0, PowerOutageSecs=0, UnderUtilizationSecs=0;
long IAO_Day_Secs, IAR_Day_Secs;
int AC1InLimit=45, BatTrgV=512, SOCstart=100;
float BatAmpHrIn=0.0, BatAmpHrOut=0.0, BatAmpHrCrx=0.0, MaxVNegAmps=0.0,BattEffFac=0.91,ChargeDisableTrigV=51.0;
float SellWH,BuyWH,InvWatts,InvWH, netbattamps=0.0, BatTrgAmp=0.0, MaxNegBatAmpsDropped=(-MAX_NEG_NBA_DROPPED);
float DaysSinceFull=999.9;
float WHtopMaxTemp=180.1,WHtopMinTemp=100.0,WHCenterMinTemp=50.0,WHmaxAnyTemp=185.0,WhLEGridGoal=50.0;
//float MrCoolOnTemp=MR_COOL_ON_TEMP_DEFAULT,MrHeatOnTemp=MR_HEAT_ON_TEMP_DEFAULT,
float	MC_On_Temp=MR_COOL_ON_TEMP_DEFAULT,MC_Off_Temp=MR_COOL_OFF_TEMP_DEFAULT;

#define MrCoolMode	((MC_On_Temp>MC_Off_Temp)?1:0)
int vacation=FALSE,ngp, invMode, selling, buying, sellv=0, invbattv=0, KBLock=0, DropSelected=0, AmpsBelowThresholdWaitSecs=5;
int InverterPwr=0, SellVoltMin=DEFAULT_SELL_V_MIN, SellVoltMax=DEFAULT_SELL_V_MAX, UnderUtilization;
int loadBalance=0;//0=OK, 1=high (need to reduce), -1=low (need to increase)
time_t epoch_time,ResetTime;
struct tm *tm_p, ResetTime_p;
int dow=0,hour=0,minute=0,second=0;
enum AirCondPwrSrcModes AirCondPwrSrc=acpsGrid;
const char * acpsModeDesc[]={"Grid","Inv","Off"};
enum WHCMD{ cNone,cOff,cOn,cCheck,cInq, cSet, cUnset };
enum WHFlags{ fNone, fSetOverLoad, fSetManLck, fRelManLck, fManual, fAuto, fOverLoad, fManLck, fAllowTimer, fTimer};
sqlite3 *ppDb, *MMDb;
int airJordanManOff=FALSE;
float CwattHoursGen=0.0, CwattHoursPump=0.0;
FILE *mmlog, *persistantData;

#define TimeTest(HOUR,MIN,SEC)	(((HOUR==hour)||(HOUR<0)) && ((MIN==minute)||(MIN<0)) && ((SEC==second)||(SEC<0)))

struct fndcNetBattAmpAverager{
	float tot;
	float avg;
	int indx;
	float history[15];
}aNBA;

float updateANBA(void){
	aNBA.tot=aNBA.tot-aNBA.history[aNBA.indx]+netbattamps;
	aNBA.history[aNBA.indx++]=netbattamps;
	if(aNBA.indx>=15)aNBA.indx=0;
	return (aNBA.avg=(aNBA.tot/15));
}

struct fndcVoltsAverager{
	float tot;
	float avg;
	int indx;
	float history[5];
}aFNDCvolts;

float updateAFNDCvolts(void){
	float Volts=FNDC_BATT_VOLTS;
	aFNDCvolts.tot=aFNDCvolts.tot-aFNDCvolts.history[aFNDCvolts.indx]+Volts;
	aFNDCvolts.history[aFNDCvolts.indx++]=Volts;
	if(aFNDCvolts.indx>=5)aFNDCvolts.indx=0;
	return (aFNDCvolts.avg=(aFNDCvolts.tot/5));
}

struct angpAverager{
	int tot;
	int avg;
	int indx;
	int history[10];
}angp;

int updateANGP(void){
	angp.tot=angp.tot-angp.history[angp.indx]+ngp;
	angp.history[angp.indx++]=ngp;
	if(angp.indx>=10)angp.indx=0;
	return (angp.avg=(angp.tot/10));
}

/*int getANGP(void){
	return angp.tot/5;
}*/

struct {
	int	Indx;
	int	HOD;
	int	SOC_Targ;
	int	SellV;
	int	factor;
	int	NextIndx;
	int	NextHOD;
	int	NextSOC_Targ;
	int	NextSellV;
	int	Nextfactor;
	char Description[64];
}PrgDataStruct;

volatile struct pwm{int resolution; int count; int percent;}UE_PWM, LE_PWM;
//volatile float	WaterHeaterKWH=0.0;
volatile long	PWM_Debug_count;

void timer_handler(int signum)
{
	//static int thCount=0;
	//wprintw(ScrollWin,"time test %d\n",++thCount);
}

void SoundAlarm(int ms)
{
	int PinState=digitalRead(PIN_ALARM);
	digitalWrite(PIN_ALARM,1);
	usleep(ms*1000);
	digitalWrite(PIN_ALARM,PinState);
}

void logMesg(const char *fmt,...){
	va_list ap;
	
	va_start(ap,fmt);
	wprintw(ScrollWin,"%.2d:%.2d:%.2d:%.2d ",dow,hour,minute,second);
	vwprintw(ScrollWin,fmt,ap);
	mmlog=fopen("/var/log/mm.log","a");
	if(mmlog==NULL){
		perror("Unable to open /var/log/mm.log");
//		exit(1);
	}else{
		fprintf(mmlog,"%.1d:%.2d:%.2d:%.2d ",dow,hour,minute,second);
		vfprintf(mmlog,fmt,ap);
		fclose(mmlog);
	}
	va_end(ap);
//	fflush(mmlog);
}

void cmdMate(char * cmd, char * v, int ibookmark){
static int iLastBookmark=0;
	if ((write (usbmate,"<",1))<0)
	{
//		wprintw(ScrollWin,"Error writing to usb\n");
		logMesg("%.2d:%.2d:%.2d:%.2d Error writing to usb\n",dow,hour,minute,second);
	}
	write (usbmate,cmd,strlen(cmd));
	write (usbmate,":",1);
	write (usbmate,v,strlen(v));
	write (usbmate,">",1);
	WMVPRINTW(FNDCWin,9,2,"LastCMD <%7s:%-4s>%4d %4d",cmd,v,ibookmark,iLastBookmark);
	logMesg("LastCMD <%7s:%-4s>%4d %4d\n",cmd,v,ibookmark,iLastBookmark);
	//wprintw(ScrollWin,"LastCMD <%7s:%-4s>%4d %4d\n",cmd,v,ibookmark,iLastBookmark);
	//fprintf(mmlog,"%.2d:%.2d:%.2d:%.2d LastCMD <%7s:%-4s>%4d %4d\n",dow,hour,minute,second,cmd,v,ibookmark,iLastBookmark);
	iLastBookmark=ibookmark;
	MateCommResponseExpected=YES;
}

void AlterCurrentLimit(int NewLimit,int ibookmark,int direction){//direction is positive for incr only, neg for decr only, 0 either
char tString[10];
int lDiff=(NewLimit-AC1InLimit);
	if((lDiff>0) && (direction<0)) return;
	if((lDiff<0) && (direction>0)) return;
	sprintf(tString,"%d",MAX(NewLimit,5));
	cmdMate("AC1INLM",tString,ibookmark);
	AC1InLimit=NewLimit;
}

void shunt_C_monitor(void){
	static time_t Cstart=0;//,Cend=0;
//	static time_t periodStart=0;
//	static long pWM=0;
	float Camps=(FNDC_SHUNT_C_AMPS), CwattHours=0.0;
	
	if (((Camps > 10.0) || (Camps < (-1.0))) && (Cstart>0))
	{
		CwattHours = (Camps * FNDC_BATT_VOLTS * (float)((difftime(epoch_time,Cstart))/3600.0));
/*		if(periodStart==0){
			periodStart=epoch_time;
		}*/
		if(FNDC_SHUNT_C_NEG)
		{
			CwattHoursPump -= CwattHours;
		}
		else
		{
			CwattHoursGen += CwattHours;
		}
	}
	Cstart = epoch_time;
}

void init(){
sqlite3_stmt *stmt;
char sql[4096];
	initilized=YES;
	angp.tot=ngp*5;
	angp.avg=ngp;
	angp.indx=0;
	aFNDCvolts.tot=FNDC_BATT_VOLTS*5;
	aFNDCvolts.avg=FNDC_BATT_VOLTS;
	aFNDCvolts.indx=0;
	{int i; for(i=0;i<5;i++){angp.history[i]=ngp;aFNDCvolts.history[i]=FNDC_BATT_VOLTS;}}
	cmdMate("SELLV","512",__LINE__);
	sellv=512;
	cmdMate("AC1INLM","45",__LINE__);	
//	BatAmpHrCrx=(100.0-(float)(FNDC_SOC))*(BatRatedAmpHr/100.0);
	strcpy(sql,"select WHKWH,BuyKWH,InvertedKWH,DroppedMin,PowerOutageMin,UnderUtilizationMin"
		",BattAHin,BattAHout,BatAmpHrCrx,debug3 from stats WHERE TimeStamp=(SELECT MAX(TimeStamp) FROM stats)");
	logMesg("At init()%d\n",0);
	if (sqlite3_prepare_v2(MMDb, sql, strlen(sql)+1, &stmt, NULL)==SQLITE_OK){
		if (sqlite3_step (stmt)==SQLITE_ROW){
			IAO_Day_Secs=(long)((sqlite3_column_double(stmt,0)*3600.0)/1.08);
			BuyWH=(float)sqlite3_column_double(stmt,1)*1000.0;
			InvWH=(float)sqlite3_column_double(stmt,2)*1000.0;
			DroppedSecs=sqlite3_column_int(stmt,3)*60;
			PowerOutageSecs=sqlite3_column_int(stmt,4)*60;
			UnderUtilizationSecs=sqlite3_column_int(stmt,5)*60;
			BatAmpHrIn=(float)sqlite3_column_double(stmt,6);
			BatAmpHrOut=(float)sqlite3_column_double(stmt,7);
			BatAmpHrCrx=(float)sqlite3_column_double(stmt,8);
			BattEffFac=(float)sqlite3_column_double(stmt,9);
			sqlite3_finalize(stmt);
		}else{
			BatAmpHrIn=0.0-(100.0-(float)(FNDC_SOC))*(BatRatedAmpHr/100.0);
		}
		sqlite3_finalize(stmt);
	}else{
		BatAmpHrIn=0.0-(100.0-(float)(FNDC_SOC))*(BatRatedAmpHr/100.0);
	}
	if((hour>=7) && (hour<12))WHtopMinTemp=115;
	if((hour>=12) && (hour<=15)){
		WHtopMinTemp=120;
		WHCenterMinTemp=100.0;
	}
	if((hour>15) && (hour<=17)){
		WHCenterMinTemp=115;
		WHtopMinTemp=130;
	}
	persistantData=fopen("persistant.ini","r");
	if(persistantData==NULL){
		perror("Unable to open persistant.ini");
		exit(1);
	}
	fscanf(persistantData,"%f %f", &CwattHoursPump, &CwattHoursGen);
	fclose(persistantData);
}	

void WHpowerLevel(int OnHigh){
int	state=digitalRead(WH_LOWER_ELEMENT);
	if(state) {
		digitalWrite(WH_LOWER_ELEMENT,0);
		usleep(17000);
	}	
	if(OnHigh){
		//cmdMate("AUXON","1",__LINE__);
		digitalWrite(WH_LOWER_ELEMENT_HV,1);
	}else{
		//cmdMate("AUXOFF","1",__LINE__);
		digitalWrite(WH_LOWER_ELEMENT_HV,0);
	}					
	if(state) usleep(50000);
	digitalWrite(WH_LOWER_ELEMENT,state);
}

void GetPrgData(int Prog, int HOD){
sqlite3_stmt *stmt;
char sql[4096];
	if ((Prog!=PrgDataStruct.Indx)||(HOD!=PrgDataStruct.HOD)){
		sprintf(sql,"SELECT PrgData.*,Programs.ProgramDescription FROM PrgData INNER JOIN Programs ON PrgData.PrgIndx=Programs.PrgIndx WHERE PrgData.PrgIndx=%d and HrOfDay=%d"
														,Prog,HOD);
		if (sqlite3_prepare_v2(ppDb, sql, strlen(sql)+1, &stmt, NULL)==SQLITE_OK){
			if (sqlite3_step (stmt)==SQLITE_ROW){
				PrgDataStruct.Indx=sqlite3_column_int(stmt,0);
				PrgDataStruct.HOD=sqlite3_column_int(stmt,1);
				PrgDataStruct.SOC_Targ=sqlite3_column_int(stmt,2);
				PrgDataStruct.SellV=sqlite3_column_int(stmt,3);
				PrgDataStruct.factor=sqlite3_column_int(stmt,4);
				logMesg("Description %s\n",sqlite3_column_text(stmt,5));
				strncpy(PrgDataStruct.Description,(char *)sqlite3_column_text(stmt,5),63);
				sqlite3_finalize(stmt);
			}//else terminate=true;
			sqlite3_finalize(stmt);
		}
		/*sprintf(sql,"SELECT * FROM PrgData WHERE PrgIndx=%d and HrOfDay=%d"
														,Prog,((HOD<=22) ? (HOD+1) : 0 ));*/
		sprintf(sql,"SELECT PrgData.*,Programs.ProgramDescription FROM PrgData INNER JOIN Programs ON PrgData.PrgIndx=Programs.PrgIndx WHERE PrgData.PrgIndx=%d and HrOfDay=%d"
														,Prog,((HOD<=22) ? (HOD+1) : 0 ));
		if (sqlite3_prepare_v2(ppDb, sql, strlen(sql)+1, &stmt, NULL)==SQLITE_OK){
			if (sqlite3_step (stmt)==SQLITE_ROW){
				PrgDataStruct.NextIndx=sqlite3_column_int(stmt,0);
				PrgDataStruct.NextHOD=sqlite3_column_int(stmt,1);
				PrgDataStruct.NextSOC_Targ=sqlite3_column_int(stmt,2);
				PrgDataStruct.NextSellV=sqlite3_column_int(stmt,3);
				PrgDataStruct.Nextfactor=sqlite3_column_int(stmt,4);
				strncpy(PrgDataStruct.Description,(char *)sqlite3_column_text(stmt,5),63);
				sqlite3_finalize(stmt);
			}//else terminate=true;
			sqlite3_finalize(stmt);
		}
	}	
}

void SaveStats(float Volts, int SOC){

sqlite3_stmt *stmt;
char sql[4096];
	if(!EnableSystem)return;
	sprintf(sql,"INSERT INTO stats values(%ld,%f,%d,%f,%f,%f,%f,%f,%d,%d,%d,%f,%f,%d,%d,%f,%f)"
				,epoch_time, Volts/*FNDC_BATT_VOLTS*/, SOC/*FNDC_SOC*/,WaterHeaterKWH,BuyWH/1000.0,InvWH/1000.0
				,CC1_KWHH,CC2_KWHH,(DroppedSecs/60),(PowerOutageSecs/60),(UnderUtilizationSecs/60)
				,BatAmpHrIn,BatAmpHrOut,DischargeAllowed,hour,/*MaxVNegAmps*/BattEffFac,BatAmpHrCrx);
	if (sqlite3_prepare_v2(MMDb, sql, strlen(sql)+1, &stmt, NULL)==SQLITE_OK){
		if (sqlite3_step (stmt)==SQLITE_DONE);{
			//success
		}//else terminate=true;
		sqlite3_finalize(stmt);
	}//else terminate=true;	
}
	
int sendmail(const char *to, const char *from, const char *subject, const char *message){
    int retval = -1;
    FILE *mailpipe = popen("/usr/lib/sendmail -t", "w");
    if (mailpipe != NULL) {
        fprintf(mailpipe, "To: %s\n", to);
        fprintf(mailpipe, "From: %s\n", from);
        fprintf(mailpipe, "Subject: %s\n\n", subject);
        fwrite(message, 1, strlen(message), mailpipe);
        fwrite(".\n", 1, 2, mailpipe);
        pclose(mailpipe);
        retval = 0;
     }
     else {
         perror("Failed to invoke sendmail");
     }
     return retval;
}

void ProcessSMSmesg(struct SMS_Message_Properties *mesg){
	logMesg("Recieved -%s- from %s %d %d\n",mesg->MsgText,mesg->PhoneNum,hour,minute);
	if(strcmp("2814509680",mesg->PhoneNum)!=0) return;
	switch (mesg->MsgText[0]) {
		case 'P'	:
		case 'p'	:
			ProgramIndx=atoi(mesg->MsgText+1);
			break;
		case 'Z': // Z1=Discharge Not Allowed
		case 'z': // Z0=Discharge Is Allowed
			if (mesg->MsgText[1]=='0')
			{
				DischargeAllowed=YES;
				MaxVNegAmps=0.0;
			}
			else
			{
				DischargeAllowed=NO;MaxVNegAmps=0.0;
				cmdMate("AC","1",__LINE__);
			}
			break;			
		case 'C':
		case 'c':
			logMesg("found c @ %d %s\n",__LINE__,mesg);
			if (mesg->MsgText[1]=='0')
			{
				logMesg("%d found c @ %d %s\n",InvChrgEnabled,__LINE__,mesg);
				if (InvChrgEnabled==YES){
//					cmdMate("BULK","2");
					cmdMate("CHG","0",__LINE__);
					InvChrgEnabled=NO;
				}
			}
			else
			{
				if (InvChrgEnabled!=YES){
					//whsSetFlags(whsAuto,whsOff);
					cmdMate("AC1INLM","50",__LINE__);
					cmdMate("CHG","2",__LINE__);
					cmdMate("BULK","1",__LINE__);
					InvChrgEnabled=YES;
				}
			}
			break;
/*		case 'K':
		case 'k':
			if (mesg->MsgText[1]=='0')
				whsSetFlags(whsDisableTimer,whsUnset);
			else
				whsSetFlags(whsDisableTimer,whsOn);
			break;		*/		
		case 'N':
		case 'n':
			if (mesg->MsgText[1]=='0')
				SMS_doSendStatRpt=NO;
			else
			{
				SMS_doSendStatRpt=YES;
				SEND_SMS_STAT_RPT;
			}
			break;
/*		case 'T':
		case 't':
			if (mesg->MsgText[1]=='0')
				whsSetFlags(whsTimer,whsUnset);
			else
				whsSetFlags(whsTimer,whsOn);
			break;*/
	}
}

float WH_KWH_LastSec(void){
	float amps,power;
	static time_t lastTimeChecked=0L;
	if (lastTimeChecked==0) lastTimeChecked=epoch_time;
	if(digitalRead(WH_LOWER_ELEMENT)){
		amps = L2_INVERTER_VOUT / WH_ELEMENT_OHMS;	
		power = amps * L2_INVERTER_VOUT;
	}
	if(digitalRead(WH_UPPER_ELEMENT)){
		amps = L1_INVERTER_VOUT / WH_ELEMENT_OHMS;
		power += amps * L1_INVERTER_VOUT;
	}
	power=(power/(1000.0/*W to KW*/ * 60.0 /*min*/ * 60.0 /*sec*/))*difftime(lastTimeChecked,epoch_time);
	lastTimeChecked=epoch_time;
	return power;  //this number is KWH for the last second or ever-how long since last called.
}

void TimeEvents(void){
static float AvHrVolts=0.0;
static long c=0L, AvHrSOC=0L;

	++c; AvHrVolts+=FNDC_BATT_VOLTS; AvHrSOC+=(long)FNDC_SOC;
	if (TimeTest(-1,32,0)){
		logMesg("PWM debug count=%d\n", PWM_Debug_count);
		PWM_Debug_count=0;
	}
	if ((TimeTest(-1,1,1))&&(PrgDataStruct.SOC_Targ<100)&&(InvChrgEnabled==YES))
	{				
		cmdMate("CHG","0",__LINE__);
		InvChrgEnabled=NO;
		logMesg("Turning charger off at %d:%d\n",hour,minute);
	}
	if((tm_p->tm_mday==32) && (hour>=10)) vacation=FALSE;
	if(vacation==FALSE)
	{
		switch (hour)
		{
		case 5	:
			if (TimeTest(5,30,0))	AmpsBelowThresholdWaitSecs=25;
			if (TimeTest(5,45,0))	if(WHtopMinTemp<115.0) WHtopMinTemp=115.0;
		case 7	:
			if (TimeTest(7,0,0))
			{
				if (InvInputMode==GridTied)
				{	
					MaxNegBatAmpsDropped=(-40);
					SellVoltMin=504;
				}
				SEND_SMS_STAT_RPT
			}
			if (TimeTest(7,15,0)){
				if(WHtopMinTemp<115.0) WHtopMinTemp=115.0;
			}
			break;
		case 8	:
/*	if((dow==1) || (dow==3) || (dow==5))*/
			if(TimeTest(8,1,0))
			{/*setACPS(acpsInverter);*/
				BathDone=FALSE;
				DropSelected=1;				
			}
			break;
		case 10	:
			if (TimeTest(10,0,0))
			{
				AmpsBelowThresholdWaitSecs=5;
				SEND_SMS_STAT_RPT
			}
		case 11	:
			if (TimeTest(11,50,0)&&(BathDone==FALSE)&&(WHtopMinTemp<120.0))WHtopMinTemp=120.0;
			break;
		case 12	:
			break;
		case 13	:
			if (TimeTest(13,0,0) && (FNDC_SOC<93) && (InvInputMode==GridTied))
			{
				if (sellv<516)
				{
					cmdMate("SELLV","516",__LINE__);
					sellv=516;
				}
				if (SellVoltMin<516){ SellVoltMin=516; }
				if (MaxNegBatAmpsDropped<(-30)){MaxNegBatAmpsDropped=(-30);}
			}
			if (TimeTest(13,30,0)&&(BathDone==FALSE)&&(WHtopMinTemp<125.0))WHtopMinTemp=125.0;
			break;
		case 14	:
			if (TimeTest(14,0,0) && (FNDC_SOC<98)  && (InvInputMode==GridTied))
			{
				if (sellv<520)
				{
					cmdMate("SELLV","520",__LINE__);
					sellv=520;
				}
				if (SellVoltMin<520){ SellVoltMin=520; }
				if (MaxNegBatAmpsDropped<(-25)){MaxNegBatAmpsDropped=(-25);}
			}	
			if (TimeTest(14,15,0))
			{
				SEND_SMS_STAT_RPT
				if(BathDone==FALSE)
				{
					if(WHtopMinTemp<140.0) WHtopMinTemp=160.0;
					if(WHCenterMinTemp<125.0) WHCenterMinTemp=140.0;		
				}
			}
			break;
		case 15	:
			if (TimeTest(15,0,0) && (FNDC_SOC<99) && (InvInputMode==GridTied))
			{
				if (sellv<540)
				{
					cmdMate("SELLV","540",__LINE__);
					sellv=540;
				}
				if (SellVoltMin<540){ SellVoltMin=540; }
				if (MaxNegBatAmpsDropped<(-20)){MaxNegBatAmpsDropped=(-20);}
			}
			if (TimeTest(15,15,0))
			{
				SEND_SMS_STAT_RPT
			}
			if ((TimeTest(15,30,0)) && (InvInputMode==GridTied))
				{	
					MaxNegBatAmpsDropped=(-4);
					DropSelected=0;
					if (SellVoltMin<540) SellVoltMin=540;
					if (sellv<540)
					{
						cmdMate("SELLV","540",__LINE__);
						sellv=540;
					}
				}		
		case 16	:	
			if ((TimeTest(16,0,0)) && (InvInputMode==GridTied))
				{	
					MaxNegBatAmpsDropped=(-4);
					DropSelected=0;
					if (SellVoltMin<560) SellVoltMin=560;
					if (sellv<560)
					{
						cmdMate("SELLV","560",__LINE__);
						sellv=560;
					}
				}		
			if (TimeTest(16,45,0))
			{
				if(BathDone==FALSE)
				{
					wh_le_src=GRID;
					WhLEGridGoal=125.0;
					logMesg("Lower WH element on grid\n");
				}
			}
			break;
//6PM
		case 17	:
/*			if (TimeTest(17,0,0) && (FNDC_SOC<97) && (InvInputMode==GridTied)){
				if (sellv<524)
				{
					cmdMate("SELLV","524",__LINE__);
					sellv=524;
				}
				if (SellVoltMin<524){ SellVoltMin=524; }
				if (MaxNegBatAmpsDropped<(-10)){MaxNegBatAmpsDropped=(-10);}
			}*/
			if (TimeTest(17,15,0))
			{
				wh_le_src=INVERTER;
				WhLEGridGoal=50.0;
				logMesg("Lower WH element on inverter\n");
			}
			if (TimeTest(17,0,0) && (InvInputMode==GridTied))
			{
				if (sellv>528)
				{
					cmdMate("SELLV","528",__LINE__);
					sellv=528;
				}
				if (SellVoltMin>528){ SellVoltMin=528; }
				MaxNegBatAmpsDropped=(0);
			}
//6:15PM
			if (TimeTest(17,15,0))
			{
				if(WHCenterMinTemp>110.0) WHCenterMinTemp=110.0;
				if(WHtopMinTemp>120.0) WHtopMinTemp=120.0;
			}	
			break;
		case 19	:
			if((BathDone==FALSE)&&(TimeTest(-1,-1,30)))
			{
				float WHTTD=(readSensorF(2,666.6)-WHtopMinTemp);
				if (WHTTD>0) WHCenterMinTemp=110.0-WHTTD;
			}
			break;
		case 20	:
			if (TimeTest(20,0,5))	if(WHCenterMinTemp>90.0) WHCenterMinTemp=90.0;
			if (TimeTest(20,0,33))
			{
				if (((FNDC_SOC>95)&&(DaysSinceFull>1.5))||(DaysSinceFull>3))
				{
					cmdMate("AC1INLM","45",__LINE__);
					cmdMate("CHG","2",__LINE__);
					cmdMate("BULK","1",__LINE__);
					InvChrgEnabled=YES;
					cmdMate("SELLV","580",__LINE__);
					sellv=580;
				}
				else
				{ 
					if (sellv>516)
					{
						cmdMate("SELLV","516",__LINE__);
						sellv=516;
					}
					if (SellVoltMin>516){ SellVoltMin=516; }				
				}
				MaxNegBatAmpsDropped=(-3);
			}
			break;
		case 22	:
			if (TimeTest(22,0,0))
			{
				WHCenterMinTemp=50.0;
				if(WHtopMinTemp>105.0) WHtopMinTemp=105.0;
				cmdMate("CHG","0",__LINE__);
				InvChrgEnabled=NO;
				if (sellv>516)
				{
					cmdMate("SELLV","516",__LINE__);
					sellv=516;
				}
				if (SellVoltMin>516){ SellVoltMin=516; }
			}
			break;
		case 23	:
			if (TimeTest(23,0,0))
			{
				preferHeatPumpOn=FALSE;
			}
			break;	
	}
//Email Report broken on this pc
	if (TimeTest(28,58,30)){
		sprintf(strtmp,"Hr:%d V:%3.1f | SOC:%d | WH:%5.3f | Buy:%2.0f | IKWH:%2.0f | KWH %4.1f %4.1f %4.1f | Drp Min %4d | Outage Min %4d | Underutilization Min %4d | In %5.2f Out %5.2f | SOC S %3d E %3d"
				,hour, FNDC_BATT_VOLTS, FNDC_SOC,WaterHeaterKWH,BuyWH/1000.0,InvWH/1000.0
				,CC1_KWHH,CC2_KWHH,CC1_KWHH+CC2_KWHH, (DroppedSecs/60),(PowerOutageSecs/60),(UnderUtilizationSecs/60)
				,BatAmpHrIn,BatAmpHrOut, SOCstart, FNDC_SOC);
//		sendmail("2814509680@vtext.com","garlee55@gmail.com","Report",strtmp);
		sendmail("garlee55@gmail.com","garlee55@gmail.com","Report",strtmp);
//		BatAmpHrOut=0.0;
	}	
  }
//Every hour on the hour
	if (TimeTest(-1,0,0)){ 
		SaveStats((AvHrVolts/c),(int)(AvHrSOC/c));//whsSetFlags(whsTimer, whsOff);
		c=0; AvHrSOC=0; AvHrVolts=0.0;
	}
//11:59:59PM
	if (TimeTest(0,0,0)){
		SOCstart=FNDC_SOC;
		PowerOutageSecs=0;
		BuyWH=0;SellWH=0;InvWH=0;
		IAO_Day_Secs=0;	IAR_Day_Secs=0;
		DroppedSecs=0;
		UnderUtilizationSecs=0;
		if(ProgramIndx!=6){DischargeAllowed=YES;MaxVNegAmps=0.0;}
//		whsSetFlags(whsTimer, whsOff);
	}
// on the half minute
	if (TimeTest(-1,-1,30)){
//		char msgBuf[0x100];
		struct SMS_Message_Properties msgBuf;
		if (SMS_Rec_Mesg(&msgBuf)!=NULL)
			ProcessSMSmesg(&msgBuf);
	}
}
void POAlarm(int signal)
{
static bool falureDetected=false;
	switch (signal)		
	{
		case 1:	//falt detected
			if(falureDetected==false)
			{
				falureDetected=true;
				digitalWrite(PIN_ALARM,1);
			}
			break;
		case 2:	//acknowleged
			digitalWrite(PIN_ALARM,0);
			break;
		case 3:	//normal
			if(falureDetected==true)
			{
				digitalWrite(PIN_ALARM,0);
				falureDetected=false;
			}
			break;
	}
}
void PowerOutage(void)
{	
	POAlarm(1);
	if(INVERTER_OP_MODE!=INV_IS_OFF)		// inverter is on
	{ 
		if((FNDC_BATT_VOLTS<44.0) || (FNDC_SOC<MIN(MAX((PrgDataStruct.SOC_Targ),(50)),(80)) && (InvIgnLowVolt==false)))
		{
			cmdMate("INV","0",__LINE__);
		}
	}
	else		//	inverter is off
	{
		if((FNDC_BATT_VOLTS>50.0 && FNDC_SOC>85) || InvIgnLowVolt==true)
		{
			cmdMate("INV","2",__LINE__);
		}
			
	}
	{
		ALARM_TIMMER(td,900ul,5,10)
		{			
			sprintf(strtmp,"Power is off! Volts= %3.1f SOC= %3d Net Batt Amps= %6.1f"
					, FNDC_BATT_VOLTS, FNDC_SOC,ANBA);
			logMesg("SMS_SendMsg returned %d. Msg: %s\n",SMS_SendMsg("2814509680",strtmp),strtmp);
			SoundAlarm(200);
			//usleep(100000);
			logMesg("SMS_SendMsg returned %d. Msg: %s\n",SMS_SendMsg("4096730837",strtmp),strtmp);
			if (FNDC_BATT_VOLTS<52.0) setACPS(acpsGrid);//4-27-2016
			else if (FNDC_BATT_VOLTS>56.0) setACPS(acpsInverter);
		}
/*			wprintw(ScrollWin,"h%dm%d@%d %lu\t%lu\t%lu\t%d\t%d\t%d\t%lu\n",hour,minute,__LINE__
				,td.ReAlarmWaitTimer,td.ReAlarmWaitDelay,td.TriggerCountWithinTimerStart
				,td.reqTriggersWithinTicks,td.TriggerCount,td.reqNumTriggers,td.CurrentTick);*/
	}
/*	{static unsigned long reSendTimer=0ul;
		if (reSendTimer < Ticks)
		{
			sprintf(strtmp,"Power is off! Volts= %3.1f SOC= %3d Net Batt Amps= %6.1f"
					, FNDC_BATT_VOLTS, FNDC_SOC,netbattamps);
			wprintw(ScrollWin,"SMS_SendMsg returned %d. Msg: %s\n",SMS_SendMsg("2814509680",strtmp),strtmp);
			reSendTimer=Ticks+900ul;
		}
	}*/

/*	if (FNDC_BATT_VOLTS<54.0){
		whsSetFlags(whsConserve,whsOn);
//		digitalWrite(27,0);
	}else{
		if (FNDC_BATT_VOLTS>56.0) whsSetFlags(whsConserve,whsOff);
//		if (preferHeatPumpOn) digitalWrite(27,1);
	}*/
}

float SOC_VarToTrg(void){
	return((((float)PrgDataStruct.SOC_Targ*(60.0-(float)minute)
				+(float)PrgDataStruct.NextSOC_Targ*(float)minute)/60.0) 
				- ((MIN(BATT_STATUS_AH,0)+BatRatedAmpHr)/BatRatedAmpHr)*100.0);
																		//(float)FNDC_SOC);
}
#define ANTIHYSTERESIS 3
#define DROP_GRID(PAUSE,FIXED_AMPS)	{cmdMate("AC","0",__LINE__); pause=PAUSE; \
								MaxNegBatAmpsDropped=MIN(0,(BatTrgAmp-FIXED_AMPS)); \
								cmdMate("AC1INLM","45",__LINE__); AC1InLimit=45;}
#define ADJ_SELLV(NEWSELLV)	{		sprintf(strtmp,"%d",NEWSELLV);\
									cmdMate("SELLV",strtmp,__LINE__);\
									sellv=NEWSELLV;}
/*int NSV=NEWSELLV;\
									if ((NSV<MaxVNegAmps)&&(DischargeAllowed==NO))NSV=MaxVNegAmps;\*/
void ControlSupportMode(void){//need to fix offset mode -- drop grid if input exceedes output by the target
	static int pause=0;
	enum CC_Modes CC_Mode1=CC1_MODE,CC_Mode2=CC2_MODE;
	float SOCVTT=SOC_VarToTrg();
	int BuyCurMax=MAX(L1_BUY_AMPS,L2_BUY_AMPS);
	int SelCurMax=MAX(L1_SELL_AMPS,L2_SELL_AMPS);
	static int Selling=0;
	int AdjustedSellV=(int)((SOCVTT * (float)PrgDataStruct.factor) + (float)PrgDataStruct.SellV);
	static int pau=0;//pause for adjusting sell voltage
	if (INVERTER_OP_MODE==IOM_CHRG){
		if (sellv!=620){
			cmdMate("SELLV","620",__LINE__);
			sellv=620;
		}
		if (AC1InLimit != 48){
			AC1InLimit = 48;
			cmdMate("AC1INLM","48",__LINE__);
		}
//		whsSetFlags(whsAuto,whsOff);
		pause=60;
		return;
	}
	else if ((INVERTER_OP_MODE==IOM_OFFSET) && ((netbattamps-BatTrgAmp)>30)){//new 02/11/2015
		ALARM_TIMMER(td,60ul,60,90){
			DROP_GRID(5,((DischargeAllowed==YES)?((0-BatTrgAmp)+20):5));
		}
/*		wprintw(ScrollWin,"Offset trig h%dm%d@%d %lu\t%lu\t%lu\t%d\t%d\t%d\t%lu\n",hour,minute,__LINE__
				,td.ReAlarmWaitTimer,td.ReAlarmWaitDelay,td.TriggerCountWithinTimerStart
				,td.reqTriggersWithinTicks,td.TriggerCount,td.reqNumTriggers,td.CurrentTick);
*/
	}
	else if ((sellv<(FNDC_BATT_VOLTS*10.1)) && ((L1_CHARGER_AMPS+L2_CHARGER_AMPS)>2)){
		ALARM_TIMMER(td,60ul,30,60){
			DROP_GRID(5,((DischargeAllowed==YES)?((0-BatTrgAmp)+20):5));
			//wprintw(ScrollWin,"Drop Grid suppressed command at line %d -- Triggered inverter charger event\n",__LINE__);
		}
/*		wprintw(ScrollWin,"Charger trig h%dm%d@%d %lu\t%lu\t%lu\t%d\t%d\t%d\t%lu\n",hour,minute,__LINE__
				,td.ReAlarmWaitTimer,td.ReAlarmWaitDelay,td.TriggerCountWithinTimerStart
				,td.reqTriggersWithinTicks,td.TriggerCount,td.reqNumTriggers,td.CurrentTick);		*/
	}
	if (pau>0)pau--;
	if (AdjustedSellV<SUPPORT_MODE_MIN_ADJ_SELL_VOLTS) AdjustedSellV=SUPPORT_MODE_MIN_ADJ_SELL_VOLTS;
	if (AdjustedSellV>SUPPORT_MODE_MAX_ADJ_SELL_VOLTS) AdjustedSellV=MAX(SUPPORT_MODE_MAX_ADJ_SELL_VOLTS,((float)PrgDataStruct.SellV));
	if ((AdjustedSellV<(int)(MaxVNegAmps*10.0))&&(DischargeAllowed!=YES)) AdjustedSellV=(int)(MaxVNegAmps*10.0);
	if (InvInputMode != GridTied) WMVPRINTW(FNDCWin,6,2,"%s                       ","");
	if (ANGP<0){Selling++;}else{Selling=0;}
	if (FNDC_BATT_VOLTS<45){
		cmdMate("AC1INLM","48",__LINE__);
//		whsSetFlags(whsConserve,whsOn);
		AC1InLimit=50;
		pause=60;
		return;
	}/*else{
		if (pause<1) whsSetFlags(whsConserve,whsOff);
	}*/
	if (Selling>5){Selling=0; DROP_GRID(5,((PrgDataStruct.SOC_Targ>99)?125:((DischargeAllowed==YES)?20:5)))}
	if (INVERTER_OP_MODE==IOM_OFFSET){
		if (AC1InLimit < (BuyCurMax + 2)&&(pause<1)){
			AC1InLimit = (BuyCurMax + 2);
			sprintf(strtmp,"%d",AC1InLimit);
			cmdMate("AC1INLM",strtmp,__LINE__);//raising limit
			WMVPRINTW(FNDCWin,5,2,"AC1InLimit %4d   ",AC1InLimit);	
			pause=20;//10/27/2015
			return;	//10/27/2015		
		} 
		pause=5; 
	}	
	if (pause>0){
		pause--;
		return;
	}
	BatTrgAmp=(SOCVTT*(BatRatedAmpHr/100)); 
	if ((BatTrgAmp<0) && (DischargeAllowed==NO)) BatTrgAmp=0;
	if ((PrgDataStruct.SOC_Targ>99) && (CC_Mode1==Silent) && (CC_Mode2==Silent)){
		if (sellv<520){
			cmdMate("SELLV","520",__LINE__);
			sellv=520;
		}
		if (AC1InLimit<45){
			cmdMate("AC1INLM","45",__LINE__);
			AC1InLimit=45;
		}
//		whsSetFlags(whsAuto,whsOff);
		return;
	}
	AdjustedSellV=(((int)((AdjustedSellV+(4/2))/4.0))*4);//round to nearest voltage resoultion for radian - 0.4v
	if (AdjustedSellV!=sellv){
		if (pau<1){
			if((DischargeAllowed!=NO)||(AdjustedSellV>sellv))//10/27/2015
			{
				ADJ_SELLV(AdjustedSellV);
				pau=60;//don't change but once a minute
			}
		}
	}
	if (PrgDataStruct.SOC_Targ>99){
		if (UnderUtilization){
		int NewLimit = BuyCurMax-3;
			sprintf(strtmp,"%d",MAX(NewLimit,5));
			cmdMate("AC1INLM",strtmp,__LINE__);
			AC1InLimit=NewLimit;
			pause=10;
		}	
		if (((INVERTER_OP_MODE!=IOM_OFFSET) && (FNDC_BATT_VOLTS*10)<(sellv-ANTIHYSTERESIS)) && (AC1InLimit<45) 
					&& (BuyCurMax>=AC1InLimit) && (TOTAL_INV_AC_CHRG_AMPS==0)){
//			AC1InLimit=(BuyCurMax+(sellv-(FNDC_BATT_VOLTS*10)));
			AlterCurrentLimit((BuyCurMax+((sellv-(FNDC_BATT_VOLTS*10))/2)+2),__LINE__,1);
/*			sprintf(strtmp,"%d",AC1InLimit);
			cmdMate("AC1INLM",strtmp,__LINE__);*/
			WMVPRINTW(FNDCWin,5,2,"AC1InLimit %4d   ",AC1InLimit);
//			whsSetFlags(whsAuto,whsOff);	//WATERHEATEROFF;
			pause=5;
		}else if (((INVERTER_OP_MODE!=IOM_OFFSET) && (FNDC_BATT_VOLTS*10)>(sellv+ANTIHYSTERESIS)) && (AC1InLimit>5)
				&& ((CC_Mode1!=Silent) || (CC_Mode2!=Silent) || (DischargeAllowed!=NO))){
			if (AC1InLimit>BuyCurMax){
				AC1InLimit=BuyCurMax;
				sprintf(strtmp,"%d",AC1InLimit);
				cmdMate("AC1INLM",strtmp,__LINE__);
			}else{
				cmdMate("AC1INLM","-",__LINE__);
				AC1InLimit--;
			}
			WMVPRINTW(FNDCWin,5,2,"AC1InLimit %4d   ",AC1InLimit);
			pause=5;		
		}
		return;
	}
	if ((SelCurMax>BuyCurMax) && (SelCurMax>AC1InLimit)) {
		sprintf(strtmp,"%d",SelCurMax);
		cmdMate("AC1INLM",strtmp,__LINE__);
		AC1InLimit=SelCurMax;
	}
	else
	{
		if (((INVERTER_OP_MODE!=IOM_OFFSET) && (FNDC_BATT_VOLTS*10)>sellv) && (AC1InLimit>5)
				&& ((CC_Mode1!=Silent) || (CC_Mode2!=Silent) || (DischargeAllowed!=NO))){
			AC1InLimit=(BuyCurMax-1);
			if (AC1InLimit<0){
				DROP_GRID(10,((DischargeAllowed==YES)?20:5));
				return;
			}
			sprintf(strtmp,"%d",MAX(AC1InLimit,5));
			cmdMate("AC1INLM",strtmp,__LINE__);
			WMVPRINTW(FNDCWin,5,2,"AC1InLimit %4d   ",AC1InLimit);
			pause=5;		
			return;
		}//below works but trying it above.  may get locked in offset mode and not update targets, etc.
/*		if (INVERTER_OP_MODE==IOM_OFFSET){
			if (AC1InLimit < (BuyCurMax + 3)){
				AC1InLimit = (BuyCurMax + 3);
				sprintf(strtmp,"%d",AC1InLimit);
				cmdMate("AC1INLM",strtmp,__LINE__);
				WMVPRINTW(FNDCWin,5,2,"AC1InLimit %4d   ",AC1InLimit);				
			} 
			pause=5; 
			return;
		}*/
		if ((((FNDC_BATT_VOLTS*10)==sellv) || (INVERTERVOLTS==sellv)) && (AC1InLimit>=5)){
			pause=30;
			return;
		}
	int InvV=L1_INVERTER_VOUT+L2_INVERTER_VOUT;	
	int LoadDifWats=(BatTrgAmp-ANBA)*INVERTERVOLTS;
		if (abs(LoadDifWats)>300) 
		{
		int ACLoadDifAmps=LoadDifWats/InvV;
		int NewLimit=BuyCurMax+ACLoadDifAmps;
			WMVPRINTW(FNDCWin,5,2,"NewLimit %4d    ",NewLimit);
			if (NewLimit<5) { 
				if ((NewLimit<(-5))&&(FNDC_BATT_VOLTS>51.2)){
//					whsSetFlags(whsAuto,whsOn);
					pause=3;
				}
				else if (NewLimit<0){DROP_GRID(5,((DischargeAllowed==YES)?20:5))}
				NewLimit=5; 
			}
			if (NewLimit>50) { NewLimit=50; }
			if (AC1InLimit!=NewLimit) {
				if (NewLimit>7){
//					whsSetFlags(whsAuto,whsOff);	
					pause=3;
				}
				if ((INVERTERVOLTS < sellv) || (ANBA > BatTrgAmp) || (NewLimit > AC1InLimit)){
					if ((NewLimit>=AC1InLimit) || (BuyCurMax>=NewLimit)){
						sprintf(strtmp,"%d",MAX(NewLimit,5));
						cmdMate("AC1INLM",strtmp,__LINE__);
						AC1InLimit=NewLimit;
						pause=2;
					}
				}
			}
		}
	}
}

void ControlGridTieUse(void){
int SOD=(100-(FNDC_SOC));
int TargetGPUse;
//int	sellAMPs=L1_SELL_AMPS+L2_SELL_AMPS-L1_BUY_AMPS-L1_BUY_AMPS;
static int GTpause=0;
static int weight=0, trigger=20000;
	if (AC1InLimit!=45){ cmdMate("AC1INLM","45",__LINE__); AC1InLimit=45; cmdMate("SELL","1",__LINE__);}
	WMVPRINTW(FNDCWin,5,2,"%s                    ","");
	TargetGPUse=500+(SOD*SOD*SOD);
	if (GTpause-- >0)return; else if(GTpause<0) GTpause=0;
	if(ANGP>500 && loadBalance!= 1 && GT_NoDrop){
		loadBalance=1;
		logMesg("Setting loadBalance=1 ANGP==%d GT_NoDrop==%d MM@%d\n",ANGP,GT_NoDrop,__LINE__);
		GTpause=3;
		return;
	}else if(ANGP< (-100) && loadBalance!= -1 && GT_NoDrop){
		loadBalance= -1;
		logMesg("Setting loadBalance= -1 ANGP==%d GT_NoDrop==%d MM@%d\n",ANGP,GT_NoDrop,__LINE__);
		GTpause=3;
		return;
	}
		
	weight+=((ANGP>0) ? (ANGP-TargetGPUse) : ((ANGP*2)-TargetGPUse)*2);
	if (abs(weight)>=trigger) // Weight triggered an event
	{
		if (weight>0) // Load is too high. Reduce load -- water heater or battery charging volts.
		{
			if ((sellv>SellVoltMin) && (sellv >= (invbattv))){	//reduce battery charging volts
				sellv-=4;
				if (sellv==SellVoltMin){
					sprintf(strtmp,"%d",SellVoltMin);
					cmdMate("SELLV",strtmp,__LINE__);
				}
				else
				{
					sprintf(strtmp,"%d",sellv);
					cmdMate("SELLV",strtmp,__LINE__);
//					cmdMate("SELLV","-",__LINE__);
				}
			}
			else 	// reduce water heater power
			{
				if (!UnderUtilization && GT_NoDrop && loadBalance!=1){
					loadBalance=1;
					logMesg("Setting loadBalance= 1 ANGP==%d GT_NoDrop==%d sellv==%d, SellVoltMin==%d invbattv==%d MM@%d\n",
													ANGP,GT_NoDrop,sellv, SellVoltMin, invbattv,__LINE__);
//					whsSetFlags(whsAuto,whsOff);
				}
			}
		}
		else //weight < 0 - Load is too low.  Increase load. 
		{
			if ((TargetGPUse<1000) && (loadBalance != -1) && GT_NoDrop){
				loadBalance= (-1);
				logMesg("Setting loadBalance= -1 ANGP==%d GT_NoDrop==%d TargetGPUse==%d MM@%d\n",
													ANGP,GT_NoDrop,TargetGPUse,__LINE__);
			}
			else 	// raise battery charging volts
			{
				if ((sellv<SellVoltMax) && (sellv <= (invbattv+0))){
					sellv+=4;
					if (sellv==SellVoltMax){
						sprintf(strtmp,"%d",SellVoltMax);
						cmdMate("SELLV",strtmp,__LINE__);
					}
					else
					{
						cmdMate("SELLV","+",__LINE__);
					}
				}
			}
		}
		weight=0;
	}
	else
	{
		if (((weight<0) && (LE_IS_ON_FULL) && sellv>= SellVoltMax ) ||
			((weight>0) && ((LE_IS_OFF) && sellv<= SellVoltMin ) ))
		{
			weight=0;
		}	
	}
	WMVPRINTW(FNDCWin,5,2,"T %5d W %7d",TargetGPUse,weight);
	if ((DropSelected) && ((FNDC_SOC)>(MIN_SOC_DROPPED))){
	static int tmr=0;
		if (((MaxNegBatAmpsDropped)*INVEFF) < (ANBA-((ANGP*10)/AFNDCV))){
			if (tmr++ > 30){
				cmdMate("AC","0",__LINE__);
				ADJ_SELLV(SellVoltMin);
				tmr=0;
			}
		}
		else
		{
			if (--tmr<0) { tmr=0; }
		}
	}
}

#define SWITCH_TO_GRID(PAUSE)	if ((!UnderUtilization && ((FNDC_BATT_VOLTS<54.0)&&((vacation==FALSE)||(hour>=15)||(FNDC_SOC<90))))|| \
									(FNDC_BATT_VOLTS<46.0)||(FNDC_SHUNT_A_AMPS<(-200))) \
										{cmdMate("AC","1",__LINE__); /*whsSetFlags(whsAuto,whsOff);*/ \
										cmdMate("AC1INLM","45",__LINE__);SavedSOC=0; pause=PAUSE;BelowSellvSecs=0;}
#define BELOWSELLVSECSTRIG	(-1800) //tenths of a volt * number of secs or decavolt secs
#define BELOWSELLVSECSMAX	1200
#define VDIFF	((FNDC_BATT_VOLTS*10)-(sellv))

void DroppedGrid(void){
static int pause=0, SavedSOC=0,	BelowSellvSecs=0;
int SOC=(int)((((float)BATT_STATUS_AH+ (float)BatRatedAmpHr)/(float)BatRatedAmpHr)*100.0); //FNDC_SOC;
	if(FNDC_BATT_VOLTS<MIN_VOLTS_DROPPED)SWITCH_TO_GRID(0);	
//	if(FNDC_BATT_VOLTS>57) digitalWrite(27,1);
//	if((FNDC_BATT_VOLTS>52) && (preferHeatPumpOn)) digitalWrite(27,1);
//	if(FNDC_BATT_VOLTS<51.2) digitalWrite(27,0);
//	if((FNDC_BATT_VOLTS<56)  && (!preferHeatPumpOn))digitalWrite(27,0);
	
	BelowSellvSecs+=VDIFF;
	if (BelowSellvSecs>BELOWSELLVSECSMAX) BelowSellvSecs=BELOWSELLVSECSMAX;
	if (BelowSellvSecs<BELOWSELLVSECSTRIG){
		/*if (whsSetFlags(whsAuto,whsInquiry)==whsOn){
			BelowSellvSecs=0;
			whsSetFlags(whsAuto,whsOff);
		}else{*/
			SWITCH_TO_GRID(15);
//		}
		return;
	}
	if (netbattamps<(-120.0))SWITCH_TO_GRID(1)
	if (pause){
		pause--;
		return;
	}
	{static int SOCchanged=0;
		if (SavedSOC==0){
			SavedSOC=SOC;
		}else{ 
			if (abs(SavedSOC-SOC)>1){
				if (++SOCchanged>=5){
					SOCchanged=0;
					SWITCH_TO_GRID(10)
					return;
				}
			}else if (SOCchanged>0) SOCchanged--;
		}
	}
	{static int lt45=0;
		if (FNDC_BATT_VOLTS<45.0){
			if (++lt45>=5){
				lt45=0;
//				whsSetFlags(whsAuto,whsOff);
				SWITCH_TO_GRID(12)
				return;
			} 
		}else if (lt45>0) lt45--;
	}
#define WHSPREAD 		10
#define WHMULTIPLIER 	3
/*	if ((!UnderUtilization) && (whsSetFlags(whsAuto,whsInquiry)==whsOn)	&& (aFNDCvolts<55.0)
							&& (ANBA < ((MIN(((100-WHSPREAD)-SOC),0)*WHMULTIPLIER)))){
		//whsSetFlags(whsAuto,whsOff);
		loadBalance=1;
		BelowSellvSecs=0;
		pause=5;
		return;
	}
	else 
	{		
		if ((whsSetFlags(whsAuto,whsInquiry)!=whsOn) &&
						(((ANBA > ((MIN((100-SOC),WHSPREAD)*WHMULTIPLIER))) && (aFNDCvolts>51.2)) ||
						(aFNDCvolts>56.8))){
			loadBalance= -1;
			whsSetFlags(whsAuto,whsOn);
			pause=5;
			return;
		}
	}*/
	{static int ltNegAmps=0;	
		if (SOC<(MIN_SOC_DROPPED)){
/*			if ((whsSetFlags(whsAuto,whsInquiry)==whsOn) && (!UnderUtilization)){
				whsSetFlags(whsAuto,whsOff);
				pause=5;
			}*/
			SWITCH_TO_GRID(12) 
		}
		else
/*		{
			if (netbattamps<(MaxNegBatAmpsDropped)){
				if (++ltNegAmps>=20){
					ltNegAmps=0;
					SWITCH_TO_GRID(12)
					} 
		}else if (ltNegAmps>0) 
			ltNegAmps--;
		}*/
		{
			ltNegAmps+=(ANBA-(MaxNegBatAmpsDropped));
			if(ltNegAmps<(-600))
			{
				ltNegAmps=0;
				SWITCH_TO_GRID(12)
			}
			if(ltNegAmps>600) ltNegAmps=600;
 		}
	}
}
#define MCRoomTemp	((sensor[6].tempF+sensor[5].tempF)/2.0)
void mrCool(void)
{//MC_On_Temp+=0.25;
	if(MrCoolMode==1)//1 is cool
	{
		if((MCRoomTemp<=MC_Off_Temp) && ((PrgDataStruct.SOC_Targ >= FNDC_SOC)||((CC1_AMPS+CC2_AMPS)<15))){ setACPS(acpsNone);}
		if((MCRoomTemp>=MC_On_Temp)&&(AirCondPwrSrc==acpsNone)) {setACPS(acpsOn);}
	}
	else //0 is heat
	{
		if((MCRoomTemp>=MC_Off_Temp) && ((PrgDataStruct.SOC_Targ >= FNDC_SOC)||((CC1_AMPS+CC2_AMPS)<15))) {setACPS(acpsNone);}
		if((MCRoomTemp<=MC_On_Temp)&&(AirCondPwrSrc==acpsNone)) {setACPS(acpsOn);}
	}
}

void think(void){
static int pause=0, ContinousNegAmpsSec=0, DetectChrgEnabledSecs=0;
int CC1Mode=CC1_MODE, CC2Mode=CC2_MODE;
	if (initilized!=YES) init();
	if(FNDC_EXTRADATAID==6) DaysSinceFull=((float)FNDC_EXTRADATA/10.0);
//need to make this play well with other logic	if (vacation!=TRUE) mrCool();    !!!!!!!!!  10.14.2016
	if (WhLEGridGoal<readSensorF(4,666.6)) wh_le_src=INVERTER;
	digitalWrite(WH_LOWER_ELEMENT_SRC,wh_le_src);
	if (INVERTER_OP_MODE==IOM_CHRG){
		if (InvChrgEnabled!=YES){
			if (DetectChrgEnabledSecs++ >= 5){ 
				InvChrgEnabled=YES;
				DetectChrgEnabledSecs=0;
			}
		}
	}else{
		DetectChrgEnabledSecs=0;
	}
	if (AllowDaytimeInvCharge==FALSE)
	{
		if (InvChrgEnabled==YES)
		{
			enum CC_Modes CC_Mode1=CC1_MODE,CC_Mode2=CC2_MODE;
			if ((((CC1_PVIV>50)&&(CC_Mode1!=Silent)) || ((CC2_PVIV>50)&&(CC_Mode2!=Silent))) && (FNDC_BATT_VOLTS>ChargeDisableTrigV))
			{
				cmdMate("CHG","0",__LINE__);
				InvChrgEnabled=NO;
				ChargeDisableTrigV=51.0;
			}
		}
	}
	UnderUtilization=((CC1Mode==1) || (CC1Mode==3) || (CC2Mode==1) || (CC2Mode==3));
	ngp = ((L1_CHARGER_AMPS+L1_BUY_AMPS-L1_SELL_AMPS)*L1_INVERTER_VOUT)
									+((L2_CHARGER_AMPS+L2_BUY_AMPS-L2_SELL_AMPS)*L2_INVERTER_VOUT);
	updateANGP();
	updateAFNDCvolts();
	invbattv=(int)(INVERTERVOLTS*10);
	netbattamps=((FNDC_SHUNT_A_AMPS)+(FNDC_SHUNT_B_AMPS)+(FNDC_SHUNT_C_AMPS));
	updateANBA();
	if (netbattamps	> 0.0){
		if ((FNDC_BATT_VOLTS <= 51.2) || (/*netbattamps*/ ADJNETBATTAMPS > ((FNDC_BATT_VOLTS-51.0)*1.6)) || (BATT_STATUS_AH<(-0.05)))
		{ 
			BatAmpHrIn+=(netbattamps/(60.0*60.0));
		}
		if (ADJNETBATTAMPS/*netbattamps*/ > 1.0)ContinousNegAmpsSec=0; else if (ContinousNegAmpsSec>1)ContinousNegAmpsSec=1;
	}else{
		BatAmpHrOut-=(netbattamps/(60.0*60.0));
		if ((ContinousNegAmpsSec>0) || (ADJNETBATTAMPS<(-1.0))){ 
			ContinousNegAmpsSec++;
			if ((FNDC_BATT_VOLTS>MaxVNegAmps)&&(ContinousNegAmpsSec>60)) MaxVNegAmps=MIN(FNDC_BATT_VOLTS,50.0);
		}
	}	
	if (((BatAmpHrIn-BatAmpHrOut)-BatAmpHrCrx)>0) BatAmpHrCrx=(BatAmpHrIn-BatAmpHrOut);
	if(((BatAmpHrIn*BattEffFac)-BatAmpHrOut)>(((BatAmpHrIn)-BatAmpHrOut)-BatAmpHrCrx)){//Must adj eff fac
		BattEffFac=(BatAmpHrIn-BatAmpHrCrx)/BatAmpHrIn;		
	}
	InverterPwr=(INVPWR);
	if (UnderUtilization)
	{ 
		UnderUtilizationSecs++;
		{ 
			ALARM_TIMMER(td,900ul,180,240)
			{
				sprintf(strtmp,"Excess power is available and has no where to go!");
				logMesg("SMS_SendMsg returned %d. Msg: %s\n",SMS_SendMsg("2814509680",strtmp),strtmp);
				SoundAlarm(200);
			}
/*			wprintw(ScrollWin,"%lu\t%lu\t%lu\t%d\t%d\t%d\t\%lu\n",__LINE__,
				td.ReAlarmWaitTimer,td.ReAlarmWaitDelay,td.TriggerCountWithinTimerStart
				,td.reqTriggersWithinTicks,td.TriggerCount,td.reqNumTriggers,td.CurrentTick);*/
		}
	}
	switch (INVERTER_AC_MODE)
	{
		case 0:	//no power from grid
			PowerOutage();
			PowerOutageSecs++;
			break;
		case 1:	//grid dropped
			//if (FNDC_BATT_VOLTS>50) whsSetFlags(whsConserve,whsOff);
			POAlarm(3);
			DroppedGrid();
			DroppedSecs++;
			break;
		case 2: //Using grid
			//if (FNDC_BATT_VOLTS>50) whsSetFlags(whsConserve,whsOff);
			POAlarm(3);
			switch (InvInputMode)
			{
				case GridTied:
					ControlGridTieUse();
					break;
				case Support:
					ControlSupportMode();
					break;
				case Backup:
				case UPS:
					if ((FNDC_BATT_VOLTS*10)>(sellv*10)) cmdMate("AC","0",__LINE__);
					break;
				default:
//test					whsSetFlags(whsAuto,whsOff);
					break;
			}
			break;
	}
	if((INVERTER_OP_MODE==INV_IS_OFF) && (INVERTER_AC_MODE!=0))
	{
		cmdMate("INV","2",__LINE__);
		POAlarm(3);
	}
	loadShed();
/*test	if (((L1_INVERTER_AMPS+L1_BUY_AMPS-L1_SELL_AMPS)>45) || ((L2_INVERTER_AMPS+L2_BUY_AMPS-L2_SELL_AMPS)>45)){
		if (digitalRead(WH_LOWER_ELEMENT)!=0) digitalWrite(WH_LOWER_ELEMENT_HV,0);
		else 
		{
			static struct timer_data td;
			static int timerInitilized=0;
			if(timerInitilized!=1) 
				TimerInit(&td, 300ul, 3, 4);
			timerInitilized=1;
			if (TriggerTimer(&td))
			{
				int L1amps=(L1_INVERTER_AMPS+L1_BUY_AMPS-L1_SELL_AMPS);
				int L2amps=(L2_INVERTER_AMPS+L2_BUY_AMPS-L2_SELL_AMPS);
				sprintf(strtmp,"HIGH LOAD! amps= %d on %s",
					MAX(L1amps,L2amps),((L1amps>L2amps)?"L1":"L2"));
				wprintw(ScrollWin,"SMS_SendMsg returned %d. Msg: %s\n",SMS_SendMsg("2814509680",strtmp),strtmp);
				usleep(100000);
//				wprintw(ScrollWin,"SMS_SendMsg returned %d. Msg: %s\n",SMS_SendMsg("4096730837",strtmp),strtmp);				
			}
			wprintw(ScrollWin,"h%dm%d@%d %lu\t%lu\t%lu\t%d\t%d\t%d\t\%lu\n",hour,minute,__LINE__,
				td.ReAlarmWaitTimer,td.ReAlarmWaitDelay,td.TriggerCountWithinTimerStart
				,td.reqTriggersWithinTicks,td.TriggerCount,td.reqNumTriggers,td.CurrentTick);
					
		}
		pause++;
	}*/
	if (pause>0){
		pause--;
	}//else{
	//	switch (whGetDesiredState(/*digitalRead(WH_LOWER_ELEMENT),INVERTER_AUX_OUT,*/
				/*(L1_INVERTER_AMPS+L1_BUY_AMPS-L1_SELL_AMPS),
				(L2_INVERTER_AMPS+L2_BUY_AMPS-L2_SELL_AMPS))){
			case whsOn:
//				if (digitalRead(WH_LOWER_ELEMENT)==0){ 
//					cmdMate("AUXON","1",__LINE__);
					digitalWrite(WH_LOWER_ELEMENT,1);
//				}
				pause=1;
				break;
			case whsOff:
//				if (digitalRead(WH_LOWER_ELEMENT)!=0){
//					cmdMate("AUXOFF","1",__LINE__);
					digitalWrite(WH_LOWER_ELEMENT,0);
//				}
				pause=1;
				break;
			default:
				break;
		}
	}*/
}

int compressor(int SwOn){
	static time_t	lastOffTime=0;
	switch (SwOn)
	{
		case OFF:
			if (digitalRead(COMPRESSOR_CTRL_PIN)){
				digitalWrite(COMPRESSOR_CTRL_PIN,OFF);
				logMesg("@ %d:%d Compressor off.\n",hour,minute);
				lastOffTime=epoch_time;
			}
			break;
		case  ON:
			if (!digitalRead(COMPRESSOR_CTRL_PIN) && difftime(epoch_time,lastOffTime)>180){
				digitalWrite(COMPRESSOR_CTRL_PIN,ON);
				logMesg("@ %d:%d Compressor on.\n",hour,minute);
			}
			break;
		case ASK:
			break;
	}
	return digitalRead(COMPRESSOR_CTRL_PIN);
		
/*	if(SwOn)   INACTIVE!!!!!!!
		cmdMate("AUXON","1",__LINE__);
	else
		cmdMate("AUXOFF","1",__LINE__);*/
}

void ResetAccumulators(void){
	IAO_Day_Secs=0;	IAR_Day_Secs=0;
	SellWH=0; BuyWH=0; InvWH=0;
	ResetTime=time(NULL);
/*	if (current_time == ((time_t)-1))
    {
		exit(1)
    }*/
    localtime_r(&ResetTime,&ResetTime_p);
//	ResetTime_p.tm_hour = localtime(&ResetTime)->tm_hour;
//	ResetTime_p.tm_min = localtime(&ResetTime)->tm_min;
//	ResetTime_p.tm_sec = localtime(&ResetTime)->tm_sec;
}

void ProcessAccumulators(void){
int SellWatts,BuyWatts,InvWatts;	
	shunt_C_monitor();
	SellWatts=((L1_SELL_AMPS-L1_BUY_AMPS)*L1_INVERTER_VOUT)+((L2_SELL_AMPS-L2_BUY_AMPS)*L2_INVERTER_VOUT);
	if (SellWatts>0) SellWH+=((float)SellWatts/3600.0);
	BuyWatts=((L1_BUY_AMPS-L1_SELL_AMPS)*L1_INVERTER_VOUT)+((L2_BUY_AMPS-L2_SELL_AMPS)*L2_INVERTER_VOUT);
	if (BuyWatts>0) BuyWH+=((float)BuyWatts/3600.0);
	InvWatts=(L1_INVERTER_AMPS*L1_INVERTER_VOUT)+(L2_INVERTER_AMPS*L2_INVERTER_VOUT);
	if (InvWatts>0) InvWH+=((float)InvWatts/3600.0);
	if(digitalRead(WH_LOWER_ELEMENT)){ 
	const int WH_Amps_120v=10,WH_Amps_240v=20;	
		if (digitalRead(WH_LOWER_ELEMENT_HV)){ 
			if (((L1_INVERTER_AMPS+L1_BUY_AMPS-L1_SELL_AMPS)>=WH_Amps_240v) && 
				((L2_INVERTER_AMPS+L2_BUY_AMPS-L2_SELL_AMPS)>=WH_Amps_240v)) {
					IAR_Day_Secs++;
			}else if ((L2_INVERTER_AMPS+L2_BUY_AMPS-L2_SELL_AMPS)>=WH_Amps_120v) {
				IAO_Day_Secs++;
			}	
		}
		else
		{
			if ((L2_INVERTER_AMPS+L2_BUY_AMPS-L2_SELL_AMPS)>=WH_Amps_120v) IAO_Day_Secs++;	
		}
	}
	if(digitalRead(WH_UPPER_ELEMENT))	IAO_Day_Secs++;
	if(INVERTER_AUX_RELAY==0) auxRelayOffTicks++;
}
/*
int pulsate(int p_pin,int p_percent)
{
	static int c=10,i=0,p=0,p_pins=0;
	if (p_pin<0)
	{
		if(++i <= p)
		{
			digitalWrite(p_pins,1);
		}
		else
		{
			digitalWrite(p_pins,0);
		}
		if (i>=c)
		{
			i=0;
			wprintw(ScrollWin,"#");
		}
	}
	else
	{
		p_pins=p_pin;
		p=p_percent;
	}
	return digitalRead(p_pins);
}
*/
void ProcessUserInput(void){
		while((ch = getch()) != ERR) {
			POAlarm(2);
			if (ch=='^') { 	//unlock the keyboard
				KBLock=0; 
			}
			else 
			{
				if (KBLock==1) { logMesg("Keyboard locked.\n");return; }
			}
			SoundAlarm(70);
			logMesg("%c key pressed...\n",ch);
			switch (ch)
			{
			case 'A':
				if ((MaxNegBatAmpsDropped*=1.25)<(-125.0)) { MaxNegBatAmpsDropped=(-125); }
				break;
			case 'a':
				MaxNegBatAmpsDropped/=1.25;
				break;
			case 'b':
				cmdMate("BTEMP","?",__LINE__);
				break;
			case 'B':
				BathDone=TRUE;
				break;
			case 'C':
				ChargeDisableTrigV+=1.0;
				if (InvChrgEnabled!=YES){
					//whsSetFlags(whsAuto,whsOff);
					cmdMate("AC1INLM","50",__LINE__);
					cmdMate("CHG","2",__LINE__);
					cmdMate("BULK","1",__LINE__);
					InvChrgEnabled=YES;
				}
				break;
			case 'c':
				ChargeDisableTrigV-=1.0;
				if (InvChrgEnabled==YES){
//					cmdMate("BULK","2");
					cmdMate("CHG","0",__LINE__);
					InvChrgEnabled=NO;//default
				}
				break;
			case 'D': // drop grid now and allow grid tie function to drop
				cmdMate("AC","0",__LINE__);
			  case 'd': // allow grid tie function to drop
				DropSelected=1;
				break;
			case 'e':
			case 'E':
				EnableSystem=YES;
				break;
			case 'f':
				WHtopMinTemp=WHtopMinTemp-6;
				break;
			case 'F':
				WHtopMinTemp=WHtopMinTemp+5;
				break;
			case 'g':
				WHCenterMinTemp=WHCenterMinTemp-6;
				break;
			case 'G':
				WHCenterMinTemp=WHCenterMinTemp+5;
				break;
			case 'H':
				airCondControl(1);	//	turn on if timer allows
				preferHeatPumpOn=TRUE;
				break;
			case 'h':
				airCondControl(2);	//	2 means turn off now ignoring timer
				preferHeatPumpOn=FALSE;
				break;
			case 'I':
				AllowDaytimeInvCharge=TRUE;
				break;
			case 'i':
				AllowDaytimeInvCharge=FALSE;//default
				break;
			case 'j':
				WHpowerLevel(0);
				break;
			case 'J':
				WHpowerLevel(1);
				break;
			case 'K':
				//MrCoolMode=1;//cool
				MC_On_Temp+=0.25;
				break;
			case 'k':
				//MrCoolMode=0;//heat
				MC_On_Temp-=0.25;
				break;				
			case 'L':	//lock the keyboard
				KBLock=1;
				break;
			case 'M':	//Raise the maximum sellv
				SellVoltMax+=4;
				break;
			case 'm':	//Lower the maximum sellv
				if ((SellVoltMax-4)>=SellVoltMin) {SellVoltMax-=4;}
				break;
			case 'n':
				SMS_doSendStatRpt=NO;
				break;
			case 'N':
				SMS_doSendStatRpt=YES;//default
				SEND_SMS_STAT_RPT
				break;
			case 'o':  
				//UE_PWM.Off--;
				//logMesg("powerFactor=%d\n",UE_PWM.Off);
				break;
			case 'O':  
				//UE_PWM.Off++;
				//logMesg("powerFactor=%d\n",UE_PWM.Off);
				break;				
			case 'P':
				ProgramIndx++; if (ProgramIndx>PrgMaxIndx) ProgramIndx=0;
				break;			
			case 'p':
				ProgramIndx--; if (ProgramIndx<0) ProgramIndx=PrgMaxIndx;
				break;
			case 'q':
			case 'Q':
				terminate=true;
				break;
			case 'r':
				BattEffFac=0.93;
//				ResetAccumulators();
				break;
			case 'R':
				BatAmpHrIn=0.0; BatAmpHrOut=0.0; BatAmpHrCrx=0.0;
				break;
			case 'S':
				BatAmpHrIn=0.0; BatAmpHrOut=((100.0-FNDC_SOC)*(BatRatedAmpHr/100.0)); BatAmpHrCrx=0.0;
				break;
			case 's'://save stats
				SaveStats(FNDC_BATT_VOLTS,FNDC_SOC);
				break;
			case 'T':
				cmdMate("SELL","1",__LINE__);
				//whsSetFlags(whsTimer,whsOn);
				break;
			case 't':
				cmdMate("SELL","0",__LINE__);
				//whsSetFlags(whsTimer,whsUnset);
				break;
			case 'U': // use grid now
				cmdMate("AC","1",__LINE__);
			  case 'u': // set use grid flag
				DropSelected=0;//default
				break;
			case 'V':	//Raise the minimum sellv
				if ((SellVoltMin+4)<=SellVoltMax) {SellVoltMin+=4;}
				break;
			case 'v':	//Lower the minimum sellv
				SellVoltMin-=4;
				break;
			case 'W': // turn on water heater
				//whsSetFlags(whsManual,whsOn);
				break;
			case 'w': // turn off water heater
				//whsSetFlags(whsManual,whsOff);
				break;
			case 'Y':	//Allow severe battery drain.  Use with caution!!!
				InvIgnLowVolt=true;
				break;
			case 'y':	//Do not allow severe battery drain.  
				InvIgnLowVolt=false;//default
				break;
			case 'Z': // Discharge Not Allowed
				DischargeAllowed=NO;MaxVNegAmps=0.0;
				break;
			case 'z': // Discharge Is Allowed
				DischargeAllowed=YES;MaxVNegAmps=0.0;//default
				break;	
			case 'X':	
				MC_Off_Temp+=0.25;
				break;
			case 'x':
				MC_Off_Temp-=0.25;	
				break;
			case '+':
				cmdMate("SELLV","+",__LINE__);
				sellv+=4;
				MaxVNegAmps=(sellv/10);
				break;
			case '-':
				cmdMate("SELLV","-",__LINE__);
				sellv-=4;
				MaxVNegAmps=(sellv/10);
				break;
			case '>':
				if ((BatTrgAmp+=2.5)>180) { BatTrgAmp=180; }
				break;
			case '<':
				if ((BatTrgAmp-=2.5)<(-180)) { BatTrgAmp=(-180); }
				break;
			case '0':
				InvInputMode=Gen;
				break;
			case '1':
				InvInputMode=Support;//default
				break;
			case '2':
				InvInputMode=GridTied;
				cmdMate("SELL","1",__LINE__);
				break;
			case '3':
				InvInputMode=UPS;
				break;
			case '4':
				InvInputMode=Backup;
				break;
			case '5':
				InvInputMode=MiniGrid;
				break;
			case	'!':
				digitalWrite(MRCOOL2KHP_PWR_GPIO,1);
				break;
			case	'@':
				digitalWrite(MRCOOL2KHP_PWR_GPIO,0);
				break;
			case 	'#':
				digitalWrite(MRCOOL2KHP_SRC_GPIO,1);
				break;
			case	'$':
				digitalWrite(MRCOOL2KHP_SRC_GPIO,0);
				break;
			case	'(':
				vacation=TRUE;logMesg("vacation=TRUE\n");
				break;
			case	')'	:
				vacation=FALSE;//default
				break;
			case	';'	:
				digitalWrite(WH_LOWER_ELEMENT_SRC,INVERTER);	//low volt managed src from inverter
				wh_le_src=INVERTER;
				WhLEGridGoal=50.0;
				logMesg("Lower WH element on inverter\n");
				break;
			case	':'	:
				if (digitalRead(WH_LOWER_ELEMENT_SRC)==INVERTER)
				{
					WhLEGridGoal=readSensorF(4,666.6)+5.0;
					digitalWrite(WH_LOWER_ELEMENT_SRC,GRID);	//high volt mechanical thermostat src from grid
					wh_le_src=GRID;
					logMesg("Lower WH element on grid\n");
				}
				else
				{
					WhLEGridGoal+=5.0;
				}
				break;
			case	'?'	:
				airJordanManOff=TRUE;
				digitalWrite(AIR_JORDAN_SRC_PIN,0);
				break;
			case	'/'	:
				airJordanManOff=FALSE;
				break;
			}
		}
}

void printStuff(void){

	WMVPRINTW(InvWin,0,2,"BattV %3.1f Volts",INVERTERVOLTS);
	WMVPRINTW(InvWin,1,2,"ACMod %5s",ACMODES[INVERTER_AC_MODE]);
	WMVPRINTW(InvWin,2,2,"OPMod %5s",OPMODES[INVERTER_OP_MODE]);
	WMVPRINTW(InvWin,3,2,"Inv   %2d %2d Amps",L1_INVERTER_AMPS,L2_INVERTER_AMPS);
	WMVPRINTW(InvWin,4,2,"Chg   %2d %2d Amps",L1_CHARGER_AMPS,L2_CHARGER_AMPS);
	WMVPRINTW(InvWin,5,2,"Buy   %2d %2d %6.3f ",L1_BUY_AMPS,L2_BUY_AMPS,BuyWH/1000);
	WMVPRINTW(InvWin,6,2,"Sell  %2d %2d %6.3f",L1_SELL_AMPS,L2_SELL_AMPS,SellWH/1000);
	WMVPRINTW(InvWin,7,2,"NetGP %5d %s Watts",ANGP,(ANGP>0) ? "$$$":"   ");
	WMVPRINTW(InvWin,8,2,"Aux  %2d%2d%2d %2d %2d %2d",(digitalRead(WH_UPPER_ELEMENT)), (digitalRead(WH_LOWER_ELEMENT_HV))
					,(digitalRead(WH_LOWER_ELEMENT)!=0),(/*INVERTER_AUX_OUT!=0*/compressor(ASK)),(vacation),digitalRead(AIR_JORDAN_SRC_PIN));
	WMVPRINTW(InvWin,9,1,"InvPwr %5d %6.3f ", InverterPwr, InvWH/1000);
	WMVPRINTW(InvWin,10,1,"SelV %3.1f >=%3.1f<=%3.1f",(float)sellv / 10.0,(float) SellVoltMin/10.0,(float) SellVoltMax/10.0);
	WMVPRINTW(InvWin,11,1,"Mode %d %8s",InvInputMode,InvInputModeLabels[InvInputMode]);
	WMVPRINTW(InvWin,12,1,"room temp %4.1f      ",/*sensor[3].tempF*/readSensorF(3,-666.9));
	WMVPRINTW(InvWin,12,1,"Wall %5.1f atic %5.1f %1s ",sensor[6].tempF,readSensorF(5,-666.9),((MrCoolMode==1)?"C":"H"));
	WMVPRINTW(InvWin,13,1,"On %4.1f Off %4.1f %1s    ",/*sensor[5].tempF*/MC_On_Temp,MC_Off_Temp,(MC_On_Temp>MC_Off_Temp)?"C":"H");
	WMVPRINTW(CCWin,0,2,"BattV  %4.1f %4.1f",CC1_BATT_VOLTS,CC2_BATT_VOLTS);
	WMVPRINTW(CCWin,1,2,"%s","                      ");
	WMVPRINTW(CCWin,1,2,"Prg %1d %s",ProgramIndx,PrgDataStruct.Description /*Programs[ProgramIndx]*/ );
	WMVPRINTW(CCWin,2,2,"KWH    %4.1f %4.1f %4.1f",CC1_KWHH,CC2_KWHH,CC1_KWHH+CC2_KWHH);
	WMVPRINTW(CCWin,3,2,"AmpHrs %4d %4d %4d",CC1_AHA,CC2_AHA,CC1_AHA+CC2_AHA);
	WMVPRINTW(CCWin,4,2,"MODE  %5s %5s",CCModes[CC1_MODE],CCModes[CC2_MODE]);
	WMVPRINTW(CCWin,5,2,"PVIV   %4d %4d",CC1_PVIV,CC2_PVIV);
	WMVPRINTW(CCWin,6,2,"OUTAMP %4.1f %4.1f",CC1_AMPS,CC2_AMPS);
	WMVPRINTW(CCWin,7,2,"Watts  %4.0f %4.0f %5.0f",CC1_AMPS*CC1_BATT_VOLTS,CC2_AMPS*CC2_BATT_VOLTS,\
									CC1_AMPS*CC1_BATT_VOLTS+CC2_AMPS*CC2_BATT_VOLTS);
	WMVPRINTW(CCWin,8,2,"%Rating %5.1f%%",	(CC1_AMPS*CC1_BATT_VOLTS+CC2_AMPS*CC2_BATT_VOLTS)/(ArrayRatedWatts/100));

	WMVPRINTW(CCWin,9,2,"Ctrl Rm %5.1f %5.1f ",readSensorF(3,-666.9),WhLEGridGoal);
	WMVPRINTW(CCWin,10,2,"%02d:%02d:%02d  %02d:%02d:%02d",hour,minute,second,ResetTime_p.tm_hour,ResetTime_p.tm_min
															,ResetTime_p,ResetTime_p.tm_sec);
	WMVPRINTW(CCWin,11,2,"   %5.1f    %3d%% >%3.0f",readSensorF(2,666.6),UE_PWM.percent,WHtopMinTemp);									
	WMVPRINTW(CCWin,12,2,"%5.1f %5.1f %3d%% >%3.0f", readSensorF(4,666.6),readSensorF(1,666.6),LE_PWM.percent
									, WHCenterMinTemp);
	WMVPRINTW(CCWin,13,2,"%1s  %5.1f     %5.1f",((digitalRead(WH_LOWER_ELEMENT_SRC)==INVERTER)?"I":"G"),readSensorF(0,666.6) 
											,readSensorF(7,777.7));

	WMVPRINTW(FNDCWin,0,2,"BattV %3.1f ",AFNDCV/*FNDC_BATT_VOLTS*/);
	WMVPRINTW(FNDCWin,1,2,"SOC   %4d%% Days %4.1f    G %06.0f ",FNDC_SOC,DaysSinceFull,CwattHoursGen);
	WMVPRINTW(FNDCWin,2,2,"Amps %6.0f %5.0f %5.1f  P %06.0f ",FNDC_SHUNT_A_AMPS,FNDC_SHUNT_B_AMPS,FNDC_SHUNT_C_AMPS,CwattHoursPump);
	WMVPRINTW(FNDCWin,3,2,"Net Amps%6.1f             ",netbattamps);
	WMVPRINTW(FNDCWin,4,2,"InvOut/In %6.1f%% ",(INVPWR/((0-FNDC_SHUNT_A_AMPS)*FNDC_BATT_VOLTS))*100);
	if (InvInputMode != GridTied)	WMVPRINTW(FNDCWin,5,18,"%4d",AC1InLimit);
	WMVPRINTW(FNDCWin,6,2,"ACPS %4s S %d P %d",acpsModeDesc[AirCondPwrSrc],
						digitalRead(MRCOOL2KHP_SRC_GPIO),digitalRead(MRCOOL2KHP_PWR_GPIO));
	WMVPRINTW(FNDCWin,7,1,"MaxNegA Dropped %5.1f ",MaxNegBatAmpsDropped);
	WMVPRINTW(FNDCWin,8,2,"KB Locked %s ",((KBLock) ? "Yes Press ^":"No Press L "));
	WMVPRINTW(FNDCWin,10,2,"%s         ","  ");
	WMVPRINTW(FNDCWin,3,18,"%s","              ");
	if (InvInputMode==GridTied){
		WMVPRINTW(FNDCWin,12,2,"%s         ",((DropSelected) ? "DS":"--"))
		WMVPRINTW(FNDCWin,3,18,"AvgNBA: %4.1f",ANBA);
	}
	else if (InvInputMode==Support){
		WMVPRINTW(FNDCWin,3,18,"Target: %4.1f",BatTrgAmp);
		
	}
	WMVPRINTW(FNDCWin,10,1,"Net E %6.2f IO %-6.2f Fac %6.3f%%  "
					,BATT_STATUS_AH, BATT_STATUS_AH_RAW, BattEffFac*100.0);
	WMVPRINTW(FNDCWin,11,2,"WaterHeater %5.3f  ",WaterHeaterKWH);
	WMVPRINTW(FNDCWin,12,2,"Batt Temp %4.1f F     ",FNDC_BATT_TEMP);
	WMVPRINTW(FNDCWin,13,2,"%33s","   	                                ");
	WMVPRINTW(FNDCWin,13,2,"%33s",USBlastResponse);

	wrefresh(InvWin);
	wrefresh(CCWin);
	wrefresh(FNDCWin);
	wrefresh(ScrollWin);
}
/*
int StoreData(void){
char c;
int i;
	i=0; 
	do{
		c = buf[i++];
		if (c == ']'){
			fprintf(fp,"]<0,0,%lld>",(long long) time(NULL));
		}
		else if (c == '['){
			fputc(10,fp);
			fputc(91,fp);
			_flushlbf();
			fflush(fp);
		}
		else{
			fputc(c,fp);
		}
	}while(c != 0);
	wprintw(ScrollWin,"-->%s\n", buf);
	wrefresh(ScrollWin);
	return 0;

}
*/
void ConnectDB(void){
sqlite3 *db;
	sqlite3_open("test",&db);
}

void IdleLoop()
{
	//Do stuff here while waiting for another data packet
//	usleep(5000);
	int i;
	for(i=0;i<5;i++){
		USBNano_ReadLine();
		usleep(2000);
	}
}

void signalIntHandler(int signo)
{
	if (signo==SIGINT) terminate = true;
}

#define POWER(ohms,volts)	((volts^2)/ohms)
	
PI_THREAD (togglePin)
{
	const int CycleMicroSec=16480;

	while(1)
	{	
		PWM_Debug_count++;
		if((UE_PWM.count+=(100/UE_PWM.resolution)) >= 100)
		{
			if(UE_PWM.percent>0)digitalWrite(WH_UPPER_ELEMENT,1);
			UE_PWM.count=0;
		}else{
			if(UE_PWM.count >= UE_PWM.percent)digitalWrite(WH_UPPER_ELEMENT,0);
		}
		
		if((LE_PWM.count+=(100/LE_PWM.resolution)) >= 100)
		{
			if(LE_PWM.percent>0)digitalWrite(WH_LOWER_ELEMENT,1);
			LE_PWM.count=0;
		}else{
			if(LE_PWM.count >= LE_PWM.percent)digitalWrite(WH_LOWER_ELEMENT,0);
		}
		delayMicroseconds(CycleMicroSec);//delay one cycle length
	}
	return 0;
}

int main( int argc, char *argv[]) 
{ 

	terminate=false;
	if (signal(SIGINT,signalIntHandler)== SIG_ERR) return 5;
	UE_PWM.count=0;
	UE_PWM.resolution=5;
	UE_PWM.percent=0;
	LE_PWM.count=40;
	LE_PWM.resolution=5;
	LE_PWM.percent=0;
	piThreadCreate(togglePin);

	IAO_Day_Secs=0;
	IAR_Day_Secs=0;
	SellWH=0;
	invMode=(-1);
	wiringPiSetupGpio () ;
	pinMode(AIR_COND_GPIO_PIN,OUTPUT);
	pinMode(AIR_JORDAN_SRC_PIN,OUTPUT);
	pinMode(PIN_ALARM,OUTPUT);
#ifdef	__USE_PWM__	
	softPwmCreate(WH_LOWER_ELEMENT,0,150);//9 60hz cycles -- 16.667 per cycle
#else	
	pinMode(WH_LOWER_ELEMENT,OUTPUT);
#endif	
	pinMode(WH_LOWER_ELEMENT_HV,OUTPUT);
	pinMode(WH_LOWER_ELEMENT_SRC,OUTPUT);
	pinMode(WH_UPPER_ELEMENT,OUTPUT);
	pinMode(MRCOOL2KHP_PWR_GPIO,OUTPUT);
	pinMode(MRCOOL2KHP_SRC_GPIO,OUTPUT);
	pinMode(COMPRESSOR_CTRL_PIN,OUTPUT);
	digitalWrite(WH_LOWER_ELEMENT_SRC,INVERTER);
	initscr();
	InvWin = newwin(15, 25, 0, 0);
	CCWin = newwin(15,25,0,26);
	FNDCWin=newwin(15,40,0,52);
	ScrollWin = newwin(25, 85, 16, 0);
	scrollok(ScrollWin,true);
    if (has_colors())
    {
        start_color();

        /*
         * Simple color assignment, often all we need.  Color pair 0 cannot
         * be redefined.  This example uses the same value for the color
         * pair as for the foreground color, though of course that is not
         * necessary:
         */
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        init_pair(2, COLOR_GREEN,   COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_BLUE,    COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(7, COLOR_WHITE,   COLOR_BLACK);
        init_pair(8, COLOR_YELLOW,  COLOR_BLUE);
    }
	wcolor_set(CCWin, 7,0);	
	wborder(InvWin, 0, 0, 0, 0, 0, 0, 0, 0);
	wborder(CCWin, 0, 0, 0, 0, 0, 0, 0, 0);
	wborder(FNDCWin, 0, 0, 0, 0, 0, 0, 0, 0);
	attron(COLOR_PAIR(8));
	nodelay(stdscr, TRUE);
	noecho();
	OPMODES[90]="InvEr";OPMODES[91]="AGSEr";OPMODES[92]="ComEr";
		
	ResetTime=time(NULL);
	epoch_time = time( NULL );
	tm_p = localtime( &epoch_time );
	hour=tm_p->tm_hour;
	minute=tm_p->tm_min;
	second=tm_p->tm_sec;

	ResetAccumulators();
	if(sqlite3_open_v2("/home/pi/projects/MateMonitor/MateConf.db",&ppDb,SQLITE_OPEN_READONLY,NULL)!=SQLITE_OK){
		WMVPRINTW(FNDCWin,5,2,"%14s %d %d %d","sqlite3 failed",hour,minute,__LINE__);
		return 3;
	}
	if(sqlite3_open_v2("/home/pi/projects/MateMonitor/MateMonitor.db",&MMDb,SQLITE_OPEN_READWRITE,NULL)!=SQLITE_OK){
		WMVPRINTW(FNDCWin,6,2,"%14s %d %d %d","sqlite3 failed",hour,minute,__LINE__);
		return 4;
	}
	USB_init();
	USBNano_init();
/* Read from the socket */ 
	do{
		static int lastSec=0;
//		if (read(sock, buf, 1024) < 0) perror("receiving datagram packet"); 
		USB_ReadLine();
//		DebugPrintData();
		epoch_time = time( NULL );
		tm_p = localtime( &epoch_time );
		dow=tm_p->tm_wday;
		hour=tm_p->tm_hour;
		minute=tm_p->tm_min;
		second=tm_p->tm_sec;
		if (second==lastSec)
		{
			IdleLoop();
		}
		else
		{
			lastSec=second;
			Ticks++;
			tick();
//		if (MateCommResponseExpected) MateCommResponse();
			GetPrgData(ProgramIndx,hour);
/*		StoreData();*/
			ProcessAccumulators();
			ProcessUserInput();
			if (EnableSystem==YES){
				TimeEvents();
				think();
			}
			printStuff();
		}
	}while(terminate==false);  // pressing 'q' will cause terminate to go true
//tidy up
	SaveStats(FNDC_BATT_VOLTS,FNDC_SOC);
	sqlite3_close(ppDb);
	sqlite3_close(MMDb);
	USB_Close();
	USBNano_Close();
//	close(sock); 
/*	fclose(fp);*/
	close(usbmate);
//	fclose(mmlog);
	persistantData=fopen("persistant.ini","w");
	if(persistantData==NULL){
		perror("Unable to open persistant.ini");
		exit(1);
	}
	fprintf(persistantData,"%f        %f       ", CwattHoursPump, CwattHoursGen);
	fclose(persistantData);
	echo();
	attroff(COLOR_PAIR(8));
	endwin();
	exit(0);
}

