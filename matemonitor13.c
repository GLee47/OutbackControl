//MateMonitor.c
#define __MateMonitor_c__

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ncurses.h>
#include "outback.h"
//#include "/mnt/sda1/home/glee/projects/MateMonitor/usb_com.h"
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
					wprintw(ScrollWin,"%d Send returned %d - %s\n",__LINE__,SMS_SendMsg("2814509680",strtmp),strtmp);}

enum CC_Modes{ Silent=0, Float=1, Bulk=2, Absorb=3, Equlize=4 };
enum {	Gen, Support, GridTied, UPS, Backup, MiniGrid } InvInputMode=Support;
const char * InvInputModeLabels[] = {"Gen", "Support","GridTied","UPS","Backup","MiniGrid"};
const char * ACMODES[] = {"NoGr","Drop","Use "};
char * OPMODES[93] = {"InOff","Searc","InvOn","Chrge","Silnt","Float","Eqlze","ChOff","Suppt","SelEn"\
							,"PasTh","11","12","13","OffSt"};
const char * CCModes[] = {"Silnt","Float","Bulk","Absrb","Equlz"};
const char * Programs[] = {"Conserve(Stormy)","High Demand&PC  ","Mild Demand&PC  ","Const SOC 80%   "
							,"Constant SOC 90 ","Volt Test 50.0  ","Volt Test 48.8  ","Volt Test 48.0  "
							,"Volt fac2 49.0  ","Mild Dmnd&Sun Md","Mild Demand&Sun ","11 Modified 3   "};
int ProgramIndx=0;
#define	PrgMaxIndx	11
unsigned long Ticks=0/*roughly = seconds since program comenced*/,auxRelayOffTicks=0;
const float ArrayRatedWatts = 245.0*30.0, BatRatedAmpHr = 225.0/*rating*/ * 2.0/*# of strings*/;

char  strtmp[256];

WINDOW * InvWin,* CCWin,* FNDCWin,* ScrollWin;
int ch;
volatile bool terminate=false;
bool initilized=NO, EnableSystem=NO, DischargeAllowed=YES, AllowDaytimeInvCharge=FALSE,
				preferHeatPumpOn=FALSE,InvChrgEnabled=NO, MateCommResponseExpected=NO, InvIgnLowVolt=false;
int CurrentDay, DroppedSecs=0, PowerOutageSecs=0, UnderUtilizationSecs=0;
long IAO_Day_Secs, IAR_Day_Secs;
int AC1InLimit=45, BatTrgV=512, SOCstart=100;
float BatAmpHrIn=0.0, BatAmpHrOut=0.0, BatAmpHrCrx=0.0, MaxVNegAmps=0.0,BattEffFac=0.91,ChargeDisableTrigV=51.0;
float SellWH,BuyWH,InvWatts,InvWH, netbattamps=0.0, BatTrgAmp=0.0, MaxNegBatAmpsDropped=(-MAX_NEG_NBA_DROPPED);
float WHtopMaxTemp=170.1,WHtopMinTemp=100.0,WHCenterMinTemp=50.0,WHmaxAnyTemp=178.0;
int ngp, invMode, selling, buying, sellv=0, invbattv=0, KBLock=0, DropSelected=0, AmpsBelowThresholdWaitSecs=5;
int InverterPwr=0, SellVoltMin=DEFAULT_SELL_V_MIN, SellVoltMax=DEFAULT_SELL_V_MAX, UnderUtilization;
time_t epoch_time,ResetTime;
struct tm *tm_p, ResetTime_p;
int dow=0,hour=0,minute=0,second=0;
enum AirCondPwrSrcModes AirCondPwrSrc=acpsGrid;
const char * acpsModeDesc[]={"Grid","Inv","Off"};
enum WHCMD{ cNone,cOff,cOn,cCheck,cInq, cSet, cUnset };
enum WHFlags{ fNone, fSetOverLoad, fSetManLck, fRelManLck, fManual, fAuto, fOverLoad, fManLck, fAllowTimer, fTimer};
sqlite3 *ppDb, *MMDb;

#define TimeTest(HOUR,MIN,SEC)	(((HOUR==hour)||(HOUR<0)) && ((MIN==minute)||(MIN<0)) && ((SEC==second)||(SEC<0)))

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
}PrgDataStruct;

void cmdMate(char * cmd, char * v, int ibookmark){
static int iLastBookmark=0;
	if ((write (usbmate,"<",1))<0)
	{
		wprintw(ScrollWin,"Error writing to usb\n");
	}
	write (usbmate,cmd,strlen(cmd));
	write (usbmate,":",1);
	write (usbmate,v,strlen(v));
	write (usbmate,">",1);
	WMVPRINTW(FNDCWin,9,2,"LastCMD <%7s:%-4s>%4d %4d",cmd,v,ibookmark,iLastBookmark);
	wprintw(ScrollWin,"LastCMD <%7s:%-4s>%4d %4d\n",cmd,v,ibookmark,iLastBookmark);
	iLastBookmark=ibookmark;
	MateCommResponseExpected=YES;
}

void init(){
sqlite3_stmt *stmt;
char sql[4096];
	initilized=YES;
	cmdMate("SELLV","512",__LINE__);
	sellv=512;
	cmdMate("AC1INLM","45",__LINE__);	
//	BatAmpHrCrx=(100.0-(float)(FNDC_SOC))*(BatRatedAmpHr/100.0);
	strcpy(sql,"select WHKWH,BuyKWH,InvertedKWH,DroppedMin,PowerOutageMin,UnderUtilizationMin"
		",BattAHin,BattAHout,BatAmpHrCrx,debug3 from stats WHERE TimeStamp=(SELECT MAX(TimeStamp) FROM stats)");
	wprintw(ScrollWin,"At init()%d\n",0);
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
		WHCenterMinTemp=110.0;
	}
	if((hour>15) && (hour<=17)){
		WHCenterMinTemp=125;
		WHtopMinTemp=130;
	}
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
		sprintf(sql,"SELECT * FROM PrgData WHERE PrgIndx=%d and HrOfDay=%d"
														,Prog,HOD);
		if (sqlite3_prepare_v2(ppDb, sql, strlen(sql)+1, &stmt, NULL)==SQLITE_OK){
			if (sqlite3_step (stmt)==SQLITE_ROW){
				PrgDataStruct.Indx=sqlite3_column_int(stmt,0);
				PrgDataStruct.HOD=sqlite3_column_int(stmt,1);
				PrgDataStruct.SOC_Targ=sqlite3_column_int(stmt,2);
				PrgDataStruct.SellV=sqlite3_column_int(stmt,3);
				PrgDataStruct.factor=sqlite3_column_int(stmt,4);
				sqlite3_finalize(stmt);
			}//else terminate=true;
			sqlite3_finalize(stmt);
		}
		sprintf(sql,"SELECT * FROM PrgData WHERE PrgIndx=%d and HrOfDay=%d"
														,Prog,((HOD<=22) ? (HOD+1) : 0 ));
		if (sqlite3_prepare_v2(ppDb, sql, strlen(sql)+1, &stmt, NULL)==SQLITE_OK){
			if (sqlite3_step (stmt)==SQLITE_ROW){
				PrgDataStruct.NextIndx=sqlite3_column_int(stmt,0);
				PrgDataStruct.NextHOD=sqlite3_column_int(stmt,1);
				PrgDataStruct.NextSOC_Targ=sqlite3_column_int(stmt,2);
				PrgDataStruct.NextSellV=sqlite3_column_int(stmt,3);
				PrgDataStruct.Nextfactor=sqlite3_column_int(stmt,4);
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
	wprintw(ScrollWin,"Recieved -%s- from %s %d %d\n",mesg->MsgText,mesg->PhoneNum,hour,minute);
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
			wprintw(ScrollWin,"found c @ %d %s\n",__LINE__,mesg);
			if (mesg->MsgText[1]=='0')
			{
				wprintw(ScrollWin,"%d found c @ %d %s\n",InvChrgEnabled,__LINE__,mesg);
				if (InvChrgEnabled==YES){
//					cmdMate("BULK","2");
					cmdMate("CHG","0",__LINE__);
					InvChrgEnabled=NO;
				}
			}
			else
			{
				if (InvChrgEnabled!=YES){
					whsSetFlags(whsAuto,whsOff);
					cmdMate("AC1INLM","50",__LINE__);
					cmdMate("CHG","2",__LINE__);
					cmdMate("BULK","1",__LINE__);
					InvChrgEnabled=YES;
				}
			}
			break;
		case 'K':
		case 'k':
			if (mesg->MsgText[1]=='0')
				whsSetFlags(whsDisableTimer,whsUnset);
			else
				whsSetFlags(whsDisableTimer,whsOn);
			break;				
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
		case 'T':
		case 't':
			if (mesg->MsgText[1]=='0')
				whsSetFlags(whsTimer,whsUnset);
			else
				whsSetFlags(whsTimer,whsOn);
			break;
	}
}

void TimeEvents(void){
static float AvHrVolts=0.0;
static long c=0L, AvHrSOC=0L;
//char tBuf[0x100];

	++c; AvHrVolts+=FNDC_BATT_VOLTS; AvHrSOC+=(long)FNDC_SOC;
//static enum LockStates InvAuxLockSaved=Undefined;
	if (TimeTest(0,30,0))	setACPS(acpsNone);
//	5AM
	if (TimeTest(5,30,0))	AmpsBelowThresholdWaitSecs=25;
//	7AM
	if (TimeTest(7,0,0))
	{
		if (InvInputMode==GridTied)
		{	
			MaxNegBatAmpsDropped=(-40);
			DropSelected=1;
			SellVoltMin=504;
		}
		SEND_SMS_STAT_RPT
	}

	if (TimeTest(7,15,0)){
		whsSetFlags(whsTimer, whsOff);
		WHCenterMinTemp=50.0;
		if(WHCenterMinTemp<110.0) WHtopMinTemp=110.0;
	}
	
//8AM Mon,Wed,Fri
/*	if((dow==1) || (dow==3) || (dow==5))*/
		if(TimeTest(8,0,0))	setACPS(acpsInverter);/*{
			WHtopMinTemp=125.0;
			WHCenterMinTemp=125.0;		
		}
	*/		
//10AM
	if (TimeTest(10,0,0)){
		AmpsBelowThresholdWaitSecs=5;
		if (InvInputMode==GridTied)DropSelected=0;
		SEND_SMS_STAT_RPT
	}
	if (TimeTest(12,50,0))WHtopMinTemp=120.0;
//1:15PM
	if (TimeTest(13,15,0)){
		if (WaterHeaterKWH<0.2){
			whsSetFlags(whsTimer, whsOn);
		}else{
			whsSetFlags(whsTimer, whsOff);
		}
		if(WHCenterMinTemp<115.0) WHCenterMinTemp=115.0;
	}
//2PM
	if (TimeTest(14,0,0) && (FNDC_SOC<93) && (InvInputMode==GridTied)){
		if (sellv<508){
			cmdMate("SELLV","508",__LINE__);
			sellv=508;
		}
		if (SellVoltMin<508){ SellVoltMin=508; }
		if (MaxNegBatAmpsDropped<(-30)){MaxNegBatAmpsDropped=(-30);}
		
	}
//2:15PM
	if (TimeTest(14,15,0)){
		if (WaterHeaterKWH<1.2){
			whsSetFlags(whsTimer, whsOn);
		}else{
			whsSetFlags(whsTimer, whsOff);
		}
//		if(ProgramIndx==11 || ProgramIndx==7 || ProgramIndx==3) DischargeAllowed=NO;MaxVNegAmps=0.0;
	}
	if (TimeTest(14,30,0))WHtopMinTemp=125.0;
//3PM
	if (TimeTest(15,0,0) && (FNDC_SOC<94)  && (InvInputMode==GridTied)){
		if (sellv<512){
			cmdMate("SELLV","512",__LINE__);
			sellv=512;
		}
		if (SellVoltMin<512){ SellVoltMin=512; }
		if (MaxNegBatAmpsDropped<(-25)){MaxNegBatAmpsDropped=(-25);}
	}
	
//	if (TimeTest(15,0,0))
//3:15PM
	if (TimeTest(15,15,0)){
		if (WaterHeaterKWH<2.2){
			whsSetFlags(whsTimer, whsOn);
		}else{
			whsSetFlags(whsTimer, whsOff);
		}
		SEND_SMS_STAT_RPT
	}
//4PM
	if (TimeTest(16,0,0) && (FNDC_SOC<95) && (InvInputMode==GridTied)){
		if (sellv<516){
			cmdMate("SELLV","516",__LINE__);
			sellv=516;
		}
		if (SellVoltMin<516){ SellVoltMin=516; }
		if (MaxNegBatAmpsDropped<(-20)){MaxNegBatAmpsDropped=(-20);}
	}
//4:15PM
	if (TimeTest(16,15,0) /*&& InvAuxLockSaved!=Undefined*/){
		whsSetFlags(whsTimer, whsOff);
		SEND_SMS_STAT_RPT
		if(WHtopMinTemp<135.0) WHtopMinTemp=135.0;
		if(WHCenterMinTemp<125.0) WHCenterMinTemp=125.0;		
	}
//5PM
	if (TimeTest(17,0,0) && (FNDC_SOC<97) && (InvInputMode==GridTied)){
		if (sellv<524){
			cmdMate("SELLV","524",__LINE__);
			sellv=524;
		}
		if (SellVoltMin<524){ SellVoltMin=524; }
		if (MaxNegBatAmpsDropped<(-10)){MaxNegBatAmpsDropped=(-10);}
	}
//6PM
	if (TimeTest(18,0,0) && (InvInputMode==GridTied)){
		whsSetFlags(whsTimer, whsOff);
		if (sellv<528){
			cmdMate("SELLV","528",__LINE__);
			sellv=528;
		}
		if (SellVoltMin<548){ SellVoltMin=548; }
		MaxNegBatAmpsDropped=(0);
	}
//6:15PM
	if (TimeTest(18,15,0)){
		whsSetFlags(whsTimer, whsOff);
		WHCenterMinTemp=50.0;
		if(WHtopMinTemp>115.0) WHtopMinTemp=115.0;
	}	
	if (TimeTest(21,0,33))
	{
		if (FNDC_SOC>95)//(InvChrgEnabled!=YES)
		{
			whsSetFlags(whsAuto,whsOff);
			cmdMate("AC1INLM","45",__LINE__);
			cmdMate("CHG","2",__LINE__);
			cmdMate("BULK","1",__LINE__);
			InvChrgEnabled=YES;
		}
	}
	if (TimeTest(22,0,0)){
		whsSetFlags(whsTimer, whsOff);
		WHCenterMinTemp=50.0;
		if(WHtopMinTemp>100.0) WHtopMinTemp=100.0;
	}

//Email Report broken on this pc
	if (TimeTest(28,58,30)){
		sprintf(strtmp,"Hr:%d V:%3.1f | SOC:%d | WH:%5.3f | Buy:%2.0f | IKWH:%2.0f | KWH %4.1f %4.1f %4.1f | Drp Min %4d | Outage Min %4d | Underutilization Min %4d | In %5.2f Out %5.2f | SOC S %3d E %3d"
				,hour, FNDC_BATT_VOLTS, FNDC_SOC,WaterHeaterKWH,BuyWH/1000.0,InvWH/1000.0
				,CC1_KWHH,CC2_KWHH,CC1_KWHH+CC2_KWHH, (DroppedSecs/60),(PowerOutageSecs/60),(UnderUtilizationSecs/60)
				,BatAmpHrIn,BatAmpHrOut, SOCstart, FNDC_SOC);
//		sendmail("2814509680@vtext.com","garlee55@gmail.com","Report",strtmp);
		sendmail("garlee55@gmail.com","garlee55@gmail.com","Report",strtmp);
//		SaveStats();
//		beep;
//		BatAmpHrIn=0.0;
//		BatAmpHrOut=0.0;
	}	
//Every hour on the hour
	if (TimeTest(-1,0,0)){ 
		SaveStats((AvHrVolts/c),(int)(AvHrSOC/c));whsSetFlags(whsTimer, whsOff);
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
		DischargeAllowed=YES;MaxVNegAmps=0.0;
		whsSetFlags(whsTimer, whsOff);
	}
// on the half minute
	if (TimeTest(-1,-1,30)){
//		char msgBuf[0x100];
		struct SMS_Message_Properties msgBuf;
		if (SMS_Rec_Mesg(&msgBuf)!=NULL)
			ProcessSMSmesg(&msgBuf);
	}
}

void PowerOutage(void)
{	
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
					, FNDC_BATT_VOLTS, FNDC_SOC,netbattamps);
			wprintw(ScrollWin,"SMS_SendMsg returned %d. Msg: %s\n",SMS_SendMsg("2814509680",strtmp),strtmp);
			usleep(100000);
			wprintw(ScrollWin,"SMS_SendMsg returned %d. Msg: %s\n",SMS_SendMsg("4096730837",strtmp),strtmp);
		}
			wprintw(ScrollWin,"h%dm%d@%d %lu\t%lu\t%lu\t%d\t%d\t%d\t%lu\n",hour,minute,__LINE__
				,td.ReAlarmWaitTimer,td.ReAlarmWaitDelay,td.TriggerCountWithinTimerStart
				,td.reqTriggersWithinTicks,td.TriggerCount,td.reqNumTriggers,td.CurrentTick);
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

	if (FNDC_BATT_VOLTS<54.0){
		whsSetFlags(whsConserve,whsOn);
//		digitalWrite(27,0);
	}else{
		if (FNDC_BATT_VOLTS>56.0) whsSetFlags(whsConserve,whsOff);
//		if (preferHeatPumpOn) digitalWrite(27,1);
	}
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
		whsSetFlags(whsAuto,whsOff);
		pause=60;
		return;
	}
	else if ((INVERTER_OP_MODE==IOM_OFFSET) && ((netbattamps-BatTrgAmp)>20)){//new 02/11/2015
		ALARM_TIMMER(td,60ul,60,90){
			DROP_GRID(5,((DischargeAllowed==YES)?((0-BatTrgAmp)+20):5));
		}
/*		wprintw(ScrollWin,"Offset trig h%dm%d@%d %lu\t%lu\t%lu\t%d\t%d\t%d\t%lu\n",hour,minute,__LINE__
				,td.ReAlarmWaitTimer,td.ReAlarmWaitDelay,td.TriggerCountWithinTimerStart
				,td.reqTriggersWithinTicks,td.TriggerCount,td.reqNumTriggers,td.CurrentTick);
*/
	}
	else if ((sellv<(FNDC_BATT_VOLTS*10)) && ((L1_CHARGER_AMPS+L2_CHARGER_AMPS)>0)){
		ALARM_TIMMER(td,60ul,30,60){
			DROP_GRID(5,((DischargeAllowed==YES)?((0-BatTrgAmp)+20):5));
		}
/*		wprintw(ScrollWin,"Charger trig h%dm%d@%d %lu\t%lu\t%lu\t%d\t%d\t%d\t%lu\n",hour,minute,__LINE__
				,td.ReAlarmWaitTimer,td.ReAlarmWaitDelay,td.TriggerCountWithinTimerStart
				,td.reqTriggersWithinTicks,td.TriggerCount,td.reqNumTriggers,td.CurrentTick);		*/
	}
	if (pau>0)pau--;
	if (AdjustedSellV<SUPPORT_MODE_MIN_ADJ_SELL_VOLTS) AdjustedSellV=SUPPORT_MODE_MIN_ADJ_SELL_VOLTS;
	if (AdjustedSellV>SUPPORT_MODE_MAX_ADJ_SELL_VOLTS) AdjustedSellV=SUPPORT_MODE_MAX_ADJ_SELL_VOLTS;
	if ((AdjustedSellV<(int)(MaxVNegAmps*10.0))&&(DischargeAllowed!=YES)) AdjustedSellV=(int)(MaxVNegAmps*10.0);
	if (InvInputMode != GridTied) WMVPRINTW(FNDCWin,6,2,"%s                       ","");
	if (ngp<0){Selling++;}else{Selling=0;}
	if (FNDC_BATT_VOLTS<45){
		cmdMate("AC1INLM","48",__LINE__);
		whsSetFlags(whsConserve,whsOn);
		AC1InLimit=50;
		pause=60;
		return;
	}else{
		if (pause<1) whsSetFlags(whsConserve,whsOff);
	}
	if (Selling>5){Selling=0; DROP_GRID(5,((PrgDataStruct.SOC_Targ>99)?125:((DischargeAllowed==YES)?20:5)))}
	if (INVERTER_OP_MODE==IOM_OFFSET){
		if (AC1InLimit < (BuyCurMax + 3)){
			AC1InLimit = (BuyCurMax + 3);
			sprintf(strtmp,"%d",AC1InLimit);
			cmdMate("AC1INLM",strtmp,__LINE__);//raising limit
			WMVPRINTW(FNDCWin,5,2,"AC1InLimit %4d   ",AC1InLimit);				
		} 
		pause=5; 
	}	
	if (pause>0){
		pause--;
		return;
	}
	BatTrgAmp=(SOCVTT*4.5); 
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
		whsSetFlags(whsAuto,whsOff);
		return;
	}
	AdjustedSellV=(((int)((AdjustedSellV+(4/2))/4.0))*4);//round to nearest voltage resoultion for radian - 0.4v
	if (AdjustedSellV!=sellv){
		if (pau<1){
			ADJ_SELLV(AdjustedSellV);
			pau=60;//don't change but once a minute
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
		if (((INVERTER_OP_MODE!=IOM_OFFSET) && (FNDC_BATT_VOLTS*10)<(sellv-ANTIHYSTERESIS)) && (AC1InLimit<45) && (BuyCurMax>=AC1InLimit)){
			AC1InLimit=BuyCurMax+1;
			sprintf(strtmp,"%d",AC1InLimit);
			cmdMate("AC1INLM",strtmp,__LINE__);
			WMVPRINTW(FNDCWin,5,2,"AC1InLimit %4d   ",AC1InLimit);
			whsSetFlags(whsAuto,whsOff);	//WATERHEATEROFF;
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
	int LoadDifWats=(BatTrgAmp-netbattamps)*INVERTERVOLTS;
		if (abs(LoadDifWats)>300) 
		{
		int ACLoadDifAmps=LoadDifWats/InvV;
		int NewLimit=BuyCurMax+ACLoadDifAmps;
			WMVPRINTW(FNDCWin,5,2,"NewLimit %4d    ",NewLimit);
			if (NewLimit<5) { 
				if ((NewLimit<(-5))&&(FNDC_BATT_VOLTS>51.2)){
					whsSetFlags(whsAuto,whsOn);
					pause=3;
				}
				else if (NewLimit<0){DROP_GRID(5,((DischargeAllowed==YES)?20:5))}
				NewLimit=5; 
			}
			if (NewLimit>50) { NewLimit=50; }
			if (AC1InLimit!=NewLimit) {
				if (NewLimit>7){
					whsSetFlags(whsAuto,whsOff);	
					pause=3;
				}
				if ((INVERTERVOLTS < sellv) || (netbattamps > BatTrgAmp) || (NewLimit > AC1InLimit)){
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
static int weight=0, trigger=20000;
	if (AC1InLimit!=45){ cmdMate("AC1INLM","45",__LINE__); AC1InLimit=45; }
	WMVPRINTW(FNDCWin,5,2,"%s                    ","");
	TargetGPUse=500+(SOD*SOD*SOD);
	weight+=((ngp>0) ? (ngp-TargetGPUse) : ((ngp*2)-TargetGPUse)*2);
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
				if (!UnderUtilization){
					whsSetFlags(whsAuto,whsOff);
				}
			}
		}
		else //weight < 0 - Load is too low.  Increase load. 
		{
			if (((whsSetFlags(whsAuto,whsInquiry)!=whsOn)) && (TargetGPUse<1000)){
				whsSetFlags(whsAuto,whsOn);
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
		if (((weight<0) && (digitalRead(WH_LOWER_ELEMENT)==1) && sellv>= SellVoltMax ) ||
			((weight>0) && ((digitalRead(WH_LOWER_ELEMENT)==0) && sellv<= SellVoltMin ) ))
		{
			weight=0;
		}	
	}
	WMVPRINTW(FNDCWin,6,2,"T %5d W %7d",TargetGPUse,weight);
	if ((DropSelected) && ((FNDC_SOC)>(MIN_SOC_DROPPED))){
	static int tmr=0;
		if (((MaxNegBatAmpsDropped)*INVEFF) < (netbattamps-((ngp*10)/invbattv))){
			if (tmr++ > 30){
				cmdMate("AC","0",__LINE__);
				tmr=0;
			}
		}
		else
		{
			if (--tmr<0) { tmr=0; }
		}
	}
}

#define SWITCH_TO_GRID(PAUSE)	if (FNDC_BATT_VOLTS<54.0){cmdMate("AC","1",__LINE__); whsSetFlags(whsAuto,whsOff); \
										cmdMate("AC1INLM","45",__LINE__);SavedSOC=0; pause=PAUSE;BelowSellvSecs=0;}
#define BELOWSELLVSECSTRIG	(-300)
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
		if (whsSetFlags(whsAuto,whsInquiry)==whsOn){
			BelowSellvSecs=0;
			whsSetFlags(whsAuto,whsOff);
		}else{
			SWITCH_TO_GRID(15);
		}
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
				whsSetFlags(whsAuto,whsOff);
				SWITCH_TO_GRID(12)
				return;
			} 
		}else if (lt45>0) lt45--;
	}
#define WHSPREAD 		10
#define WHMULTIPLIER 	3
	if ((!UnderUtilization) && (whsSetFlags(whsAuto,whsInquiry)==whsOn)	&& (FNDC_BATT_VOLTS<55.0)
							&& (netbattamps < ((MIN(((100-WHSPREAD)-SOC),0)*WHMULTIPLIER)))){
		whsSetFlags(whsAuto,whsOff);
		BelowSellvSecs=0;
		pause=5;
		return;
	}
	else 
	{		
		if ((whsSetFlags(whsAuto,whsInquiry)!=whsOn) &&
						(((netbattamps > ((MIN((100-SOC),WHSPREAD)*WHMULTIPLIER))) && (FNDC_BATT_VOLTS>51.2)) ||
						(FNDC_BATT_VOLTS>56.8))){
			whsSetFlags(whsAuto,whsOn);
			pause=5;
			return;
		}
	}
	{static int ltNegAmps=0;	
		if (SOC<(MIN_SOC_DROPPED)){
			if ((whsSetFlags(whsAuto,whsInquiry)==whsOn) && (!UnderUtilization)){
				whsSetFlags(whsAuto,whsOff);
				pause=5;
			}
			SWITCH_TO_GRID(12) 
		}
		else
		{
			if (netbattamps<(MaxNegBatAmpsDropped)){
				if (++ltNegAmps>=20){
					ltNegAmps=0;
					SWITCH_TO_GRID(12)
					} 
		}else if (ltNegAmps>0) ltNegAmps--;
		}
	}
}

void think(void){
static int pause=0, ContinousNegAmpsSec=0, DetectChrgEnabledSecs=0;
int CC1Mode=CC1_MODE, CC2Mode=CC2_MODE;
	if (initilized!=YES) init();
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
			if (((CC1_PVIV>50) || (CC2_PVIV>50)) && (FNDC_BATT_VOLTS>ChargeDisableTrigV))
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
	invbattv=(int)(INVERTERVOLTS*10);
	netbattamps=((FNDC_SHUNT_A_AMPS)+(FNDC_SHUNT_B_AMPS)+(FNDC_SHUNT_C_AMPS));
	if (netbattamps	> 0.0){
		if ((FNDC_BATT_VOLTS <= 51.2) || (netbattamps> ((FNDC_BATT_VOLTS-51.0)*1.6)) || (BATT_STATUS_AH<(-0.05)))
		{ 
			BatAmpHrIn+=(netbattamps/(60.0*60.0));
		}
		if (netbattamps	> 1.0)ContinousNegAmpsSec=0; else if (ContinousNegAmpsSec>1)ContinousNegAmpsSec=1;
	}else{
		BatAmpHrOut-=(netbattamps/(60.0*60.0));
		if ((ContinousNegAmpsSec>0) || (netbattamps<(-1.0))){ 
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
				wprintw(ScrollWin,"SMS_SendMsg returned %d. Msg: %s\n",SMS_SendMsg("2814509680",strtmp),strtmp);
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
			if (FNDC_BATT_VOLTS>50) whsSetFlags(whsConserve,whsOff);
			DroppedGrid();
			DroppedSecs++;
			break;
		case 2: //Using grid
			if (FNDC_BATT_VOLTS>50) whsSetFlags(whsConserve,whsOff);
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
					if ((FNDC_BATT_VOLTS*10)>(sellv+10)) cmdMate("AC","0",__LINE__);
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

void compressor(int SwOn){
	if(SwOn)
		cmdMate("AUXON","1",__LINE__);
	else
		cmdMate("AUXOFF","1",__LINE__);
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

void ProcessUserInput(void){
		while((ch = getch()) != ERR) {
			if (ch=='^') { 	//unlock the keyboard
				KBLock=0; 
			}
			else 
			{
				if (KBLock==1) { return; }
			}
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
			case 'C':
				ChargeDisableTrigV+=1.0;
				if (InvChrgEnabled!=YES){
					whsSetFlags(whsAuto,whsOff);
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
					InvChrgEnabled=NO;
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
				WHtopMinTemp--;
				break;
			case 'F':
				WHtopMinTemp++;
				break;
			case 'g':
				WHCenterMinTemp--;
				break;
			case 'G':
				WHCenterMinTemp++;
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
				AllowDaytimeInvCharge=FALSE;
				break;
			case 'j':
				WHpowerLevel(0);
				break;
			case 'J':
				WHpowerLevel(1);
				break;
			case 'K':
				whsSetFlags(whsDisableTimer,whsOn);
				break;
			case 'k':
				whsSetFlags(whsDisableTimer,whsUnset);
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
				SMS_doSendStatRpt=YES;
				SEND_SMS_STAT_RPT
				break;
			case 'o':  // unlock inv aux lock
				whsSetFlags(whsManual,whsUnset);
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
				whsSetFlags(whsTimer,whsOn);
				break;
			case 't':
				whsSetFlags(whsTimer,whsUnset);
				break;
			case 'U': // use grid now
				cmdMate("AC","1",__LINE__);
			  case 'u': // set use grid flag
				DropSelected=0;
				break;
			case 'V':	//Raise the minimum sellv
				if ((SellVoltMin+4)<=SellVoltMax) {SellVoltMin+=4;}
				break;
			case 'v':	//Lower the minimum sellv
				SellVoltMin-=4;
				break;
			case 'W': // turn on water heater
				whsSetFlags(whsManual,whsOn);
				break;
			case 'w': // turn off water heater
				whsSetFlags(whsManual,whsOff);
				break;
			case 'Y':	//Allow severe battery drain.  Use with caution!!!
				InvIgnLowVolt=true;
				break;
			case 'y':	//Do not allow severe battery drain.  
				InvIgnLowVolt=false;
				break;
			case 'Z': // Discharge Not Allowed
				DischargeAllowed=NO;MaxVNegAmps=0.0;
				break;
			case 'z': // Discharge Is Allowed
				DischargeAllowed=YES;MaxVNegAmps=0.0;
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
				InvInputMode=Support;
				break;
			case '2':
				InvInputMode=GridTied;
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
	WMVPRINTW(InvWin,7,2,"NetGP %5d %s Watts",ngp,(ngp>0) ? "$$$":"   ");
	WMVPRINTW(InvWin,8,2,"Aux  %2d%2d%2d %2d",(digitalRead(WH_UPPER_ELEMENT)), (digitalRead(WH_LOWER_ELEMENT_HV))
													,(digitalRead(WH_LOWER_ELEMENT)!=0),(INVERTER_AUX_OUT!=0));
	WMVPRINTW(InvWin,9,1,"InvPwr %5d %6.3f ", InverterPwr, InvWH/1000);
	WMVPRINTW(InvWin,10,1,"SelV %3.1f >=%3.1f<=%3.1f",(float)sellv / 10.0,(float) SellVoltMin/10.0,(float) SellVoltMax/10.0);
	WMVPRINTW(InvWin,11,1,"Mode %d %8s",InvInputMode,InvInputModeLabels[InvInputMode]);
	WMVPRINTW(InvWin,12,1,"room temp %4.1f      ",/*sensor[3].tempF*/readSensorF(3,-666.9));
	WMVPRINTW(InvWin,13,1,"atic temp %4.1f      ",/*sensor[5].tempF*/readSensorF(5,-666.9));
	WMVPRINTW(CCWin,0,2,"BattV  %4.1f %4.1f",CC1_BATT_VOLTS,CC2_BATT_VOLTS);
	WMVPRINTW(CCWin,1,2,"Prg %1d %s",ProgramIndx,Programs[ProgramIndx]);
	WMVPRINTW(CCWin,2,2,"KWH    %4.1f %4.1f %4.1f",CC1_KWHH,CC2_KWHH,CC1_KWHH+CC2_KWHH);
	WMVPRINTW(CCWin,3,2,"AmpHrs %4d %4d %4d",CC1_AHA,CC2_AHA,CC1_AHA+CC2_AHA);
	WMVPRINTW(CCWin,4,2,"MODE  %5s %5s",CCModes[CC1_MODE],CCModes[CC2_MODE]);
	WMVPRINTW(CCWin,5,2,"PVIV   %4d %4d",CC1_PVIV,CC2_PVIV);
	WMVPRINTW(CCWin,6,2,"OUTAMP %4.1f %4.1f",CC1_AMPS,CC2_AMPS);
	WMVPRINTW(CCWin,7,2,"Watts  %4.0f %4.0f %5.0f",CC1_AMPS*CC1_BATT_VOLTS,CC2_AMPS*CC2_BATT_VOLTS,\
									CC1_AMPS*CC1_BATT_VOLTS+CC2_AMPS*CC2_BATT_VOLTS);
	WMVPRINTW(CCWin,8,2,"%Rating %5.1f%%",	(CC1_AMPS*CC1_BATT_VOLTS+CC2_AMPS*CC2_BATT_VOLTS)/(ArrayRatedWatts/100));

	WMVPRINTW(CCWin,9,2,"M%1d A%1d T%1d DT%1d C%1d S%1d TE%d",whsSetFlags(whsManual,whsInquiry),whsSetFlags(whsAuto,whsInquiry),
			whsSetFlags(whsTimer,whsInquiry),whsSetFlags(whsDisableTimer,whsInquiry),
			whsSetFlags(whsConserve,whsInquiry), whGetDesiredState(/*digitalRead(WH_LOWER_ELEMENT),INVERTER_AUX_OUT,*/
			(L1_INVERTER_AMPS+L1_CHARGER_AMPS+L1_BUY_AMPS-L1_SELL_AMPS),
			(L2_INVERTER_AMPS+L2_CHARGER_AMPS+L2_BUY_AMPS-L2_SELL_AMPS)),digitalRead(WH_UPPER_ELEMENT));
	WMVPRINTW(CCWin,10,2,"%02d:%02d:%02d  %02d:%02d:%02d",hour,minute,second,ResetTime_p.tm_hour,ResetTime_p.tm_min
															,ResetTime_p,ResetTime_p.tm_sec);
	WMVPRINTW(CCWin,11,2,"   %5.1f        >%3.0f",readSensorF(2,666.6),WHtopMinTemp);									
	WMVPRINTW(CCWin,12,2,"%5.1f %5.1f  %s >%3.0f", readSensorF(4,666.6),readSensorF(1,666.6)
									,(digitalRead(WH_UPPER_ELEMENT) ? "^ " : "  "), WHCenterMinTemp);
	WMVPRINTW(CCWin,13,2,"   %5.1f  %3s%s      ",readSensorF(0,666.6)
											,((digitalRead(WH_LOWER_ELEMENT_HV) && digitalRead(WH_LOWER_ELEMENT)) ? "^^^" : "  ")
											,(digitalRead(WH_LOWER_ELEMENT) ? "^ " : "  "));
//	WMVPRINTW(CCWin,13,1,"%5.1f %5.1f %5.1f %5.1f",sensor[0].tempF,sensor[1].tempF,sensor[2].tempF,sensor[4].tempF);
	WMVPRINTW(FNDCWin,0,2,"BattV %3.1f ",FNDC_BATT_VOLTS);
	WMVPRINTW(FNDCWin,1,2,"SOC   %3d%% ",FNDC_SOC);
	WMVPRINTW(FNDCWin,2,2,"Amps %5.1f %5.1f %5.1f  ",FNDC_SHUNT_A_AMPS,FNDC_SHUNT_B_AMPS,FNDC_SHUNT_C_AMPS);
	WMVPRINTW(FNDCWin,3,2,"Net Amps%6.1f             ",netbattamps);
	WMVPRINTW(FNDCWin,4,2,"InvOut/In %6.1f%% ",(INVPWR/((0-FNDC_SHUNT_A_AMPS)*FNDC_BATT_VOLTS))*100);
	if (InvInputMode != GridTied) WMVPRINTW(FNDCWin,6,2,"ACPS %4s S %d P %d",acpsModeDesc[AirCondPwrSrc],
						digitalRead(MRCOOL2KHP_SRC_GPIO),digitalRead(MRCOOL2KHP_PWR_GPIO));
	WMVPRINTW(FNDCWin,7,1,"MaxNegA Dropped %5.1f ",MaxNegBatAmpsDropped);
	WMVPRINTW(FNDCWin,8,2,"KB Locked %s ",((KBLock) ? "Yes Press ^":"No Press L "));
	WMVPRINTW(FNDCWin,10,2,"%s         ","  ");
	WMVPRINTW(FNDCWin,3,18,"%s","              ");
	if (InvInputMode==GridTied){
		WMVPRINTW(FNDCWin,12,2,"%s         ",((DropSelected) ? "DS":"--"))
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

int main( int argc, char *argv[]) 
{ 
	terminate=false;
	if (signal(SIGINT,signalIntHandler)== SIG_ERR) return 5;
	IAO_Day_Secs=0;
	IAR_Day_Secs=0;
	SellWH=0;
	invMode=-1;
	wiringPiSetupGpio () ;
	pinMode(27,OUTPUT);
	pinMode(AIR_COND_GPIO_PIN,OUTPUT);
	pinMode(WH_LOWER_ELEMENT,OUTPUT);
	pinMode(WH_LOWER_ELEMENT_HV,OUTPUT);
	pinMode(WH_UPPER_ELEMENT,OUTPUT);
	pinMode(MRCOOL2KHP_PWR_GPIO,OUTPUT);
	pinMode(MRCOOL2KHP_SRC_GPIO,OUTPUT);
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
	echo();
	attroff(COLOR_PAIR(8));
	endwin();
	exit(0);
}

