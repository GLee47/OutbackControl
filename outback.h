//outback.h
#ifndef __outback_h__
	#define __outback_h__

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define	NumPacketsLoop	60
#define UTIL_RATE	0.126
#define INVERTER_V			((float)(metric[1][20]/10.0))
#define MISC_BIT			((char)(metric[1][21]))
#define INV_AUX_OUT			CHECK_BIT(MISC_BIT,4)
#define INV_AUX_RELAY		CHECK_BIT(MISC_BIT,5)
#define CHAN2_CC_V			((float)(metric[2][12]/10.0))
#define CHAN3_CC_V			((float)(metric[3][12]/10.0))
#define FLEXNETDC_V			((float)(metric[4][8]/10.0))
#define FLEXNETDC_TEMP		((((metric[4][12])-10.0)*(9.0/5.0))+32.0)
#define AVG_DEV_V			((INVERTER_V+CHAN2_CC_V+CHAN3_CC_V+FLEXNETDC_V)/4)
#define DISCONNECT_VOLT		52.0
#define CONNECT_VOLT		48.8
#define DISC_DELAY_TICKS	330
#define CONN_DELAY_TICKS	330
#define L1_AAC_BUY			metric[1][5]
#define L2_AAC_BUY			metric[1][12]
#define LOOP_V				(FLEXNETDC_V)
#define TIMESTAMP			"%02d%02d%02d%02d",ts->tm_mon+1,ts->tm_mday,ts->tm_hour,ts->tm_min
#define TIMESTAMPSECS		"%02d%02d%02d%02d%02d",ts->tm_mon+1,ts->tm_mday,ts->tm_hour,ts->tm_min,ts->tm_sec
#define RADIAN_AC_MODE		((int)(metric[1][19]))
//#define BUF_INT(pos,len)	{buf[pos+len]=0; atoi(buf[pos];}					

//#define DATA_FILE_NAME	"/mnt/sda1/home/glee/c/matedata2014_12_11.csv"
//#define MATE3_USB "/dev/ttyMate3" 
#define DEFAULT_SELL_V_MIN 512
#define DEFAULT_SELL_V_MAX 556
//#define GP_MAX	1250
//#define GP_MIN	0
#define SUPPORT_MODE_MIN_ADJ_SELL_VOLTS 448
#define SUPPORT_MODE_MAX_ADJ_SELL_VOLTS 560
#define MIN_SOC_DROPPED	55
#define MIN_VOLTS_DROPPED 44.8
#define MAX_NEG_NBA_DROPPED	20
#define WH_LOWER_ELEMENT	26  /* BCM pin# */ 
#define WH_LOWER_ELEMENT_HV	12  /* BCM pin# */ 
#define WH_UPPER_ELEMENT	16  /* BCM pin# */ 
#define AIR_COND_GPIO_PIN	5	/* frount window unit on/off */
#define MRCOOL2KHP_PWR_GPIO	6	/*Mr Cool great room pwr high on/low off */
#define	MRCOOL2KHP_SRC_GPIO	13	/*Mr Cool great room power sorce - high is inverter, low is grid */

//#define WATERHEATEROFF	digitalRead(WH_LOWER_ELEMENT)
//#define WATERHEATERON	if (INVERTER_AUX_OUT==0){cmdMate("AUXON","1");}
#define WaterHeaterKWH ((((float)IAO_Day_Secs)/3600.0)*1.08)+(((((float)IAR_Day_Secs)/3600.0))*4.32)
#define MAX(ARG1,ARG2)		(((ARG1)>(ARG2)) ? (ARG1) : (ARG2))
#define MIN(ARG1,ARG2)		(((ARG1)<(ARG2)) ? (ARG1) : (ARG2))
//#define INT_ROUND(NUMBR,RESOLUTION)	NUMBR=(((int)((NUMBR+(RESOLUTION/2))/(float)RESOLUTION))*RESOLUTION)
#define INVERTER_START	14
#define CC1_START		90
#define CC2_START		138
#define INVERT_INDX		1
#define CC1_INDX		2
#define CC2_INDX		3
#define FNDC_INDX		4
#define FLEXNETDC_Start	186
#define CC1_BATT_VOLTS		(((float) (data[CC1_INDX][11])/10.0))//  BufInt(CC1_START+37,3)/10)
#define CC2_BATT_VOLTS		(((float) (data[CC2_INDX][11])/10.0))//  BufInt(CC2_START+37,3)/10)
#define FNDC_BATT_VOLTS		(((float) (data[FNDC_INDX][7])/10.0))//BufInt(FLEXNETDC_Start+31,3)/10)
#define fSellV				(float)(sellv/10.0)
#define FNDC_STATUS_FLAGS	((char)(data[FNDC_INDX][10]))	//BufInt(FLEXNETDC_Start+43,2))
#define FNDC_SHUNT_A_NEG	CHECK_BIT(FNDC_STATUS_FLAGS,3)
#define FNDC_SHUNT_B_NEG	CHECK_BIT(FNDC_STATUS_FLAGS,4)
#define FNDC_SHUNT_C_NEG	CHECK_BIT(FNDC_STATUS_FLAGS,5)
#define FNDC_SHUNT_A_AMPS	((((float)/*BufInt(FLEXNETDC_Start+ 7,4)*/ (data[FNDC_INDX][2]))/10.0)*((FNDC_SHUNT_A_NEG==0) ? 1 : (-1)))
#define FNDC_SHUNT_B_AMPS	((((float)/*BufInt(FLEXNETDC_Start+12,4)*/ (data[FNDC_INDX][3]))/10.0)*((FNDC_SHUNT_B_NEG==0) ? 1 : (-1)))
#define FNDC_SHUNT_C_AMPS	((((float)/*BufInt(FLEXNETDC_Start+17,4)*/ (data[FNDC_INDX][4]))/10.0)*((FNDC_SHUNT_C_NEG==0) ? 1 : (-1)))
#define FNDC_BATT_TEMP		((((float)(data[FNDC_INDX][11])-10.0)*1.8)+32)
#define CC1_MODE			(data[CC1_INDX][10])	//BufInt(CC1_START+34,2)
#define CC2_MODE			(data[CC2_INDX][10])	//BufInt(CC2_START+34,2)
#define INVERTER_AC_MODE	(data[INVERT_INDX][18])	//BufInt(INVERTER_START+62,2)
#define INVERTER_OP_MODE	(data[INVERT_INDX][16])	//BufInt(INVERTER_START+55,2)
#define L1_INVERTER_AMPS	(data[INVERT_INDX][2])	//BufInt(INVERTER_START+7,2)
#define L2_INVERTER_AMPS	(data[INVERT_INDX][9])	//BufInt(INVERTER_START+31,2)
#define L1_INVERTER_VOUT	(data[INVERT_INDX][8])	//BufInt(INVERTER_START+27,3)
#define L2_INVERTER_VOUT	(data[INVERT_INDX][15])	//BufInt(INVERTER_START+51,3)
#define L1_CHARGER_AMPS		(data[INVERT_INDX][3])	//BufInt(INVERTER_START+10,2)
#define L2_CHARGER_AMPS		(data[INVERT_INDX][10])	//BufInt(INVERTER_START+34,2)
#define L1_BUY_AMPS			(data[INVERT_INDX][4])	//BufInt(INVERTER_START+13,2)
#define L2_BUY_AMPS			(data[INVERT_INDX][11])	//BufInt(INVERTER_START+37,2)
#define L1_SELL_AMPS		(data[INVERT_INDX][5])	//BufInt(INVERTER_START+16,2)
#define L2_SELL_AMPS		(data[INVERT_INDX][12])	//BufInt(INVERTER_START+40,2)
#define TOT_LOAD(LEG)		(((LEG##_BUY_AMPS) + (LEG##_INVERTER_AMPS))) /*- LEG##_CHARGER_AMPS)*/
#define INVERTERVOLTS		(((float)(data[INVERT_INDX][19]))/10.0)	//((float)BufInt(INVERTER_START+65,3)/10)
#define CC1_AHA				(data[CC1_INDX][12])	//BufInt(CC1_START+41,4)
#define CC1_KWHH			(((float)(data[CC1_INDX][6]))/10.0)	//((float)BufInt(CC1_START+20,4)/10)
#define CC1_PVIV			(data[CC1_INDX][5])	//BufInt(CC1_START+16,3)
#define CC1_AMPS			(((float)data[CC1_INDX][3])+(((float)data[CC1_INDX][7])/10.0))	//(((float)BufInt(CC1_START+10,2))+(((float)BufInt(CC1_START+25,1))/10))
#define CC2_AHA				(data[CC2_INDX][12])	//BufInt(CC2_START+41,4)
#define CC2_KWHH			(((float)(data[CC2_INDX][6]))/10.0)	//((float)BufInt(CC2_START+20,4)/10)
#define CC2_PVIV			(data[CC2_INDX][5])	//BufInt(CC2_START+16,3)
#define CC2_AMPS			(((float)data[CC2_INDX][3])+(((float)data[CC2_INDX][7])/10.0))	//(((float)BufInt(CC2_START+10,2))+(((float)BufInt(CC2_START+25,1))/10))
#define FNDC_SOC			(data[FNDC_INDX][8]) //BufInt(FLEXNETDC_Start+35,3)
#define WMVPRINTW(WinPtr,X,Y,Format,...) wmove(WinPtr,X,Y); wprintw(WinPtr,Format,__VA_ARGS__);
#define INV_MISC_BIT 		((char)(data[INVERT_INDX][20]))	//BufInt(INVERTER_START+69,3))
#define INVERTER_AUX_OUT	CHECK_BIT(INV_MISC_BIT,4)
#define INVERTER_AUX_RELAY	CHECK_BIT(INV_MISC_BIT,5)
#define INVPWR				((L1_INVERTER_AMPS)*(L1_INVERTER_VOUT)+(L2_INVERTER_AMPS)*(L2_INVERTER_VOUT))
#define WH_HALF_DC_AMPS		13
#define INVEFF				0.92
#define PUMPMAXAMPS			9
#define BATT_STATUS_AH_RAW	(((BatAmpHrIn)-BatAmpHrOut)-BatAmpHrCrx)
#define BATT_STATUS_AH		((BatAmpHrIn*BattEffFac)-BatAmpHrOut)
#define IOM_OFFSET			14
#define IOM_CHRG			3
#define INV_IS_OFF			0
		
enum InverterACModes{iacmNoGr=0,iacmDrop=1,iacmUse=2};
enum AirCondPwrSrcModes{acpsGrid,acpsInverter,acpsNone};

	#ifndef __MateMonitor_c__
		extern float WHtopMaxTemp,WHtopMinTemp,WHCenterMinTemp,WHmaxAnyTemp;	
		extern int UnderUtilization, sellv;
		extern float netbattamps;
	#endif

#endif

#ifndef YES
#define YES	1
#define	NO	0
#endif

#ifndef __usb_com_h__
#include "usb_com.h"
#endif
