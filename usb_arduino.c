//usb_com
#define __usb_com_c__

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include "usb_arduino.h"
#include "outback.h"
#include <ncurses.h>

	/* baudrate settings are defined in <asm/termbits.h>, which is
	   included by <termios.h> */
#define BAUDRATE B9600
	/* change this definition for the correct port */
#define MODEMDEVICE "/dev/ttyNano"
#define _POSIX_SOURCE 1		/* POSIX compliant source */

#define FALSE 0
#define TRUE 1
#define STALE_TICKS	120

extern unsigned long Ticks;
extern WINDOW *InvWin, *CCWin, *FNDCWin, *ScrollWin;

struct address64
{
  long highval;
  long lowval;
};

#define MAX_C_TEMP	120
#define MIN_C_TEMP	-20
#define	MAX_F_TEMP	250
#define MIN_F_TEMP	0


struct address64 sensorAddress[20] = { {0x28680315, 0x600005A},
{0x28337115, 0x6000062},
{0x28760C15, 0x600007D},
{0x28FAD9CD, 0x4000034},
{0x28F21415, 0x6000009},
{0x281B1C15, 0x6000088},
{0x28BF4615, 0x6000020},
{0x28F60215, 0x6000035},
									/*      {0x28A86515,0x6000048}, */
{0, 0}
};

struct tempSensorValue sensor[20];

float
readSensorF (int SensorIndx, float ErrorValue)
{
  if ((Ticks - sensor[SensorIndx].lastUpdate) > STALE_TICKS)
    return ErrorValue;
  return sensor[SensorIndx].tempF;
}

int
getAddressIndx (long address1, long address2)
{
  int i = 0;
  for (i = 0; i < 20; i++)
    {
      if (sensorAddress[i].highval == 0)
	{
	  wprintw (ScrollWin, "%lx %lx\n", address1, address2);
	  return -1;
	}
      if ((sensorAddress[i].highval == address1)
	  && (sensorAddress[i].lowval == address2))
	return i;
    }
  return -1;
}

//const int TypeFieldCount[8]={0,0,0,15,13,0,23,0};
//int data[8][32];
char USBNanolastResponse[256];
//int InverterIndx=0,CC1Indx=0,CC2Indx=0,FNDCIndx=0;

int usbNano;
struct termios Nano_oldtio;

static int ProcessBuff (char *line);

void
DebugNanoPrintData (void)
{
}

void
USBNano_init ()
{
  struct termios newtio;
  //printf("%s\n","at init");
//          memset(data,0,sizeof data);
  {
    int i;
    for (i = 0; i < 20; i++)
      {
	sensor[i].tempF = 555;
	sensor[i].tempC = 555;
      }
  }
  memset (USBNanolastResponse, 0, sizeof USBNanolastResponse);
  /* 
     Open modem device for reading and writing and not as controlling tty
     because we don't want to get killed if linenoise sends CTRL-C.
   */
  usbNano = open (MODEMDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);	//printf("%d\n",__LINE__);
  if (usbNano < 0)
    {
      perror (MODEMDEVICE);
      exit (-1);
    }

  tcgetattr (usbNano, &Nano_oldtio);	/* save current serial port settings */
  bzero (&newtio, sizeof (newtio));	/* clear struct for new port settings */

  /* 
     BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
     CRTSCTS : output hardware flow control (only used if the cable has
     all necessary lines. See sect. 7 of Serial-HOWTO)
     CS8     : 8n1 (8bit,no parity,1 stopbit)
     CLOCAL  : local connection, no modem contol
     CREAD   : enable receiving characters
   */
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

  /*
     IGNPAR  : ignore bytes with parity errors
     ICRNL   : map CR to NL (otherwise a CR input on the other computer
     will not terminate input)
     otherwise make device raw (no other input processing)
   */
  newtio.c_iflag = IGNPAR | ICRNL;

  /*
     Raw output.
   */
  newtio.c_oflag = 0;

  /*
     ICANON  : enable canonical input
     disable all echo functionality, and don't send signals to calling program
   */
  newtio.c_lflag = ICANON;

  /* 
     initialize all control characters 
     default values can be found in /usr/include/termios.h, and are given
     in the comments, but we don't need them here
   */
  newtio.c_cc[VINTR] = 0;	/* Ctrl-c */
  newtio.c_cc[VQUIT] = 0;	/* Ctrl-\ */
  newtio.c_cc[VERASE] = 0;	/* del */
  newtio.c_cc[VKILL] = 0;	/* @ */
  newtio.c_cc[VEOF] = 4;	/* Ctrl-d */
  newtio.c_cc[VTIME] = 0;	/* inter-character timer unused */
  newtio.c_cc[VMIN] = 1;	/* blocking read until 1 character arrives */
  newtio.c_cc[VSWTC] = 0;	/* '\0' */
  newtio.c_cc[VSTART] = 0;	/* Ctrl-q */
  newtio.c_cc[VSTOP] = 0;	/* Ctrl-s */
  newtio.c_cc[VSUSP] = 0;	/* Ctrl-z */
  newtio.c_cc[VEOL] = 0;	/* '\0' */
  newtio.c_cc[VREPRINT] = 0;	/* Ctrl-r */
  newtio.c_cc[VDISCARD] = 0;	/* Ctrl-u */
  newtio.c_cc[VWERASE] = 0;	/* Ctrl-w */
  newtio.c_cc[VLNEXT] = 0;	/* Ctrl-v */
  newtio.c_cc[VEOL2] = 0;	/* '\0' */
  //printf("%d\n",__LINE__);
  /* 
     now clean the modem line and activate the settings for the port
   */
  tcflush (usbNano, TCIFLUSH);	//printf("%d\n",__LINE__);
  tcsetattr (usbNano, TCSANOW, &newtio);	//printf("%d\n",__LINE__);
}

void
USBNano_Close (void)
{
  /* restore the old port settings */
  tcsetattr (usbNano, TCSANOW, &Nano_oldtio);
  return;
}

int
USBNano_ReadLine (void)
{
  char buf[512];
  int res;
  res = read (usbNano, buf, 500);
  if (res == -1)
    return res;
  buf[res] = 0;			/* set end of string, so we can printf */
//      printf("%s\n", buf);
  return ProcessBuff (buf);
}

/*
static int ProcessCommandEcho(char * line, int maxChar)
{
	int count=0;
	//return difference in chars to the closing delimiter -- '>'
	while (line[count]!='>' && line[count]!=0 && count<=maxChar) 
	{
		line++;
		count++;
	}
	return count;
}
*/
/*
 * static int ProcessResponse(char * line, int maxChar)
{
	int count=0;
	line++;
	maxChar = (maxChar>255) ? 255 : maxChar; 
	//return difference in chars to the closing bracket -- ']'
	while (line[count]!=']' && line[count]!=0 && count<=maxChar) 
	{
		USBNanolastResponse[count]=line[count];
		count++;
	}
	USBNanolastResponse[count]=0;
	return count;
}
*/
long
power (long base, int exp)
{
  long results = 1l;
  while (exp)
    {
      results *= base;
      exp--;
    }
  return results;
}

static int
ProcessBuff (char *line)
{
//      int lineLen=strlen(line);
  int lcursor = 0 /*, dcursor=0 */ ;
//      int checksum=0,fldCkSum=0;
  static int indx = -1;
  long address1 = 0, address2 = 0;
  char *pEnd;
  long tval;
  const long bytesize = 256;
  if (*line == 0)
    return 1;
  if (strncmp (line, "ROM = ", 6) == 0)
    {
      long pow = 7;
      lcursor = 6;
      pEnd = line + lcursor;
//              printf("%s\n",pEnd);
      for (pow = 3; pow >= 0; pow--)
	{
	  tval = strtol (pEnd, &pEnd, 16);
	  address1 += tval * power (bytesize, pow);
	  //printf("%ld %ld %s %ld\n",tval,address1,pEnd,power(bytesize,pow));
	}
      for (pow = 3; pow >= 0; pow--)
	{
	  address2 += strtol (pEnd, &pEnd, 16) * power (bytesize, pow);
	  //printf("%ld %s %ld\n",address2,pEnd,power(bytesize,pow));
	}
      indx = getAddressIndx (address1, address2);
      //printf("%lld\n",address);
    }
  else if (strncmp (line, "  Temperature = ", 16) == 0)
    {
      float t;
      if ((indx > 19) || (indx < 0))
	{
//                      printf("indx=%d\n",indx);
	  return 1;
	}
      pEnd = line + 16;
//              printf("indx=%d pEnd->%s\n",indx,pEnd);
      t = strtof (pEnd, &pEnd);
      if (t < MAX_C_TEMP && t > MIN_C_TEMP)
	sensor[indx].tempC = t;
//              printf("tempC=%f\n",strtof(pEnd,&pEnd));
      pEnd += 10;
//              printf("tempC=%f pEnd point to ->%s\n",sensor[indx].tempC,pEnd);
      t = strtof (pEnd, &pEnd);
      if (t < MAX_F_TEMP && t > MIN_F_TEMP)
	{
	  sensor[indx].tempF = t;
	  sensor[indx].lastUpdate = Ticks;
	}
//              printf("tempF=%f\n",strtof(pEnd,&pEnd));
//              printf("tempF=%f pEnd point to ->%s\n",sensor[indx].tempF,pEnd);
    }
  else
    {
//              printf("%s\n",line);
    }
  if (strncmp (line, "No more addresses.", 18) == 0)
    return 0;
  return 1;
/*	FIELD=0;
	for (lcursor=0;((lcursor<lineLen) && (line[lcursor]!=0) && (dcursor<32));lcursor++)
	{
		int c=line[lcursor];
		if (c=='[') 
		{
			lcursor+=ProcessResponse(line+lcursor, lineLen-lcursor);
			if (lcursor>lineLen) return;
		}
		else if (c=='<') 
		{
			lcursor+=ProcessCommandEcho(line+lcursor, lineLen-lcursor);
			if (lcursor>lineLen) return;
		}
		else
		{
			if (c!=',')
			{
//				printf("%c ",c);
				if ((c>'9') || (c<'0')) break;
				fldCkSum+=(c-'0');
				if (FIELD!=0)
				{
					FIELD*=10;
				}
				FIELD+=(c-'0');
			}
			else // c=','
			{
				checksum+=fldCkSum;
				dcursor++;
				rowDATA[dcursor]=0;
				fldCkSum=0;
			}
		}
	}*/
}

/*
int main(){
	int retval=1;
	int i;
	USBNano_init();
	while(retval){
		retval=USBNano_ReadLine();
//		if(retval== -1)printf("%s","+");
	}
//	printf("\n");
	for(i=0;i<5;i++){
//		if(sensorAddress[i].highval==0l) break;
		printf("%5.1f C  %5.1f F address %12ld %12ld\n",sensor[i].tempC, sensor[i].tempF,sensorAddress[i].highval,sensorAddress[i].lowval);
	}
	USBNano_Close();
	return 0;
}*/
