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
//#include "usb_com.h"
void DebugPrintData (void);
void USB_init ();
void USB_Close (void);
void USB_ReadLine (void);

	/* baudrate settings are defined in <asm/termbits.h>, which is
	   included by <termios.h> */
#define BAUDRATE B9600
	/* change this definition for the correct port */
#define MODEMDEVICE "/dev/ttyMate3"
#define _POSIX_SOURCE 1		/* POSIX compliant source */

#define FALSE 0
#define TRUE 1

volatile int STOP = FALSE;
const int TypeFieldCount[8] = { 0, 0, 0, 15, 13, 0, 23, 0 };

int data[8][32];
char USBlastResponse[256];
int USBlastReadlineBytes = 0;
int InverterIndx = 0, CC1Indx = 0, CC2Indx = 0, FNDCIndx = 0;

int usbmate;
struct termios oldtio;

static void ProcessBuff (char *line);
//int dataFileHandle;

void
DebugPrintData (void)
{
  int d, i;
  for (d = 0; d < 8; d++)
    {
      for (i = 0; i < 32; i++)
	{
	  printf ("%d ", data[d][i]);
	}
      printf ("\n");
    }
}

void
USB_init ()
{
  struct termios newtio;
  /*
     dataFileHandle = open("/home/pi/projects/MateMonitor",O_RDWR);
     if (dataFileHandle == -1)
     {
     perror("Open error");
     exit(EXIT_FAILURE);
     }
     mmap
   */
  memset (data, 0, sizeof data);
  memset (USBlastResponse, 0, sizeof USBlastResponse);
  /* 
     Open modem device for reading and writing and not as controlling tty
     because we don't want to get killed if linenoise sends CTRL-C.
   */
  usbmate = open (MODEMDEVICE, O_RDWR | O_NOCTTY);
  if (usbmate < 0)
    {
      perror (MODEMDEVICE);
      exit (-1);
    }

  tcgetattr (usbmate, &oldtio);	/* save current serial port settings */
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

  /* 
     now clean the modem line and activate the settings for the port
   */
  tcflush (usbmate, TCIFLUSH);
  tcsetattr (usbmate, TCSANOW, &newtio);
}

void
USB_Close (void)
{
  /* restore the old port settings */
  tcsetattr (usbmate, TCSANOW, &oldtio);
  return;
}

void
USB_ReadLine (void)
{
  char buf[512];
  int res;
  res = read (usbmate, buf, 500);
  buf[res] = 0;			/* set end of string, so we can printf */
//      printf("%s\n", buf);
  USBlastReadlineBytes = res;
  ProcessBuff (buf);
}

static int
ProcessCommandEcho (char *line, int maxChar)
{
  int count = 0;
  //return difference in chars to the closing delimiter -- '>'
  while (line[count] != '>' && line[count] != 0 && count <= maxChar)
    {
      line++;
      count++;
    }
  return count;
}

static int
ProcessResponse (char *line, int maxChar)
{
  int count = 0;
  line++;
  maxChar = (maxChar > 255) ? 255 : maxChar;
  //return difference in chars to the closing bracket -- ']'
  while (line[count] != ']' && line[count] != 0 && count <= maxChar)
    {
      USBlastResponse[count] = line[count];
      count++;
    }
  USBlastResponse[count] = 0;
  return count;
}

#define FIELD	rowDATA[dcursor]
static void
ProcessBuff (char *line)
{
  int lineLen = strlen (line);
  int lcursor = 0, dcursor = 0;
  int checksum = 0, fldCkSum = 0;
  int rowDATA[32];

  if (*line == 0)
    return;
  FIELD = 0;
  for (lcursor = 0;
       ((lcursor < lineLen) && (line[lcursor] != 0) && (dcursor < 32));
       lcursor++)
    {
      int c = line[lcursor];
      if (c == '[')
	{
	  lcursor += ProcessResponse (line + lcursor, lineLen - lcursor);
	  if (lcursor > lineLen)
	    return;
	}
      else if (c == '<')
	{
	  lcursor += ProcessCommandEcho (line + lcursor, lineLen - lcursor);
	  if (lcursor > lineLen)
	    return;
	}
      else
	{
	  if (c != ',')
	    {
//                              printf("%c ",c);
	      if ((c > '9') || (c < '0'))
		break;
	      fldCkSum += (c - '0');
	      if (FIELD != 0)
		{
		  FIELD *= 10;
		}
	      FIELD += (c - '0');
	    }
	  else			// c=','
	    {
	      checksum += fldCkSum;
	      dcursor++;
	      rowDATA[dcursor] = 0;
	      fldCkSum = 0;
	    }
	}
    }
  if (dcursor > 1)
    {
//              printf("%d %d dcursor=%d  checksum=%d rowDATA[dcursor]=%d\n",rowDATA[0],rowDATA[1],dcursor,checksum,rowDATA[dcursor]); 
      if (checksum == rowDATA[dcursor])
	{
	  int i = 0;
	  int l = TypeFieldCount[rowDATA[1]];
//                      printf("%d %d %d %d %d\n",i,l,rowDATA[0],rowDATA[1],rowDATA[2]);
	  for (i = 0; i < l; i++)
	    {
//                              printf("%d %d %d\n",i,rowDATA[0],rowDATA[i]);
	      data[rowDATA[0]][i] = rowDATA[i];
	    }
	}
    }
}
