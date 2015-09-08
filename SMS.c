/*
 * SMS.c
 * Send and recieve text message
 * 
 * Copyright 2015 Gary Lee <glee@Compaq>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */

#define SMSTEXTMODE	"AT+CMGF=1\r"
#define SMS_INIT_MSG "AT+CMGW="
#define SMS_SEND_MSG "AT+CMSS="
#define SMS_DELETE_MSG	"AT+CMGD="
#define SMSMSG	"123" "ABC"


#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <ctype.h>
#include <time.h>

enum SMS_Alert{
	SMS_Alert_info,
	SMS_Alert_Nominal,
	SMS_Alert_Notify,
	SMS_Alert_Warning,
	SMS_Alert_Critical,
	SMS_Alert_ActionTaken
};
 
struct SMS_Message_Properties{
	char	PhoneNum[0x10];
	char	MsgText[0x100];
	time_t	SentTime;
	int		MinSecRepeat;
};
	

struct termios options;
int fd; /* File descriptor for the port */
int PortOpened=0;

char *quotedString(char *s, int MaxChars)
{
//	printf("%s\n",s);
	int i, first=0;
	if (s[0]==0) //we have reached the end of the string before we have found closing quote
			return NULL;
	for (i=0; i<MaxChars; i++)
	{
		if (first==0) //have not found leading quote yet
		{
			if (s[i]=='"')
			{
				first=i+1;
			}
		}
		else //found leading quote -- looking for closing qoute
		{
			if(s[i]=='"')
			{
				s[i]=0;
				return s+first;
			}
		}
	}
	if (first==0)
		return NULL;
	else
		return s+first;
}

char *chop(char *s, char c)
{
	while((s[0]!=c) && (s[0]!=0))s++;
	s[0]=0;
	return s;
}

char *ltrim(char *s)
{
	while(isspace(*s)) s++;
	return s;
}

char *rtrim(char *s)
{
	char* back = s + strlen(s);
	while(isspace(*--back));
	*(back+1) = '\0';
	return s;
}

char *trim(char *s) //returns a pointer to string without leading or trailing spaces
{
	return rtrim(ltrim(s)); 
}


int isInStr(char *str, char *find) //returns 1 if find is in str, 0 if not
{
	if(str[0]==find[0])
	{
		if (find[1]==0)
			return 1;
		return (isInStr(str+1,find+1));
	}
	if(str[1]==0)
		return 0;
	return (isInStr(str+1,find));
}
	 	 
int openPortRaw(void)
{
	/* open the port */
	
	fd = open("/dev/ttySMS", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		//Could not open the port.
		return(0);
	}
	fcntl(fd, F_SETFL, 0);
	PortOpened=1;
//	printf("fd=%d\n",fd);
	fcntl(fd, F_SETFL, 0);

	/* get the current options */
	tcgetattr(fd, &options);

	/* set raw input, 1 second timeout */
	options.c_cflag     |= (CLOCAL | CREAD);
	options.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag     &= ~OPOST;
	options.c_cc[VMIN]  = 0;
	options.c_cc[VTIME] = 10;

	/* set the options */
	tcsetattr(fd, TCSANOW, &options);
	return (fd);
}

int                  /* O - 0 =  ok, -1 =  bad */
chat(char *send, char *expect, int numTries, char *replyBuff)   /* I - Serial port file */
{
char buffer[255];  /* Input buffer */
char *bufptr;      /* Current char in buffer */
int  nbytes;       /* Number of bytes read */
int  tries;        /* Number of tries so far */
int res;			//response
		
	for (tries = 0; tries < numTries; tries ++)
	{
		/* send an AT command followed by a CR */
		if ((res=write(fd, send, strlen(send))) < strlen(send))
		{
			continue;
		}
//		printf("wrote %d bytes\n",res);
		usleep(10);
		 /* read characters into our string buffer until we get a CR or NL */
		bufptr = buffer;
		while ((nbytes = read(fd, bufptr, buffer + sizeof(buffer) - bufptr - 1)) > 0)
		{
			bufptr += nbytes;
			if (bufptr[-1] == '\n' || bufptr[-1] == '\r')
				break;
		}

		 /* nul terminate the string and see if we got an OK response */
		*bufptr = '\0';
//		printf("%s %d\n",buffer,strlen(buffer));
		if (isInStr(buffer, expect))
		{
			if(replyBuff!=NULL)
				strcpy(replyBuff,buffer);
			return (0);
		}
	}

	return (-1);
}


int SMS_SendMsg(char * PhNum, char * Msg)
{
char reply[0x100], * msgNum, tBuf[0x100];
int i;
	if(!PortOpened) if(!openPortRaw()) return 0;
	if (chat("AT\r","OK",3,NULL)) return 0;			
	if (chat("AT+CMGF=1\r","OK",3,reply)) return 0;	
	sprintf(tBuf,"AT+CMGW=\"%s\"\r",PhNum);
	if (chat(tBuf,">",3,reply)) return 0;		
	sprintf(tBuf,"%s\032",Msg);
	if (chat(tBuf,"+CMGW:",3,reply)) return 0;		
	for(i=0;i<strlen(reply);i++)
	{
		if(reply[i]==':')break;
	}	
	if(i<strlen(reply))
	{
		msgNum=reply+i+1;
		msgNum[4]=0;
		sprintf(tBuf,"%s%s\r","AT+CMSS=",trim(msgNum));
		if (chat(tBuf,"OK",3,reply)) return 0;			
	}
	return 1;
}

/*
int main(void)
{*/
//	SMS_SendMsg("4092420745","Sending this to google voice.");

char *SMS_Rec_Mesg(/*char*/ struct SMS_Message_Properties *buff)
{
char reply[0x1000], mn[0x10], * msgNum, tBuf[0x100], *mType, *pNum, *dtime, *message;
int i;
	if(!PortOpened) if(!openPortRaw()) return NULL;
	if (chat("AT\r","OK",3,NULL)) return NULL;			
	if (chat("AT+CMGF=1\r","OK",3,reply)) return NULL;
	if (chat("AT+CMGL=\"REC UNREAD\"\r","+CMGL:",3,reply)) return NULL;	
//	printf("%s\n",reply);
	for(i=0;i<strlen(reply);i++)
	{
		if(reply[i]==':')break;
	}	
	if(i<strlen(reply))
	{
		msgNum=reply+i+1;
		msgNum[4]=0;
	}
	strcpy(mn,msgNum);
	mType=quotedString(msgNum+5,0x900);
	if (mType==NULL) return NULL;
//	printf("%s -%s- %d\n",mn,mType,strlen(mType));
	pNum=quotedString(mType+strlen(mType)+1,0x800);
	if (pNum==NULL) return NULL;
//	printf("-%s-\n",pNum);
	dtime=quotedString(pNum+strlen(pNum)+1,0x700);
	message=trim(dtime+strlen(dtime)+1);
	if ((pNum==NULL) || (dtime==NULL) || (message==NULL)) return NULL;
	chop(message,13);
	chop(message,10);
//	printf("t=%s p=%s t=%s m=%s\n",mType,pNum,dtime,message);
	sprintf(tBuf,"AT+CMGD=%s\r",trim(mn));
	chat(tBuf,"OK",3,reply);
//	printf("%s\n",reply);
	strcpy(buff->MsgText,message);
	strcpy(buff->PhoneNum,pNum);
	//TODO - convert time to time_t and set buff->SentTime
	return buff->MsgText;
}	

