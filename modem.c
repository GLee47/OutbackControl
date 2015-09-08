
       #include <stdio.h>   /* Standard input/output definitions */
    #include <string.h>  /* String function definitions */
    #include <unistd.h>  /* UNIX standard function definitions */
    #include <fcntl.h>   /* File control definitions */
    #include <errno.h>   /* Error number definitions */
    #include <termios.h> /* POSIX terminal control definitions */
#include <ctype.h>

    /*
     * 'open_port()' - Open serial port 1.
     *
     * Returns the file descriptor on success or -1 on error.
     */

int fd; /* File descriptor for the port */

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

char *trim(char *s)
{
    return rtrim(ltrim(s)); 
}

int open_port(void)
{

      fd = open("/dev/ttySMS", O_RDWR | O_NOCTTY | O_NDELAY);
      if (fd == -1)
      {
       /*
        * Could not open the port.
        */

        perror("open_port: Unable to open /dev/ttyf1 - ");
      }
      else
        fcntl(fd, F_SETFL, 0);

      return (fd);
}

int instr(char *str, char *find)
{
	if(str[0]==find[0])
	{
		if (find[1]==0)
			return 1;
		return (instr(str+1,find+1));
	}
	if(str[1]==0)
		return 0;
	return (instr(str+1,find));
}
   
   
struct termios options;

void openPortRaw(void)
{
    /* open the port */
    fd = open("/dev/ttySMS", O_RDWR | O_NOCTTY | O_NDELAY);
    printf("fd=%d\n",fd);
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
        if (instr(buffer, expect))
        {
			if(replyBuff!=NULL)
				strcpy(replyBuff,buffer);
			return (0);
		}
      }

      return (-1);
    }

//Next you need to establish communications with the MODEM. The best way to do this is by sending the "AT" command to the MODEM. This also allows smart MODEMs to detect the baud you are using. When the MODEM is connected correctly and powered on it will respond with the response "OK".
//
//    Listing 4 - Initializing the MODEM.

    int                  /* O - 0 = MODEM ok, -1 = MODEM bad */
    init_modem()   /* I - Serial port file */
    {
      char buffer[255];  /* Input buffer */
      char *bufptr;      /* Current char in buffer */
      int  nbytes;       /* Number of bytes read */
      int  tries;        /* Number of tries so far */
	  int res;			//response
	  
      for (tries = 0; tries < 3; tries ++)
      {
       /* send an AT command followed by a CR */
        if ((res=write(fd, "AT\r", 3)) < 3)
        {
			continue;
		}
		printf("wrote %d bytes\n",res);
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
		printf("%s %d\n",buffer,strlen(buffer));
        if (instr(buffer, "OK"))
          return (0);
      }

      return (-1);
    }

int main(void)
{
char reply[0x100], * msgNum, value[0x100];
int i;
	openPortRaw();
	//printf("init_modem returned %d\n",init_modem());
	printf("chat returned %d\n",chat("AT\r","OK",3,NULL));
	printf("chat returned %d\n",chat("AT+CMGF=1\r","OK",3,reply));
	printf("reply=%s\n",trim(reply));
	printf("chat returned %d\n",chat("AT+CMGW=\"2814509680\"\r",">",3,reply));
	printf("reply=%s\n",trim(reply));
	printf("chat returned %d\n",chat("this msg is from Outback monitor program :)\032","+CMGW:",3,reply));
	printf("reply=%s\n",trim(reply));
	for(i=0;i<strlen(reply);i++)
	{
		if(reply[i]==':')break;
	}	
	printf("%d %d 	>%s  %d\n",i,strlen(reply),reply,__LINE__);
	if(i<strlen(reply))
	{
		msgNum=reply+i+1;
		msgNum[4]=0;
		printf("%s at line %d\n",trim(msgNum),__LINE__);
		sprintf(value,"%s%s\r","AT+CMSS=",trim(msgNum));
//		value[0]=0;
//		strcat(value,"AT+CMSS=");
//		strcat(value,trim(msgNum));
		printf("Value=%s\n",value);
		printf("chat returned %d\n",chat(value,"OK",3,reply));
		printf("reply=%s at line %d\n",trim(reply),__LINE__);
//		sprintf(value,"%s%s\r","AT+CMGD=",trim(msgNum));
//		usleep(100000);
//		printf("chat returned %d\n",chat(value,"OK",3,reply));
//		printf("reply=%s at line %d\n",trim(reply),__LINE__);
	}
	else
	{
		printf("failed line %d  i=%d strlen=%d 	reply=%s\n",__LINE__,i,strlen(reply),reply);
	}
	return 0;
}
	
