

#include <iostream>
using namespace std;

#include "usb_com.h"
#include "outback.h"
#include <unistd.h>
#include <signal.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

extern int USBlastReadlineBytes;
volatile int mmTerminate=0;
 
PI_THREAD (mmInput)
{
	char asc[256];
	while(1)
	{
		cin >> asc;
		if(strcmp(asc,"q")==0)mmTerminate=1;
	}
	return  0;
} 
 
int main(int argc, char **argv)
{
	//int i;
	if(piThreadCreate(mmInput)!=0) return(1);
	USB_init();
	softPwmCreate(26,0,167);
	//for(i=0;i<1000;i++)
	while(mmTerminate==0)
	{
		//usleep(5000);
		USBlastReadlineBytes=0;
		while(USBlastReadlineBytes==0)
		{
			usleep(500000);
			cout << "#";
			USB_ReadLine();
		}
		cout << "test" /*<< i*/ << "\n";
		cout << FNDC_BATT_VOLTS << "\n";
		cout << L1_BUY_AMPS << "\n";
	}
	USB_Close();
	close(usbmate);
	return 0;
}



