//usb_com.h

extern volatile int STOP; 
//const int TypeFieldCount[8]={0,0,0,15,13,0,23,0};
extern int data[8][32];
extern char USBlastResponse[256];
extern int InverterIndx,CC1Indx,CC2Indx,FNDCIndx;

extern int usbmate;
extern struct termios oldtio;

//void ProcessBuff(char * line);

void DebugPrintData(void);

void USB_init();

void USB_Close(void);

void USB_ReadLine(void);

//int ProcessResponse(char * line, int maxChar);

//void ProcessBuff(char * line);
