//usb_com.h
#ifndef __usb_com_h__
#define __usb_com_h__

#ifndef __usb_com_c__
extern int data[8][32];
extern char USBlastResponse[256];
extern int InverterIndx, CC1Indx, CC2Indx, FNDCIndx;
extern int usbmate;
#endif //ifndef __usb_com_c__

void DebugPrintData (void);

void USB_init ();

void USB_Close (void);

void USB_ReadLine (void);

#endif //ifndef __usb_com_h__
