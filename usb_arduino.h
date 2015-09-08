//usb_arduino.h
#ifndef __usb_arduino_h__
#define __usb_arduino_h__

#define WH_SENSOR_TOP_INDX			2
#define WH_SENSOR_CENTER_LEFT_INDX	4
#define WH_SENSOR_CENTER_RIGHT_INDX	1
#define WH_SENSOR_BOTTOM_INDX		0

void DebugNanoPrintData(void);

void USBNano_init();

void USBNano_Close(void);

int USBNano_ReadLine(void);

float readSensorF(int SensorIndx, float ErrorValue);

struct tempSensorValue{
	float			tempC;
	float			tempF;
	unsigned long	lastUpdate;
};

	#ifndef __usb_com_c__
		extern struct tempSensorValue sensor[20];
	#endif

#endif
