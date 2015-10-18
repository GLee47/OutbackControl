//load_ontrol.h

#ifndef __load_control_h__
	#define __load_control_h__
	
	enum AirCondPwrSrcModes{acpsGrid,acpsInverter,acpsNone,acpsOn};
	void LoadControl(void);
	void loadShed(void);
	int setACPS(enum AirCondPwrSrcModes);
	
#endif
