//load_ontrol.h

#ifndef __load_control_h__

#define __load_control_h__
#define	MAX_LOAD_AMPS		33
#define LOWER_HV_L1_DIFF	20
#define LOWER_HV_L2_DIFF	10
#define LOWER_LV_L2_DIFF	10
#define LOWER_LV_L1_DIFF	0
#define UPPER_L1_DIFF		10
#define UPPER_L2_DIFF		0
#define COMPRESSOR_L1_DIFF	11
#define COMPRESSOR_L2_DIFF	0
#define AIR_COND_AMPS		7
	
	enum AirCondPwrSrcModes{acpsGrid,acpsInverter,acpsNone,acpsOn};
	void LoadControl(void);
	void loadShed(void);
	int setACPS(enum AirCondPwrSrcModes);
	
#endif
