#ifndef I2CTELEM_H__
#define I2CTELEM_H__

typedef struct {
	int initialized; 
	float converterV; 
	float converterA; 
	float converterW; 
	float boardTemp; 
	int extSensorPresent; 
	float extSensorTemp; 
	float extSensorHumid; 
} I2C_Telemetry_t; 

extern I2C_Telemetry_t I2C_Telemetry; 

extern void I2CDEV_Init(void); 
extern void I2CDEV_Sample(void); 

#endif
