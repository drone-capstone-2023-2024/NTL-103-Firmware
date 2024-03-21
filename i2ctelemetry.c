#include "libi2c.h"
#include "libi2cdrv.h"

#include "i2ctelemetry.h"

I2C_Telemetry_t I2C_Telemetry; 

#define TMP1075N 0x49
#define INA236A 0x41
#define SHT30 0x44

void I2CDEV_Init0(void); 
void I2CDEV_Init1(int st); 
void I2CDEV_Init2(int st); 
// I2C device initializations:
void I2CDEV_Init0(void) {
	// TMP1075N: No initialization required. Device defaults to 35ms continuous measurement mode. 
	// INA236A: Set 81.92mV shunt range, avg=16, ts=8.244ms (both), continuous
	I2CDRV_W3R0(INA236A, 0x00, 0x45FF, I2CDEV_Init1); 
}
void I2CDEV_Init1(int st) {
	if(st) {
		I2CDEV_Init0(); 
		return; 
	}
	// SHT30: Set continuous conversion mode 2Hz
	I2CDRV_W2R0(SHT30, 0x2236, I2CDEV_Init2); 
}
void I2CDEV_Init2(int st) {
	I2C_Telemetry.extSensorPresent = !st; 
	// Initialization complete
	I2C_Telemetry.initialized = 1; 
}

void I2CDEV_Act0(void); 
void I2CDEV_Act1(int st, uint16_t rd); 
void I2CDEV_Act2(int st, uint16_t rd); 
void I2CDEV_Act3(int st, uint16_t rd); 
void I2CDEV_Act4(int st, uint16_t rd1, uint8_t rd2, uint16_t rd3, uint8_t rd4); 
// I2C device readings
void I2CDEV_Act0(void) {
	// TMP1075N: read temperature
	I2CDRV_W1R2(TMP1075N, 0x00, I2CDEV_Act1); 
}
void I2CDEV_Act1(int st, uint16_t rd) {
	if(!st) I2C_Telemetry.boardTemp = (float)(int16_t)rd * 3.90625e-3; 
	// INA236A: read voltage
	I2CDRV_W1R2(INA236A, 0x02, I2CDEV_Act2); 
}
void I2CDEV_Act2(int st, uint16_t rd) {
	if(!st) I2C_Telemetry.converterV = (float)rd * 1.6e-3; 
	// INA236A: read current (39mOhm)
	I2CDRV_W1R2(INA236A, 0x01, I2CDEV_Act3); 
}
void I2CDEV_Act3(int st, uint16_t rd) {
	if(!st) I2C_Telemetry.converterA = (float)(int16_t)rd * 6.41025641e-5; 
	// TODO: power with power register
	I2C_Telemetry.converterW = I2C_Telemetry.converterV * I2C_Telemetry.converterA; 
	// SHT30: Read periodic measurement
	if(I2C_Telemetry.extSensorPresent) {
		I2CDRV_W2R6(SHT30, 0xE000, I2CDEV_Act4); 
	}
}
void I2CDEV_Act4(int st, uint16_t rd1, uint8_t rd2, uint16_t rd3, uint8_t rd4) {
	if(!st) {
		// TODO: implement CRC check
		I2C_Telemetry.extSensorTemp = rd1 * 0.0026703288319219 - 45.0; // (val/65535) * 175 - 45
		I2C_Telemetry.extSensorHumid = rd3 * 0.0015259021896696; // (val/65535) * 100
	}
	// Readout complete
}

void I2CDEV_Init(void) {
	I2C_Telemetry.initialized = 0; 
	I2C_Telemetry.extSensorPresent = 0; 
	float NaN; 
	*((uint32_t *)&NaN) = 0xFFFFFFFF; 
	I2C_Telemetry.boardTemp = NaN; 
	I2C_Telemetry.converterA = NaN; 
	I2C_Telemetry.converterV = NaN; 
	I2C_Telemetry.converterW = NaN; 
	I2C_Telemetry.extSensorTemp = NaN; 
	I2C_Telemetry.extSensorHumid = NaN; 
	I2CDEV_Init0(); 
	while(!I2C_Telemetry.initialized); 
}

void I2CDEV_Sample(void) {
	if(I2C_Busy) return; 
	I2CDEV_Act0(); 
}
