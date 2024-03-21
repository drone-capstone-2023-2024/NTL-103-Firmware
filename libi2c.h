#ifndef LIBI2C_H__
#define LIBI2C_H__

#include <stdint.h>

#define I2C_WR_MAX 4
#define I2C_RD_MAX 6
typedef struct {
	uint8_t addr; 
	uint8_t index; 
	uint8_t wrlen; 
	uint8_t rdlen; 
	uint8_t wrbuf[I2C_WR_MAX]; 
	uint8_t rdbuf[I2C_RD_MAX]; 
} I2C_Transaction_t; 

#define I2C_OKAY 0
#define I2C_NACK 1
#define I2C_ARLO 2
extern void (*I2C_Callback)(int status); 

extern uint8_t I2C_Busy; 
extern I2C_Transaction_t I2C_Transaction; 

extern void I2C_HWInit(void); 
extern void I2C_Start(void); 

#endif
