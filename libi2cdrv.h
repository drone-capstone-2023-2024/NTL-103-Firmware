#ifndef LIBI2CDRV_H__
#define LIBI2CDRV_H__

#include <stdint.h>

extern void I2CDRV_W3R0(uint8_t addr, uint8_t  wr1, uint16_t wr2, void (*cb)(int st)); 
extern void I2CDRV_W1R2(uint8_t addr, uint8_t  wr,                void (*cb)(int st, uint16_t rd)); 
extern void I2CDRV_W2R0(uint8_t addr, uint16_t wr,                void (*cb)(int st)); 
extern void I2CDRV_W2R3(uint8_t addr, uint16_t wr,                void (*cb)(int st, uint16_t rd1, uint8_t rd2)); 

#endif
