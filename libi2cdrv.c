#include "libi2c.h"

#include "libi2cdrv.h"

void * I2CDRV_Callback; 

void I2CDRV_CB_R1(int st) {
	((void (*)(int, uint8_t))I2CDRV_Callback)(st, I2C_Transaction.rdbuf[0]); 
}

void I2CDRV_CB_R2(int st) {
	uint16_t rd = (uint16_t)I2C_Transaction.rdbuf[0] << 8 | I2C_Transaction.rdbuf[1]; 
	((void (*)(int, uint16_t))I2CDRV_Callback)(st, rd); 
}

void I2CDRV_CB_R3(int st) {
	uint16_t rd1 = (uint16_t)I2C_Transaction.rdbuf[0] << 8 | I2C_Transaction.rdbuf[1]; 
	uint8_t rd2 = I2C_Transaction.rdbuf[2]; 
	((void (*)(int, uint16_t, uint8_t))I2CDRV_Callback)(st, rd1, rd2); 
}

void I2CDRV_W3R0(uint8_t addr, uint8_t wr1, uint16_t wr2, void (*cb)(int st)) {
	I2C_Transaction.addr = addr; 
	I2C_Transaction.wrlen = 3; 
	I2C_Transaction.wrbuf[0] = wr1; 
	I2C_Transaction.wrbuf[1] = wr2 >> 8; 
	I2C_Transaction.wrbuf[2] = wr2; 
	I2C_Transaction.rdlen = 0; 
	I2C_Callback = cb; 
	I2C_Start(); 
}

void I2CDRV_W1R2(uint8_t addr, uint8_t wr, void (*cb)(int st, uint16_t rd)) {
	I2C_Transaction.addr = addr; 
	I2C_Transaction.wrlen = 1; 
	I2C_Transaction.wrbuf[0] = wr; 
	I2C_Transaction.rdlen = 2; 
	I2C_Callback = I2CDRV_CB_R2; 
	I2CDRV_Callback = cb; 
	I2C_Start(); 
}

void I2CDRV_W2R0(uint8_t addr, uint16_t wr, void (*cb)(int st)) {
	I2C_Transaction.addr = addr; 
	I2C_Transaction.wrlen = 2; 
	I2C_Transaction.wrbuf[0] = wr >> 8; 
	I2C_Transaction.wrbuf[1] = wr; 
	I2C_Transaction.rdlen = 0; 
	I2C_Callback = cb; 
	I2C_Start(); 
}

void I2CDRV_W2R3(uint8_t addr, uint16_t wr, void (*cb)(int st, uint16_t rd1, uint8_t rd2)) {
	I2C_Transaction.addr = addr; 
	I2C_Transaction.wrlen = 2; 
	I2C_Transaction.wrbuf[0] = wr >> 8; 
	I2C_Transaction.wrbuf[1] = wr; 
	I2C_Transaction.rdlen = 3; 
	I2C_Callback = I2CDRV_CB_R3; 
	I2CDRV_Callback = cb; 
	I2C_Start(); 
}
