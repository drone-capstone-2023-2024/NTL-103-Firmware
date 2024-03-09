#include <stm32g030xx.h>

#include "librcc.h"    // System clock frequency
#include "libgpio.h"   // LED, servo, misc

#include "liblfp.h"    // LFP
#include "liblfphw.h"  // LFP HW middleware

#include "libi2c.h"    // I2C

#include "i2ctelemetry.h" // Application component


// Periodic service utility

typedef struct {
	uint8_t pending; 
	uint8_t interval; 
	uint8_t counter; 
} PeriodicService_t; 

PeriodicService_t PeriodicService_Init(uint8_t interval) {
	PeriodicService_t service = {
		.pending = 0, 
		.interval = interval, 
		.counter = 0, 
	}; 
	return service; 
}

void PeriodicService_Force(PeriodicService_t * service) {
	service->pending = 1; 
}

void PeriodicService_SetInterval(PeriodicService_t * service, uint8_t interval) {
	service->interval = interval; 
	service->counter = interval; // Force update
}

void PeriodicService_Tick(PeriodicService_t * service) {
	if(++service->counter >= service->interval) {
		service->counter = 0; 
		service->pending = 1; 
	}
}

int PeriodicService_Serve(PeriodicService_t * service) {
	if(service->pending) {
		service->pending = 0; 
		return 1; 
	}
	return 0; 
}


// Utility

void Util_WriteFloat(LFP_Frame_t * frame, int startIndex, float number) {
	uint8_t * arr = (uint8_t *)&number; 
	frame->payload[startIndex    ] = arr[3]; 
	frame->payload[startIndex + 1] = arr[2]; 
	frame->payload[startIndex + 2] = arr[1]; 
	frame->payload[startIndex + 3] = arr[0]; 
}


// Application code

int main(void) {
	SysClock_HWInit(); 
	IO_HWInit(); 
	LFP_HWInit(); 
	I2C_HWInit(); 
	
	IO_LED1_On(); 
	I2CDEV_Init(); 
	IO_LED1_Off(); 
	
	// SysTick 20Hz
	SysTick->CTRL = 0x00; 
	SysTick->VAL = 0x00; 
	SysTick->LOAD = 400000 - 1; 
	SysTick->CTRL = 0x01; 
	
	uint8_t cntsec = 0; // Second counter
	
	PeriodicService_t powerService = PeriodicService_Init(2); 
	PeriodicService_t tempService = PeriodicService_Init(2); 
	PeriodicService_t envService = PeriodicService_Init(2); 
	
	while(1) {
		uint8_t doSampling = 0; 
		
		LFP_Frame_t frame; 
		
		// Handle OUT packets
		if(LFP_RxAvailable() && LFP_Rx(&frame) == LFP_DECODE_OKAY) {
			switch(frame.service) {
				case 0: { // Power telemetry
					// Power telemetry
					if(frame.length == 0) {
						// PacketOutPowerRequest
						PeriodicService_Force(&powerService); 
					}
					if(frame.length == 1) {
						// PacketOutPowerSetRate
						PeriodicService_SetInterval(&powerService, frame.payload[0]); 
					}
					break; 
				}
				case 1: { // Temperature telemetry
					if(frame.length == 0) {
						// PacketOutTempRequest
						PeriodicService_Force(&tempService); 
					}
					if(frame.length == 1) {
						// PacketOutPowerSetRate
						PeriodicService_SetInterval(&tempService, frame.payload[0]); 
					}
					break; 
				}
				case 2: { // Environmental sensor
					if(frame.length == 0) {
						// PacketOutEnvRequest
						PeriodicService_Force(&envService); 
					}
					if(frame.length == 1) {
						// PacketOutPowerSetRate
						PeriodicService_SetInterval(&envService, frame.payload[0]); 
					}
					break; 
				}
                case 3: { // Actuator
                    if(frame.length == 0) {
                        // PacketOutServoStop
                        IO_Servo_Stop();
                    }
                    if(frame.length == 1) {
                        // PacketOutServoSet
                        IO_Servo_SetPos(frame.payload[0]);
                    }
                }
			}
		}
		
		// Handle tick
		if(SysTick->CTRL & 0x00010000) { // 20Hz
			if(++cntsec >= 20) { // 1Hz
				cntsec = 0; 
				// Logic: I2C periodic sampling
				doSampling = 1; 
				// Logic: Counter 1 Power telemetry
				PeriodicService_Tick(&powerService); 
				// Logic: Counter 2 Temperature telemetry
				PeriodicService_Tick(&tempService); 
				// Logic: Counter 3 Environmental telemetry
				PeriodicService_Tick(&envService); 
				// .. more logic
			}
			// Logic: periodic flashing
			if(cntsec == 1) IO_LED1_On(); 
			if(cntsec == 2) IO_LED1_Off(); 
			// Logic (temporary): low battery indicator
			if(I2C_Telemetry.converterV > 6.5 && I2C_Telemetry.converterV < 19.8) {
				if(cntsec & 0x1) IO_LED2_On(); 
				else IO_LED2_Off(); 
			}
			// .. more logic
		}
		
		// Telemetry writeback
		if(PeriodicService_Serve(&powerService)) {
			// PacketInPowerTelemetry
			frame.service = 0; 
			frame.length = 12; 
			Util_WriteFloat(&frame, 0, I2C_Telemetry.converterV); 
			Util_WriteFloat(&frame, 4, I2C_Telemetry.converterA); 
			Util_WriteFloat(&frame, 8, I2C_Telemetry.converterW); 
			LFP_BlockingTx(&frame); 
		}
		if(PeriodicService_Serve(&tempService)) {
			// PacketInTempTelemetry
			frame.service = 1; 
			frame.length = 20; 
			Util_WriteFloat(&frame, 0, I2C_Telemetry.boardTemp); 
			for(int i = 4; i < 20; i++) frame.payload[i] = 0xFF; 
			LFP_BlockingTx(&frame); 
		}
		if(PeriodicService_Serve(&envService)) {
			// PacketInEnvTelemetry
			frame.service = 2; 
			frame.length = 8; 
			if(I2C_Telemetry.extSensorPresent) {
				Util_WriteFloat(&frame, 0, I2C_Telemetry.extSensorTemp); 
				Util_WriteFloat(&frame, 4, I2C_Telemetry.extSensorHumid); 
			}
			else for(int i = 0; i < 8; i++) frame.payload[i] = 0xFF; 
			LFP_BlockingTx(&frame); 
			
		}
		
		// Trigger sampling at the very end
		if(doSampling) {
			I2CDEV_Sample(); 
		}
	}
}

