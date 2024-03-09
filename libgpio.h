#ifndef LIBGPIO_H__
#define LIBGPIO_H__

#include <stdint.h>

extern void IO_HWInit(void); 

extern void IO_LED1_On(void); 
extern void IO_LED1_Off(void); 
extern void IO_LED2_On(void); 
extern void IO_LED2_Off(void); 

extern void IO_Servo_Start(void); 
extern void IO_Servo_Stop(void); 
extern void IO_Servo_SetPos(uint8_t pos); 

#endif

