#ifndef __NU32v2_H
#define __NU32v2_H

#define SYS_FREQ  	(80000000L)	// clock frequency set by bootloader

#define LED0     LATBbits.LATB13
#define SW       PORTGbits.RG6

#define DESIRED_BAUDRATE_NU32 38400	//  baudrate
void initLEDs(void);
// Serial Functions
void initSerialNU32v2();
void WriteString(UART_MODULE id, const char *string);
void PutCharacter(UART_MODULE id, const char character);
void ShortDelay(UINT32 DelayCount);

char RS232_Out_Buffer[32];  // Buffer for sprintf in serial communication

#endif // __NU32v2_H


