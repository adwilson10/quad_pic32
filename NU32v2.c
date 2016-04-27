#include <plib.h>
#include "NU32v2.h"

// Initialize LEDs on G12 and G13 as output and the switch on G6 as input
// G6 has an internal pullup resistor enabled by the bootloader
void initLEDs(void) {
  TRISBCLR = 0xDFFF;
  LATBCLR = 0xDFFF;
  TRISCCLR = 0x2000;
  LATCCLR = 0x2000;
  TRISDCLR = 0x09F0;
  LATDCLR = 0x09F0;
  TRISECLR = 0x0DFF;
  LATDCLR = 0x0DFF;
  TRISFCLR = 0x000B;
  LATFCLR = 0x000B;
  TRISGCLR = 0x038C;
  LATGCLR = 0x038C;
  TRISDCLR = 0x000F;
  
  TRISBCLR = 0x2000; 
  TRISGSET = 0x0040;
  AD1PCFG=0xFFFF;
  TRISCSET = 0x4000;
  mPORTCRead();
}

// Initialize the serial port 
void initSerialNU32v2() {
  int pbClk;
  // Configure the system performance
  pbClk = SYSTEMConfigPerformance(SYS_FREQ);

  UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
  //UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART2, pbClk, DESIRED_BAUDRATE_NU32);
  UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

  // Configure UART3 RX Interrupt
//  INTEnable(INT_U2RX, INT_ENABLED);
//  INTSetVectorPriority(INT_UART_2_VECTOR, INT_PRIORITY_LEVEL_3);
//  INTSetVectorSubPriority(INT_UART_2_VECTOR, INT_SUB_PRIORITY_LEVEL_0);
}

// Write a string over the serial port
void WriteString(UART_MODULE id, const char *string) {
  while(*string != '\0') {
  while(!UARTTransmitterIsReady(id));
  UARTSendDataByte(id, *string);
  string++;
  while(!UARTTransmissionHasCompleted(id));
  }
}

// Put a character over the serial port, called by WriteString
void PutCharacter(UART_MODULE id, const char character) {
  while(!UARTTransmitterIsReady(id));
  UARTSendDataByte(id, character);
  while(!UARTTransmissionHasCompleted(id));
}

void ShortDelay(UINT32 DelayCount)     // Delay Time (CoreTimer Ticks)
{
  UINT32 StartTime;                    // Start Time
  StartTime = ReadCoreTimer();         // Get CoreTimer value for StartTime
  while ( (UINT32)(ReadCoreTimer() - StartTime) < DelayCount ) {};
}
