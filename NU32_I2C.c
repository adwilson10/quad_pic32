#include <plib.h>
#include "NU32v2.h"
#include "NU32_I2C.h"

char NU32_RS232OutBuffer[32]; // Buffer for sprintf in serial tx

//Turn on I2C Module and calibrate for standard operation.
void I2Cinitialize(void){

    I2CSetFrequency(I2C1, SYS_FREQ, I2C_CLOCK_FREQ);
    I2CEnable(I2C1, TRUE);
}

void InitializeIMU(void){

    I2Cinitialize();  //Turn on and initialize I2C module 1
    ShortDelay(5000);
    I2Cwrite((0x68 << 1)|I2C_WRITE, 0x6B, 0x00); //Initialize MPU6050
    ShortDelay(5000);
    I2Cwrite((0x68 << 1)|I2C_WRITE, 0x19, 0x01); //Set Sample Rate Divider
    ShortDelay(5000);
    I2Cwrite((0x68 << 1)|I2C_WRITE, 0x1A, 0x01); //Set DLPF
    ShortDelay(5000);
    I2Cwrite((0x68 << 1)|I2C_WRITE, 0x1B, 0x10); //Change Gyro sensitivity
    ShortDelay(5000);
    I2Cwrite((0x68 << 1)|I2C_WRITE, 0x6A, 0x00); //Disable Aux I2C
    ShortDelay(5000);
    I2Cwrite((0x68 << 1)|I2C_WRITE, 0x37, 0x02); //Aux I2C Bypass
    //WriteString(UART2,"Initialized MPU6050\r\n");
    ShortDelay(10000);
    I2Cwrite((0x1E << 1)|I2C_WRITE, 0x02, 0x00); //Initialize H
    ShortDelay(50000);
}


//This function sends one byte via I2C. It should only be called after a
//start event.
void I2Csendonebyte(char value){
  //check to see that the transmitter buffer is empty
  if(!I2C1STATbits.TBF){
     I2C1TRN = value; //Load the transmit register with the value to transmit
     //Check to ensure there was not a bus collision or a write collision
     if(I2C1STATbits.BCL | I2C1STATbits.IWCOL){
       sprintf(NU32_RS232OutBuffer,"Transmit or Write Collision\r\n");
     WriteString(UART2,NU32_RS232OutBuffer);
     }
     //wait until transmission status is cleared at the end of transmisssion
     while(I2C1STATbits.TRSTAT){
     }
  } else{  //report error if transmit buffer was already full
     sprintf(NU32_RS232OutBuffer,"Transmit Buffer Full\r\n");
     WriteString(UART2,NU32_RS232OutBuffer);
  }
  //Report if byte not acknowledged. This could indicate that the wrong byte
  //was sent or the SCLK and SDA lines were not hooked up correctly.
  if (I2C1STATbits.ACKSTAT){
     sprintf(NU32_RS232OutBuffer,"Byte not Acknowledged\r\n");
     WriteString(UART2,NU32_RS232OutBuffer);
  }
}

//This function reads one byte via I2C. It should only be used after a start
//event and the slave device have already been addressed as per I2C protocol
void I2Creadonebyte(char * variable){
  I2C1CONbits.RCEN = 1;    //Enable recieve
  if(I2C1STATbits.I2COV){  //Report if recieve events overlap.
     sprintf(NU32_RS232OutBuffer,"Data Overflow\r\n");
     WriteString(UART2,NU32_RS232OutBuffer);
  }
  while(!I2C1STATbits.RBF); //Wait until recieve register is full.
 // I2C1CONbits.ACKEN = 1;
  //Save recieved data to variable. Automatically clears RCEN and RBF
  *variable = I2C1RCV;
}

//Initiates a start event, master pulls SDA low hwile SCLK is high
int I2Cstartevent(void){
  int flag=0;
  //Check that the last event was a stop event which means the bus is idle
  if(!I2C1STATbits.S){
     I2C1CONbits.SEN = 1;   //initiate start event
     flag=1;
     if (I2C1STATbits.BCL){ //check for bus collisions
      //if there was a collision report the error
      sprintf(NU32_RS232OutBuffer,"Start Event Collision\r\n");
      WriteString(UART2,NU32_RS232OutBuffer);
      flag=0;
    }
    //wait until the end of the start event clears the start enable bit
    while(I2C1CONbits.SEN);
  }
  else{ //if the bus was not idle report the error
    sprintf(NU32_RS232OutBuffer,"Bus is not idle\r\n");
     WriteString(UART2,NU32_RS232OutBuffer);
     flag=0;
  }
  return flag;
}

//Initiates a repeat start event
void I2Crepeatstartevent(void){
 //Initiates a repeat start event, master pulls SDA low
  I2C1CONbits.RSEN = 1;
  if (I2C1STATbits.BCL){
    sprintf(NU32_RS232OutBuffer,"Repeat Start Event Collision\r\n");
     WriteString(UART2,NU32_RS232OutBuffer);
  }
  //wait until the end of the repeatstart event clears the repeat
  //start enable bit
  while(I2C1CONbits.RSEN);
}

//Initiates a stop event to end transmission and leave bus idle
void I2Cstopevent(void){
  //Initiates a stop event, master sets SDA high on a high SCLK
  I2C1CONbits.PEN = 1;
  //wait for stop event enable to be cleared by completion of stop event
  while(I2C1CONbits.PEN);
}

void ResetI2C(void){
    int i=0;
    int j=0;
    I2CEnable(I2C1, FALSE);
    TRISBSET = 0x0200;
    if(PORTDbits.RD9==0){
        sprintf(NU32_RS232OutBuffer,"Bus is low...\r\n");
        WriteString(UART2,NU32_RS232OutBuffer);
      }
    TRISDCLR = 0x0400;
    while(PORTDbits.RD9==0){
        LATDbits.LATD10=!LATDbits.LATD10;
        sprintf(NU32_RS232OutBuffer,"Pulsing SCK...\r\n");
        WriteString(UART2,NU32_RS232OutBuffer);
        ShortDelay(400000);
     }
    sprintf(NU32_RS232OutBuffer,"SDA high...\r\n");
    WriteString(UART2,NU32_RS232OutBuffer);
    TRISDSET = 0x0400;
    ShortDelay(10000);
    I2CEnable(I2C1, TRUE);
    ShortDelay(50000);
    I2Cstopevent();
}

//This function initiates a full transmission to write a single byte to
//a register of a slave device via I2C. The device address is the 7 bit
//address of the slave device with a 0 as the least significant bit.
void I2Cwrite(char deviceaddress, char registeraddress, char value){
  while(I2Cstartevent()==0){
      sprintf(NU32_RS232OutBuffer,"Trying to stop bus...\r\n");
      WriteString(UART2,NU32_RS232OutBuffer);
      ResetI2C();
  }

  I2Csendonebyte(deviceaddress);
  I2Csendonebyte(registeraddress);
  I2Csendonebyte(value);

  I2Cstopevent();
}

//This function initiates a full transmission to read a single byte from
//a register of a slave device via I2C.The device address is the 7 bit
//address of the slave device with a 0 as the least significant bit.
void I2Cread(char writeaddress, char registeraddress, char * value){
  char readaddress = writeaddress + 0x01;
  while(I2Cstartevent()==0){
      sprintf(NU32_RS232OutBuffer,"Trying to stop bus...\r\n");
      WriteString(UART2,NU32_RS232OutBuffer);
      ResetI2C();
  }
  I2Csendonebyte(writeaddress);
  I2Csendonebyte(registeraddress);

  I2Crepeatstartevent();
  I2Csendonebyte(readaddress);
  I2Creadonebyte(value);
  I2Cstopevent();
}

void Read_MPU6050(unsigned char value[]){

  char writeaddress = (0x68 << 1)|I2C_WRITE;
  char readaddress = writeaddress + 0x01;
  int i=0;

  I2C1CONbits.ACKDT = 0;    //set ACK
  while(I2Cstartevent()==0){
      sprintf(NU32_RS232OutBuffer,"Trying to stop bus...\r\n");
      WriteString(UART2,NU32_RS232OutBuffer);
      ResetI2C();
  }
  I2Csendonebyte(writeaddress);
  I2Csendonebyte(0x3B);

  I2Crepeatstartevent();
  I2Csendonebyte(readaddress);

  for(i=0; i<13; i++)
  {
      I2Creadonebyte(&value[i]);
      I2C1CONbits.ACKEN=1;
      while(I2C1CONbits.ACKEN);
  }

  I2Creadonebyte(&value[13]);
  I2C1CONbits.ACKDT = 1;
  I2C1CONbits.ACKEN=1;  //Set NACK
  while(I2C1CONbits.ACKEN);
  I2Cstopevent();

}

void Read_HMC5883L(unsigned char value[]){

  char writeaddress = (0x1E << 1)|I2C_WRITE;
  char readaddress = writeaddress + 0x01;

  I2C1CONbits.ACKDT = 0;    //set ACK
  while(I2Cstartevent()==0){
      sprintf(NU32_RS232OutBuffer,"Trying to stop bus...\r\n");
      WriteString(UART2,NU32_RS232OutBuffer);
      ResetI2C();
  }
  I2Csendonebyte(writeaddress);
  I2Csendonebyte(0x03);

  I2Crepeatstartevent();
  I2Csendonebyte(readaddress);
  I2Creadonebyte(&value[0]);
  I2C1CONbits.ACKEN=1;
  while(I2C1CONbits.ACKEN);
  I2Creadonebyte(&value[1]);
  I2C1CONbits.ACKEN=1;
  while(I2C1CONbits.ACKEN);
  I2Creadonebyte(&value[2]);
  I2C1CONbits.ACKEN=1;
  while(I2C1CONbits.ACKEN);
  I2Creadonebyte(&value[3]);
  I2C1CONbits.ACKEN=1;
  while(I2C1CONbits.ACKEN);
  I2Creadonebyte(&value[4]);
  I2C1CONbits.ACKEN=1;
  while(I2C1CONbits.ACKEN);
  I2Creadonebyte(&value[5]);
  I2C1CONbits.ACKDT = 1;    //set NACK
  I2C1CONbits.ACKEN=1;
  while(I2C1CONbits.ACKEN);
  I2Cstopevent();

}

