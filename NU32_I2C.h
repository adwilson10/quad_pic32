#ifndef __NU32I2C_H
#define __NU32I2C_H

#define I2C_CLOCK_FREQ  400000

  //prototypes for functions
  void I2Cinitialize(void);
  void InitializeIMU(void);
  void I2Cwrite(char deviceaddress, char registeraddress, char value);
  void I2Creadonebyte(char * variable);
  void I2Csendonebyte(char value);
  int I2Cstartevent(void);
  void I2Crepeatstartevent(void);
  void I2Cstopevent(void);
  void I2Cread(char writeaddress, char registeraddress, char * value);
  void ResetI2C(void);
  void Read_MPU6050(unsigned char value[]);
  void Read_HMC5883L(unsigned char value[]);

#endif // __NU32_H

