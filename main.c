#include <plib.h>
#include "NU32v2.h"
#include "NU32_I2C.h"
#include "math.h"
					
#pragma config FSOSCEN = OFF

#define CONTROL_FREQ	500
#define CON_PRE_SCALER	8
#define COMM_FREQ       60
#define COMM_PRE_SCALER	32


#define ACC_CON         0.00549316406
#define GYRO_CON        0.03048780487 //FS_SEL=2
#define FILT_CON        0.002
#define YAW_FILT        0.0
#define DT              0.002
#define P               10
#define D               8
#define Pyaw            15
#define Dyaw            5

short ROLL_Zero = 0;
short PITCH_Zero = 0;
short YAW_Zero = 280;
short DPITCH_Zero = 0;
short DROLL_Zero = 0;
short DYAW_Zero = 0;


int HBridgeDuty;
int iRX = 0;
int flag = 0;
int receive_buf[13] = {0};
float imu_values[9] = {0};
int datain[6] = {0};
int collect=0; int pitchcon=0; int rollcon=0;
short send_data=0;

float rollavg = 0; float pitchavg = 0; float yawavg =280;
float droll = 0; float dpitch = 0; float dyaw =0;


void CalibrateIMU(void)
{
    unsigned char imu_data[14];
    short acc_x=0;  short acc_y=0;
    short gyro_x=0; short gyro_y=0; short gyro_z=0;
    int i=0;

    for (i=0; i<20;i++){
        Read_MPU6050(imu_data);
        acc_x += ((imu_data[0] << 8) | imu_data[1]);
        acc_y += ((imu_data[2] << 8) | imu_data[3]);
        gyro_x += ((imu_data[8] << 8) | imu_data[9]);
        gyro_y += ((imu_data[10] << 8) | imu_data[11]);
        gyro_z += ((imu_data[12] << 8) | imu_data[13]);

        ShortDelay(400000);
    }
    float acc_x_cal = acc_x*0.05;
    float acc_y_cal = acc_y*0.05;
    float gyro_x_cal = gyro_x*0.05;
    float gyro_y_cal = gyro_y*0.05;
    float gyro_z_cal = gyro_z*0.05;

    ROLL_Zero = (short) acc_x_cal;
    PITCH_Zero = (short) acc_y_cal;
    DROLL_Zero = (short) gyro_x_cal;
    DPITCH_Zero = (short) gyro_y_cal;
    DYAW_Zero = (short) gyro_z_cal;

}

void quad_shutdown(void)
{
    DisableWDT();
    INTDisableInterrupts();
    SetDCOC1PWM(0);
    SetDCOC2PWM(0);
    SetDCOC3PWM(0);
    SetDCOC4PWM(0);
    LED0=0;
    while(1){

    }
}

int main(void) {

	// SYSTEMConfig() optimizes performance for our clock frequency.
	SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

	initLEDs(); 	// set LED pins as outputs

        int i =0;
        for(i=0;i<10;i++){
            ShortDelay(50000);
        }
       
//        if(RCONbits.POR==1){
//            WriteString(UART2,"PWR");
//        } else {
//            if(RCONbits.BOR==1){
//                WriteString(UART2,"BRR");
//            }
//        }
//        if(RCONbits.EXTR==1){
//            WriteString(UART2,"MCLR");
//        }
//        if(RCONbits.WDTO==1){
//            WriteString(UART2,"WTDR");
//        }
//        if(RCON & 0x0040){
//            WriteString(UART2,"SWR");
//        }

        RCON = 0x0000;

        // Configure Change Notification
        //mCNOpen(CN_ON,CN0_ENABLE,CN_PULLUP_DISABLE_ALL);
        //ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_1);

        // configure Timer 4 using internal clock with given prescaler
        OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_8, SYS_FREQ/CONTROL_FREQ/CON_PRE_SCALER - 1);
        ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_7);

        // configure Timer 2 using internal clock with given prescaler
        OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_32, SYS_FREQ/COMM_FREQ/COMM_PRE_SCALER - 1);
        ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_6);
        
  	OpenTimer3(T3_ON | T3_PS_1_1, 4000);
        OpenOC1(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
  	OpenOC2(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
        OpenOC3(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
        OpenOC4(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);

  	HBridgeDuty = 0;
        SetDCOC1PWM(HBridgeDuty);
        SetDCOC2PWM(HBridgeDuty);
        SetDCOC3PWM(HBridgeDuty);
  	SetDCOC4PWM(HBridgeDuty);
        
        InitializeIMU();
        for(i=0;i<10;i++){
            ShortDelay(20000);
        }
        //wait for button push to calibrate
        LED0=1;
        while(SW){
            ShortDelay(20000);
        }
        LED0=0;
        CalibrateIMU();
        for(i=0;i<10;i++){
            ShortDelay(20000);
        }
        
        initSerialNU32v2(); //Setup serial communication
        for(i=0;i<20;i++){
            ShortDelay(20000);
        }

        while(UARTReceivedDataIsAvailable(UART2)){
            UARTGetDataByte(UART2);
        }
        
        if(PORTCbits.RC14==0){
            DisableWDT();
            INTDisableInterrupts();
            SetDCOC1PWM(0);
            SetDCOC2PWM(0);
            SetDCOC3PWM(0);
            SetDCOC4PWM(0);

            while(1){
                LED0=!LED0;
                for(i=0;i<400;i++)
                    ShortDelay(100000);
            }
        }

        INTClearFlag(INT_SOURCE_UART_TX(UART2));
        INTClearFlag(INT_SOURCE_UART_RX(UART2));

        LED0 = 1;
        INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
        INTEnableInterrupts();

	while(1) {		// infinite loop
		
	}
	return 0;
} // end main

void __ISR(_CHANGE_NOTICE_VECTOR, ipl1) CNInterrupt(void)
{
    if(PORTCbits.RC14==0){
        quad_shutdown();
    }
    mCNClearIntFlag();
}

/* Control loop */
void __ISR(_TIMER_4_VECTOR, ipl7srs) Timer4Handler(void) {

    unsigned char imu_data[14];
    unsigned char mag_data[6];

    short acc_x=0;  short acc_y=0;  short acc_z=0;
    short gyro_x=0; short gyro_y=0; short gyro_z=0;
    short mag_z=0; short mag_x=0; short mag_y=0;

    Read_MPU6050(imu_data);
    acc_x = ((imu_data[0] << 8) | imu_data[1]);
    acc_y = ((imu_data[2] << 8) | imu_data[3]);
    acc_z = ((imu_data[4] << 8) | imu_data[5]);
    gyro_x = ((imu_data[8] << 8) | imu_data[9]);
    gyro_y = ((imu_data[10] << 8) | imu_data[11]);
    gyro_z = ((imu_data[12] << 8) | imu_data[13]);

//    Read_HMC5883L(mag_data);
//    mag_x = ((mag_data[0] << 8) | mag_data[1]);
//    //mag_z = ((mag_data[2] << 8) | mag_data[3]);
//    mag_y = ((mag_data[4] << 8) | mag_data[5]);
//
//    mag_x = (mag_x-400)*2;

    float roll = (acc_x-ROLL_Zero)*ACC_CON;
    float pitch = (acc_y-PITCH_Zero)*ACC_CON;

    float dpitch = (gyro_x-DROLL_Zero)*GYRO_CON;
    float droll = -(gyro_y-DPITCH_Zero)*GYRO_CON;
    float dyaw = (gyro_z-DYAW_Zero)*GYRO_CON;

    float heading = atan2(mag_x,mag_y);
    if(heading <0){
        heading += 2*3.14159;
    }
    float heading_deg = heading*180/3.14159;

    if(yawavg>270&&heading_deg<90){
        heading_deg=heading_deg+360;
    }else if(yawavg<90&&heading_deg>270){
        heading_deg=heading_deg-360;
    }

    rollavg = (1-FILT_CON)*(rollavg + droll*DT)+FILT_CON*roll;
    pitchavg = (1-FILT_CON)*(pitchavg + dpitch*DT)+FILT_CON*pitch;
    yawavg = (1-YAW_FILT)*(yawavg + dyaw*DT)+YAW_FILT*heading_deg;

    if(yawavg>=360){
        yawavg=yawavg-360;
    }else if(yawavg<0){
        yawavg=yawavg+360;
    }

    float yawerr = (yawavg-YAW_Zero);

    if(yawerr<-180){
        yawerr = yawerr+360;
    } else {
        if(yawerr>180){
            yawerr = yawerr-360;
        }
    }

    float motor1 = (collect*20)+P*(rollavg-rollcon*0.2)-P*(pitchavg-pitchcon*0.2)+D*droll-D*dpitch+Pyaw*yawerr+Dyaw*dyaw;
    float motor2 = (collect*20)+P*(rollavg-rollcon*0.2)+P*(pitchavg-pitchcon*0.2)+D*droll+D*dpitch-Pyaw*yawerr-Dyaw*dyaw;
    float motor3 = (collect*20)-P*(rollavg-rollcon*0.2)+P*(pitchavg-pitchcon*0.2)-D*droll+D*dpitch+Pyaw*yawerr+Dyaw*dyaw;
    float motor4 = (collect*20)-P*(rollavg-rollcon*0.2)-P*(pitchavg-pitchcon*0.2)-D*droll-D*dpitch-Pyaw*yawerr-Dyaw*dyaw;

    if(motor1<1200){
        motor1 = 0;
    }
    if(motor2<1200){
        motor2 = 0;
    }
    if(motor3<1200){
        motor3 = 0;
    }
    if(motor4<1200){
        motor4 = 0;
    }
    SetDCOC1PWM((int) motor1);
    SetDCOC2PWM((int) motor2);
    SetDCOC3PWM((int) motor3);
    SetDCOC4PWM((int) motor4);

    // clear the interrupt flag
    mT4ClearIntFlag();
}

/* Comm loop */
void __ISR(_TIMER_2_VECTOR, ipl6) Timer2Handler(void) {

    while(UARTReceivedDataIsAvailable(UART2)){

        unsigned char data;
        data = UARTGetDataByte(UART2);

        receive_buf[iRX] = ((int) data);
        iRX = iRX+1;

        if(iRX==1 && receive_buf[0]!=75){
            iRX = 0;
        }

        if(iRX==4 && receive_buf[0]==75){
            collect = receive_buf[1];
            rollcon = receive_buf[2];
            pitchcon = receive_buf[3];

            rollcon = rollcon-100;
            pitchcon = pitchcon-100;

            iRX=0;
            send_data=1;

            if(flag==0){
              //EnableWDT();
              flag=1;
            }
            ClearWDT();
        }
    }

    if(send_data==1){

        unsigned short dataout[3];
        dataout[0]= (unsigned short) (rollavg*350+32768);
        dataout[1]= (unsigned short) (pitchavg*350+32768);
        dataout[2]= (unsigned short) (yawavg*180);

        unsigned char send_buf[6];
        unsigned char battery;
        int i=0;

        if(PORTCbits.RC14==0){
            battery=0;
        } else {
            battery=1;
        }

        for(i=0; i<3; i++) {
            send_buf[2*i] = dataout[i]>>8;
            send_buf[2*i+1] = dataout[i]-(send_buf[2*i]<<8);
        }

        while(!UARTTransmitterIsReady(UART2));
        UARTSendDataByte(UART2, 0x4B);
        while(!UARTTransmissionHasCompleted(UART2));
        for(i=1; i<7; i++) {
            while(!UARTTransmitterIsReady(UART2));
            UARTSendDataByte(UART2, send_buf[i-1]);
            while(!UARTTransmissionHasCompleted(UART2));
        }
        while(!UARTTransmitterIsReady(UART2));
        UARTSendDataByte(UART2, battery);
        while(!UARTTransmissionHasCompleted(UART2));
        send_data=0;
    }
    mT2ClearIntFlag();
}

void _bootstrap_exception_handler(void){
    quad_shutdown();
}

void _general_exception_handler (unsigned cause, unsigned status){
    quad_shutdown();
}

void _simple_tlb_refill_exception_handler(void){
    quad_shutdown();
}

void _cache_err_exception_handler(void){
    quad_shutdown();
}


