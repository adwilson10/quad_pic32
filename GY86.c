#include <plib.h>
#include "GY86.h"
#include "NU32v2.h"

int ROLL_Zero = 0;
int PITCH_Zero = 0;
int YAW_Zero = 0;
int DPITCH_Zero = 0;
int DROLL_Zero = 0;
int DYAW_Zero = 0;

void CalibrateIMU(void)
{
    unsigned char imu_data[14];
    short acc_x=0;  short acc_y=0;  short acc_z=0;
    short gyro_x=0; short gyro_y=0; short gyro_z=0;
    int i=0;

    for (i=0; i<20;i++){
        Read_MPU6050(imu_data);
        acc_x += ((imu_data[0] << 8) | imu_data[1]);
        acc_y += ((imu_data[2] << 8) | imu_data[3]);
        acc_z += ((imu_data[4] << 8) | imu_data[5]);
        gyro_x += ((imu_data[8] << 8) | imu_data[9]);
        gyro_y += ((imu_data[10] << 8) | imu_data[11]);
        gyro_z += ((imu_data[12] << 8) | imu_data[13]);

        ShortDelay(400000);
    }
    float acc_x_cal = acc_x*0.05;
    float acc_y_cal = acc_y*0.05;
    float acc_z_cal = acc_z*0.05;
    float gyro_x_cal = gyro_x*0.05;
    float gyro_y_cal = gyro_y*0.05;
    float gyro_z_cal = gyro_z*0.05;

    ROLL_Zero = acc_x_cal;
    PITCH_Zero = acc_y_cal;
    //YAW_Zero = acc_z_cal;
    DROLL_Zero = gyro_x_cal;
    DPITCH_Zero = gyro_y_cal;
    DYAW_Zero = gyro_z_cal;

}

void UpdateIMU(float imu_values[])
{
    unsigned char imu_data[14]={0};
    unsigned char mag_data[6]={0};

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

    float roll = (acc_x-ROLL_Zero)*ACC_CON;
    float pitch = (acc_y-PITCH_Zero)*ACC_CON;
    float yaw = (acc_z-YAW_Zero)*ACC_CON;

    float dpitch = (gyro_x-DROLL_Zero)*GYRO_CON;
    float droll = -(gyro_y-DPITCH_Zero)*GYRO_CON;
    float dyaw = (gyro_z-DYAW_Zero)*GYRO_CON;

    Read_HMC5883L(mag_data);
    mag_x = ((mag_data[0] << 8) | mag_data[1]);
    mag_z = ((mag_data[2] << 8) | mag_data[3]);
    mag_y = ((mag_data[4] << 8) | mag_data[5]);

    imu_values[0] = droll;
    imu_values[1] = dpitch;
    imu_values[2] = dyaw;
    imu_values[3] = roll;
    imu_values[4] = pitch;
    imu_values[5] = yaw;
    imu_values[6] = mag_x;
    imu_values[7] = mag_y;
    imu_values[8] = mag_z;

}
