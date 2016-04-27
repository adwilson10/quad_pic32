#ifndef GY86_H
#define	GY86_H

#define ACC_CON         0.00549316406
#define GYRO_CON        0.03048780487 //FS_SEL=2
#define FILT_CON        0.025
#define YAW_FILT        0.0025
#define DT              0.01

    void UpdateIMU(float imu_values[]);

#endif // GY86_H


