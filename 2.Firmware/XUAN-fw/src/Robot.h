#ifndef XUAN_FW_ROBOT_H
#define XUAN_FW_ROBOT_H

#include <Arduino.h>
#include <Wire.h>
#include "Preferences.h"
#include "Control.h"
#include "U8g2lib.h"
#include "ESP32CAN.h"
#include "CAN_config.h"
#include "I2Cdev.h"
#include "MPU6050.h"


#define INERTIA 1
#define FRICTION 0

#define COMP_FILTER_RATIO           0.999
#define GYRO_LPF_CUTOFF_FREQ        100
#define ACCEL_LPF_CUTOFF_FREQ       50
#define MAX_OUTPUT_MOTOR_SPEED      200000
#define SX_INTERGRAL_MAX_ANGLE      10

class Robot
{
public:
    Robot();

    void Init();

    void SetTurnAngle(float angle);

    float GetEncoder(char motorId);

    float encoder_speed[2];
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;
    float ax_g_fil, ay_g_fil, az_g_fil;
    float gx_dps_fil, gy_dps_fil, gz_dps_fil;
    float pitch, pitch_acc;

    float mode2_pitch;
    float mode1_pitch;
    float mode1_roll;

    float turnSetpoint;
    float turnCurrent;

private:


};

extern Robot robot;
extern U8G2_SSD1306_128X80_NONAME_F_HW_I2C u8g2;
extern MPU6050 accelgyro;
extern CAN_device_t CAN_cfg;

#endif //XUAN_FW_ROBOT_H
