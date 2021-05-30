#include "Robot.h"

U8G2_SSD1306_128X80_NONAME_F_HW_I2C u8g2(U8G2_R1, /* reset=*/ U8X8_PIN_NONE, /* SCL=*/ 22, /* SDA=*/21);
MPU6050 accelgyro;
CAN_device_t CAN_cfg;

Robot::Robot()
{

}

void Robot::Init()
{
    u8g2.begin();
    u8g2.enableUTF8Print();
    u8g2.setFont(u8g2_font_6x10_mf);
    u8g2.setFontDirection(0);
    u8g2.clearBuffer();

    u8g2.setCursor(0, 10);
    u8g2.printf("[AX] P:%.0f D:%.0f", PID_AngleX.p, PID_AngleX.d);
    u8g2.setCursor(0, 20);
    u8g2.printf("[SX] P:%.0f I:%.0f", PID_SpeedX.p, PID_SpeedX.i);
    u8g2.sendBuffer();


    ledcSetup(8, 50, 10);
    ledcAttachPin(27, 8);

    PID_AngleX.relax_point = 1.71;
    PID_AngleX.p = 1000;
    PID_AngleX.d = 0;

    PID_SpeedX.setpoint = 0;
    PID_SpeedX.p = 0.000001;
    PID_SpeedX.i = 0;
}


void Robot::SetTurnAngle(float angle)
{

}
