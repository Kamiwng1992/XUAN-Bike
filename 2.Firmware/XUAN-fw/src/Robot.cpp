#include "Robot.h"

U8G2_SSD1306_128X80_NONAME_F_HW_I2C u8g2(U8G2_R3, /* reset=*/ U8X8_PIN_NONE, /* SCL=*/ 22, /* SDA=*/21);
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

    // ToDo: Very wired thing, CANNOT delete these code otherwise Bike goes unstable
    ledcSetup(8, 50, 10);
    ledcAttachPin(27, 8);
}


void Robot::SetTurnAngle(float angle)
{

}
