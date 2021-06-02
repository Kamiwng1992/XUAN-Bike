#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define RADIANS_TO_DEGREES  57.2958


MPU6050 palm;

Quaternion q ;

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize = 42;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount ;     // count of all bytes currently in FIFO
uint8_t fifoBuffer [64]; // FIFO storage buffer

void setup()
{
  Wire.begin(32, 33);
  Serial.begin(115200);
  while (!Serial);

  palm.initialize();
  devStatus = palm.dmpInitialize();
  if (devStatus == 0)
  {
    Serial.println(F("Enabling palm..."));
    palm.setDMPEnabled(true);
    //packetSize = palm.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }


}

long timer;
void loop()
{
  mpuIntStatus = palm.getIntStatus();
  if ((mpuIntStatus & 0x10) || fifoCount == 16)
  {
    palm.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02)
  {
    do {
      fifoCount = palm.getFIFOCount();
    }
    while (fifoCount < packetSize) ;
    palm.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    palm.dmpGetQuaternion(&q, fifoBuffer);
  }

  if (millis() - timer > 15)
  {
    Serial.print(q.w, 4);
    Serial.print(",");
    Serial.print(q.x, 4);
    Serial.print(",");
    Serial.print(q.y, 4);
    Serial.print(",");
    Serial.println(q.z, 4);
    timer = millis();
  }
}
