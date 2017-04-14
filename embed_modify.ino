#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "I2Cdev.h"
#include "MPU6050.h"

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

SoftwareSerial BTSerial(10,11);

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO
#define LED_PIN 13

int accx, accy, accz, s = 1, isstep = 0;
int flag = 2;
float angle;


void setup(void)
{
  BTSerial.begin(38400);
  Serial.begin(38400);
  if (!mag.begin())
  {
    //Serial.println("No HMC5883 detected");
    while (1);
  }

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  accelgyro.initialize();

  pinMode(LED_PIN, OUTPUT);

}

void loop(void)
{
  // accx=analogRead(A0)-232;

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accx = ax;
  accy = ay;
  accz = az;

  /*Serial.print("accx ");
  Serial.print(analogRead(A0));
  Serial.print("accy ");
  Serial.println(analogRead(A1));
  */

  angle = atan2(accx, accz) * 180.0 / 3.1416;
  
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = atan2((float)event.magnetic.y, (float)event.magnetic.x);
  float declinationAngle = 0;
  heading += declinationAngle;
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;
  Serial.println(heading);
    float headingDegrees = heading * 180.0 / PI;
  Serial.println(angle);
  
  if (angle <= -50 && angle >= -90 && isstep == 1)
  {
    s++;
    flag = 1;
    isstep = 0;
  }
  if (angle <= -110 && angle >= -150 && isstep == 0)
  {
    isstep = 1;
  }
  
  delay(500);
  long val = flag * 1000 + (int)headingDegrees;
  char string_val[4];
  itoa(val, string_val, 10);
  Serial.println(string_val);

  BTSerial.print(string_val);
  BTSerial.print('$');
  flag = 2;
}


