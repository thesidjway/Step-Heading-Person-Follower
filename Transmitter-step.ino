#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
int accx,accy,s=1,isstep=0;
int flag=2;
float angle;
SoftwareSerial BTSerial(10,11);
void setup(void) 
{
  
  BTSerial.begin(38400);
  Serial.begin(38400);
  if(!mag.begin())
  {
   
    Serial.println("No HMC5883 detected");
    while(1);
  }
}

void loop(void) 
{
  accx=analogRead(A0)-232;
  accy=analogRead(A1)-229;
  
  Serial.print("accx ");
  Serial.print(analogRead(A0));
  Serial.print("accy ");
  Serial.println(analogRead(A1));

  
  angle=atan2(-accy,accx)*180.0/3.142;
  sensors_event_t event; 
  mag.getEvent(&event);
  float heading = atan2((float)event.magnetic.y, (float)event.magnetic.x);
  float declinationAngle = 0;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  float headingDegrees = heading * 180.0/PI; 
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  Serial.print(" Angle: ");Serial.println(angle);
  if(angle<=10 && angle>=0 && isstep==1)
  {
    s++;
    flag=1;
    isstep=0;
  }
  if(angle<=-10 && angle>=-25 && isstep==0)
  {
    isstep=1;
  }
  
  Serial.print("No. of Steps: ");
  Serial.println(2*s);
  delay(500);
  long val=flag*1000+(int)headingDegrees;
  char string_val[4];
  itoa(val,string_val,10);
  BTSerial.write(string_val);
  BTSerial.write('$');
  flag=2;
}
