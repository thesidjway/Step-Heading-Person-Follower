#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>


#define INPUT_1 3
#define INPUT_2 2
#define INPUT_3 8
#define INPUT_4 6
#define ENABLE_1 11
#define ENABLE_2 4
#define STEP_TIME 1000
#define DUTYCYCLE 128
#define INITIALIZE_TURN 100
#define STOP_TURN 100
#define RX 10
#define TX 11
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
char  buffer_in[4];
float heading , DIFF , TARGET_ANGLE;
SoftwareSerial receiver(RX, TX);




void setup() {

  pinMode(INPUT_1,OUTPUT);
  pinMode(INPUT_2,OUTPUT);
  pinMode(INPUT_3,OUTPUT);
  pinMode(INPUT_4,OUTPUT);
  pinMode(ENABLE_1,OUTPUT);
  pinMode(ENABLE_2,OUTPUT);
  
  
  receiver.begin(38400);
  Serial.begin(38400);
 if (!mag.begin())
  {
    Serial.println("No HMC5883 detected");
    while (1);
  }


  //  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

}

void loop() {

 // retrive_data();
  //turn();
  get_heading();
  run_bot();
}



float get_heading ()
{
  Serial.println("in heading");
  sensors_event_t event;
  mag.getEvent(&event);

  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0;
  heading += declinationAngle;
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;
  float headingDegrees = heading * 180 / M_PI;
    Serial.print("Heading: ");
  Serial.println(headingDegrees);
  return headingDegrees;

}


void turn()
{
  //   if (motion == 0)
  {

   // delay(INITIALIZE_TURN);
    /*;*/
    DIFF = get_heading() - TARGET_ANGLE;
    
    while (DIFF > 10|| DIFF < -10)
    {
Serial.print("DIFF: ");
    Serial.println(DIFF);
      if (DIFF <= -10)
        turn_cw();

      else if (DIFF >= 10)
        turn_ccw();

      DIFF = get_heading() - TARGET_ANGLE;

    }
    stop_turn();
    //motion == 1;
    //}
  }}

  void turn_cw ()
  {
   // Serial.println("In turn cw");
  analogWrite(ENABLE_1, DUTYCYCLE);
   analogWrite(ENABLE_2, DUTYCYCLE);
    digitalWrite(INPUT_1, HIGH);
    digitalWrite(INPUT_2, LOW);
    digitalWrite(INPUT_3, LOW);
    digitalWrite(INPUT_4, HIGH);

  }

  void turn_ccw ()
  {
   // Serial.println("In turn ccw");
   analogWrite(ENABLE_1,DUTYCYCLE);
    analogWrite(ENABLE_2, DUTYCYCLE);
    digitalWrite(INPUT_1, LOW);
    digitalWrite(INPUT_2, HIGH);
    digitalWrite(INPUT_3, HIGH);
    digitalWrite(INPUT_4, LOW);

  }


  void run_bot()
  {
    Serial.println("In runbot");
    //if(motion == 1){
    analogWrite(ENABLE_1,255);
    analogWrite(ENABLE_2,255);

    //while(STEPS==0){

    digitalWrite(INPUT_1, HIGH);
    digitalWrite(INPUT_2, LOW);
    digitalWrite(INPUT_3, HIGH);
    digitalWrite(INPUT_4, LOW);

    delay(STEP_TIME);
    //STEPS -= 1;   }
  /*  Serial.println(digitalRead(ENABLE_1));
    Serial.println(digitalRead(ENABLE_2));
    Serial.println(digitalRead(INPUT_1));
    Serial.println(digitalRead(INPUT_2));
    Serial.println(digitalRead(INPUT_3));
    Serial.println(digitalRead(INPUT_4));
    
    Serial.println("LllllllllllllllL");*/
    digitalWrite(ENABLE_1, LOW);
    analogWrite(ENABLE_2, LOW);
   /* 
    Serial.println(digitalRead(ENABLE_1));
    Serial.println(digitalRead(ENABLE_2));
    Serial.println(digitalRead(INPUT_1));
    Serial.println(digitalRead(INPUT_2));
    Serial.println(digitalRead(INPUT_3));
    Serial.println(digitalRead(INPUT_4));
    */


    //motion =0;
    //}
  }

  void stop_turn()
  {

    digitalWrite(INPUT_1, LOW);
    digitalWrite(INPUT_2, LOW);
    digitalWrite(INPUT_3, LOW);
    digitalWrite(INPUT_4, LOW);
    delay(STOP_TURN);
    digitalWrite(ENABLE_1, LOW);
    digitalWrite(ENABLE_2, LOW);

  }


int cntr=0;
int rec_val;
char reading;
 void retrive_data()
  {
    
    while(!receiver.available());
    
    for ( cntr=0, reading = receiver.read(); reading != '$'; cntr++ )
    
    
    {
    
    buffer_in[cntr] = reading;
    while ( !receiver.available() );
    reading = receiver.read();
    }
/*      buffer_in[cntr] = receiver.read();
      //Serial.println(buffer_in[cntr]);
      cntr++;
      Serial.println("in buffer");
    //while(!receiver.available());
    //delayMicroseconds(100); 
      if ( buffer_in[cntr]== '$')   
      */
      
      rec_val=atoi(buffer_in);
      Serial.println(rec_val);
      cntr=0;
      receiver.flush();
      TARGET_ANGLE=rec_val%1000;
      /* if(TARGET_ANGLE>=345 && TARGET_ANGLE<=360)
{
TARGET_ANGLE=345;
}
if(TARGET_ANGLE>=0 && TARGET_ANGLE<=15)
{
TARGET_ANGLE=15;
}*/
      if(rec_val/1000==1)
      {
        run_bot();
      }
     

  } // STEPS = int(buffer_in) / 1000;


