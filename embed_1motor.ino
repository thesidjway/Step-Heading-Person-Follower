#define LEFT_DUTY 170
#define RIGHT_DUTY 180
#define ANGLE_THRES 5

void setup() {
pinMode(2,OUTPUT);  // left motor input1
pinMode(3,OUTPUT);  // left motor input2
pinMode(4,OUTPUT);  // right motor input1
pinMode(5,OUTPUT);  // right motor input2
pinMode(6,OUTPUT);  // enable 1
pinMode(7,OUTPUT);  // enable 2
}

void move_forward()
{
  analogWrite(6,LEFT_DUTY);
  analogWrite(7,RIGHT_DUTY);
  digitalWrite(2,HIGH);
  digitalWrite(3,LOW);
  digitalWrite(4,HIGH);
  digitalWrite(5,LOW);
}

void move_backward()
{
  analogWrite(6,LEFT_DUTY);
  analogWrite(7,RIGHT_DUTY);
  digitalWrite(2,LOW);
  digitalWrite(3,HIGH);
  digitalWrite(4,LOW);
  digitalWrite(5,HIGH);
}

void move_left()
{
  analogWrite(6,LEFT_DUTY);
  analogWrite(7,RIGHT_DUTY);
  digitalWrite(2,LOW);
  digitalWrite(3,HIGH);
  digitalWrite(4,HIGH);
  digitalWrite(5,LOW);
}

void move_right()
{
  analogWrite(6,LEFT_DUTY);
  analogWrite(7,RIGHT_DUTY);
  digitalWrite(2,HIGH);
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
  digitalWrite(5,HIGH);
}

int getMagnetoValue()
{
  
}
void correctDirection(int desiredAngle)
{
  int currAngle = getMagnetoValue();
  while ( fabs(desiredAngle - currAngle)> ANGLE_THRES )
  {
    if (desiredAngle > currAngle)
    {
      move_right();
      delay(10);
    }
    else if (desiredAngle < currAngle)
    {
      move_left();
      delay(10);
    }
  }
}
    
  
void loop() 
{
  move_forward();
  
  //move_backward();
 
}
