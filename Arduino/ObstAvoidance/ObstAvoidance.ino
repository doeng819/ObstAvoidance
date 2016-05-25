
#include "Arduino.h"
#include <Servo.h>

#define SRVPIN 6
//#define SRVPIN 9

//////////////////////////////////////////
/// ultrasonic sensor
//////////////////////////////////////////

class Ultrasonic
{
  public:
    Ultrasonic(int pin);
    void DistanceMeasure(void);
    long microsecondsToCentimeters(void);
  private:
    int _pin;//pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
    long duration;// the Pulse time received;
};
Ultrasonic::Ultrasonic(int pin)
{
  _pin = pin;
}
/*Begin the detection and get the pulse back signal*/
void Ultrasonic::DistanceMeasure(void)
{
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(_pin, LOW);
  pinMode(_pin, INPUT);
  duration = pulseIn(_pin, HIGH);
}
/*The measured distance from the range 0 to 400 Centimeters*/
long Ultrasonic::microsecondsToCentimeters(void)
{
  return duration / 29 / 2;
}

//VH_RADIUS 30 //回転半径30cm
//sqrt(2)*VH_RADIUS = 42.4264068712cm
const long COLLISION_THRES = 42;
//const long COLLISION_THRES = 100;
//ultrasonic instance declared before use
Ultrasonic ultrasonic(7);

boolean Sensor_Detect_Obstacle(void)
{
  long RangeInCentimeters;

  ultrasonic.DistanceMeasure();// get the current signal time;
  RangeInCentimeters = ultrasonic.microsecondsToCentimeters();//convert the time to centimeters

 //   Serial.print("Sensor_Detect_Obstacle :");
 //   Serial.println(RangeInCentimeters);

  if (RangeInCentimeters <= COLLISION_THRES)
  {
    Serial.println("Obstacle detected!");
    return true;
  }
  else
    return false;
}

//////////////////////////////////////////
/// servo
//////////////////////////////////////////

Servo myservo;  // create servo object to control a servo
// a maximum of eight servo objects can be created

//CurPos 0:  0 degrees (left)
//CurPos 1: 45 degrees
//CurPos 2: 90 degrees (center)
//CurPos 3: 135 degrees
//CurPos 4: 180 degrees (right)
const int CurPosMax = 5;
const int CenterPos = 2;
const int CurPosArray[] = {0, 45, 90, 135, 180};

void Sensor_TurnToPos(int CurPos)
{
  myservo.write(CurPosArray[CurPos]);   // tell servo to go to position in variable 'pos'
  delay(1500);                          // waits for the servo to reach the position

  //  Serial.print("Sensor_TurnToPos :");
  //  Serial.println(CurPos);
}

void Vehicle_ResumeFowardMove(void)
{
  Serial.println("Vehicle_ResumeFowardMove :");
}

void Vehicle_StopMove(void)
{
  Serial.println("Vehicle_StopMove :");
}

void Vehicle_TurnToPos(int CurPos)
{
  Serial.print("Vehicle_TurnToPos :");
  Serial.println(CurPos);

}

void setup()
{
  myservo.attach(SRVPIN);// attaches the servo on pin to the servo object
  Serial.begin(9600);
}


void loop()
{
  boolean ObstacleFlag;
  int CurPos;

  Sensor_TurnToPos(CenterPos);
  ObstacleFlag = Sensor_Detect_Obstacle();

  if (ObstacleFlag == true)
  {
    Vehicle_StopMove();
    CurPos = 0;

    while (CurPos < CurPosMax)
    {
      Sensor_TurnToPos(CurPos);
      ObstacleFlag = Sensor_Detect_Obstacle();

      if (ObstacleFlag == false)
      {
        //found a way out
        Sensor_TurnToPos(CenterPos);
        Vehicle_TurnToPos(CurPos);
        CurPos = CurPosMax; //in order to exit from the loop
      }
      else
      {
        //still obstacle exists
        CurPos++;
      }
    }

    //Sweeped right and left but still obstacles all around
    if (ObstacleFlag == true)
    {
      Serial.println("Vehicle will turn 180 and exit :");
      Sensor_TurnToPos(CenterPos);
      Vehicle_TurnToPos(180);
    }

    Vehicle_ResumeFowardMove();
  }

}

