#include <NewPing.h>
#include <Servo.h>

// ENCODER
#define clkPin 2 //signal A
#define dtPin 3 //signal B

//PROXIMITY SENSOR
#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define SERVO_PIN 9
#define MAX_DISTANCE 160
#define WALL_DISTANCE 100



/* INFORMATION
 * Motor to encoder ratio 1:369
 * 4428 "encoder ticks" per motor revolution
 * Clockwise rotation = decreasing ticks
 * CounterClockWise rotation = increasing ticks
 * Steering has about 135 degree turn -> 67 degrees each way
 * Turning steering shaft ~65 degrees is 800 ticks
 * 
 */

/* TODO: Move motor to certain angle based on distance from target.
 *  Assume kart is following wall on its right.
 *  Assume kart is to be 1.0m away from the wall
 *  Assume maximum distance away from wall is 1.5m
 *  Assume minimum distance away from wall is 0.5m
 * 
 *  At 1.50m ("max distance") left of target, turn -800
 * 
 */

Servo servo;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
long microseconds, cm;
unsigned long time;

int encoderVal = 0; //encoder val calibrated or set to 0
int encoderValTime = 0;
int prevEncoderVal = 0;
int prevEncoderValTime = 0;
int rotationSpeed = 0;
int cycleCounter = 0;
unsigned long timeSum = 0;
//unsigned long avgTime = 0.0;

int requestedEncoderVal = 0;



void setup()
{
  pinMode(clkPin, INPUT);  //pin numbers as inputs
  pinMode(dtPin, INPUT);   //pin numbers as inputs
  Serial.begin(250000);     //sets data rate, bits/second
  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(1500);
}

void loop()
{
  //Proximity Sensor
//  digitalWrite(ECHO_PIN, LOW); 
  microseconds = sonar.ping();
//  cm = sonar.convert_cm(microseconds);

  turnMotor();
  
  //prevEncoderValTime = encoderValTime;
  
  int change = getEncoderTurn();
  if(change != 0)
  {
    /*
    cycleCounter += 1;
    sum up time to calculate avg every 10 cycles
    timeSum += encoderValTime - prevEncoderValTime;
    prevEncoderVal = encoderVal;
    */
    
    encoderVal = encoderVal + change;
    
    /*
    if(cycleCounter == 9)
    {
      ===Rotations / Time * Multiplier===
      rotationSpeed = (1000000 / timeSum);
      Serial.println(rotationSpeed);
      timeSum = 0.0;
      cycleCounter = 0;
    }
    */
    
    //Printable things:
    Serial.println(encoderVal);
    //Serial.println(encoderValTime - prevEncoderValTime);
    //Serial.println((encoderVal - prevEncoderVal));
  }

  //requestedEncoderVal = convertDistanceToTicks(cm);
  //
  //while encoder val != requestedEncoderVal
  //  if encoderVal < requestedEncoderVal
  //    turn motor counterClockWise, increase encoder val
  //  else
  //    turn motor clockWise, decrease encoder val
    
    //Serial.println(change);


}

int getEncoderTurn(void)
{
  static int oldA = HIGH;   //oldA = 1
  static int oldB = HIGH;   //oldB = 1
  int result = 0;
  int newA = digitalRead(clkPin);  // see if signal A is high or low
  int newB = digitalRead(dtPin);   // see if signal B is high or low
  if (newA != oldA || newB != oldB)  // if either A or B have a low voltage
  {
    // something has changed
    if (oldA == HIGH && newA == LOW)  // if A dropped from 1 to 0
    {
      result = (oldB * 2 - 1);
      //if(result != 0)
        //encoderValTime = micros();
    }
  }
  oldA = newA;
  oldB = newB;
  return result;
}


int convertDistanceToTicks(int centimeters)
{
  return -16*(centimeters-100);
}

void turnMotor()
{
  servo.writeMicroseconds(1500);
}

