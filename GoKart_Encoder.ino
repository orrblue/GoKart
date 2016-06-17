#include <NewPing.h>
#include <Servo.h>


//ENCODER
#define clkPin 3 //signal B
#define dtPin 2 //signal A

#define ENCODER_A 2
#define ENCODER_B 3

//PROXIMITY SENSOR
//#define TRIGGER_PIN 11
//#define ECHO_PIN 12
//#define MAX_DISTANCE 160
//#define WALL_DISTANCE 100

//MOTOR
//#define CLOCK_PIN 6
//#define COUNTER_CLOCK_PIN 7
#define MAX_ENCODER_VAL 150 //500 is safe
#define MIN_ENCODER_VAL -150 //-500 is safe
#define SERVO_PIN 9

#define TURN_L_PW 1600
#define TURN_R_PW 1380 //1380 vs 1600 is about the same turn speed, but still drifts. 1381 drifts other way
#define NEUTRAL_PW 1500

/* INFORMATION
 * Motor to encoder ratio 1:369
 * 4428 "encoder ticks" per motor revolution
 * leftward rotation = decreasing ticks. 1550 and up
 * rightward rotation = increasing ticks. 1450 and down
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
//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
//long microseconds, cm;
//unsigned long time;

int encoderVal = 0; //encoder val calibrated or set to 0
int encoderValTime = 0;
int prevEncoderVal = 0;
int prevEncoderValTime = 0;
int rotationSpeed = 0;
int cycleCounter = 0;
unsigned long timeSum = 0;
//unsigned long avgTime = 0.0;

int requestedEncoderVal = 0;
bool reachedEndRight = false;
bool reachedEndLeft = true;

volatile int encoderAcc = 0;
volatile byte encoderAState = HIGH;
volatile byte encoderBState = HIGH;

void setup()
{
  pinMode(clkPin, INPUT);  //pin numbers as inputs
  pinMode(dtPin, INPUT);   //pin numbers as inputs
  
  Serial.begin(250000);     //sets data rate, bits/second

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_A),
    risingALine,
    RISING
  );
  attachInterrupt(
    digitalPinToInterrupt(ENCODER_A),
    fallingALine,
    FALLING
  );
  attachInterrupt(
    digitalPinToInterrupt(ENCODER_B),
    risingBLine,
    RISING
  );
  attachInterrupt(
    digitalPinToInterrupt(ENCODER_B),
    fallingBLine,
    FALLING
  );
  
  servo.attach(SERVO_PIN);
  stopMotor();
  delay(2500);
  turnMotorRight();
}

void loop()
{
  //Proximity Sensor
//  digitalWrite(ECHO_PIN, LOW); 
//  microseconds = sonar.ping();
//  cm = sonar.convert_cm(microseconds);  
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
    //Serial.print("  Enc: " );   
    
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
    //Serial.println(encoderValTime - prevEncoderValTime);
    //Serial.println((encoderVal - prevEncoderVal));
  }

  Serial.println(encoderVal);

  //requestedEncoderVal = convertDistanceToTicks(cm);

  if (encoderVal >= MAX_ENCODER_VAL && !reachedEndRight) {
    stopMotor();
    reachedEndRight = true;
    reachedEndLeft = false;
    delay(500);
    turnMotorLeft();
    Serial.println("FIRST IF");
  } else if (encoderVal <= MIN_ENCODER_VAL && !reachedEndLeft) {
    stopMotor();
    reachedEndRight = false;
    reachedEndLeft = true;
    delay(500);
    turnMotorRight();
    Serial.println("SECOND IF");
  }
}

void risingALine(void) {
  encoderAState = HIGH;
}

void fallingALine(void) {
  encoderAState = LOW;

  if (encoderBState == HIGH)
    ++encoderAcc;
  else
    --encoderAcc;
}

void risingBLine(void) {
  encoderBState = HIGH;
}

void fallingBLine(void) {
  encoderBState = LOW;
}

int getEncoderTurn(void) {
  int ret = encoderAcc;
  encoderAcc = 0;

  return ret;
}

/*int getEncoderTurn(void)
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
}*/


/*int convertDistanceToTicks(int centimeters)
{
  return -16*(centimeters-100); //Change sign if wrong direction
}*/

void stopMotor()
{
  servo.writeMicroseconds(NEUTRAL_PW);
}

void turnMotorLeft()
{
  servo.writeMicroseconds(TURN_L_PW);
}

void turnMotorRight()
{
  servo.writeMicroseconds(TURN_R_PW);
}

