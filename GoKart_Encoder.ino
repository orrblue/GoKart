#define clkPin 2 //signal A
#define dtPin 3 //signal B

float encoderVal = 0.0; //encoder val calibrated or set to 0
unsigned long encoderValTime = 1;
float prevEncoderVal = 0.0;
unsigned long prevEncoderValTime = 0;
float rotationSpeed = 0.0;
int cycleCounter = 0;
unsigned long timeSum = 0;
unsigned long avgTime = 0;



void setup()
{
  pinMode(clkPin, INPUT);  //pin numbers as inputs
  pinMode(dtPin, INPUT);   //pin numbers as inputs
  Serial.begin(250000);     //sets data rate, bits/second
}

void loop()
{
  
  prevEncoderValTime = encoderValTime;
  
  int change = getEncoderTurn();
  if(change != 0)
  {
    cycleCounter += 1;
    timeSum += encoderValTime; //sum up time to calculate avg every 10 cycles
    
    prevEncoderVal = encoderVal;
    encoderVal = encoderVal + change;
    if(cycleCounter == 9)
    {
      avgTime = timeSum / 10;
      cycleCounter = 0;
      timeSum = 0;
      Serial.println(avgTime);
    }
    Serial.println(encoderVal);
    //Serial.println(encoderValTime - prevEncoderValTime);
    //Serial.println((encoderVal - prevEncoderVal));
  }

  //if(encoderValTime - prevEncoderValTime != 0 && encoderVal - prevEncoderVal != 0)
    //rotationSpeed = (encoderVal - prevEncoderVal) / (encoderValTime - prevEncoderValTime);
  
  //if(encoderVal - prevEncoderVal != 0)
    //Serial.println((encoderVal - prevEncoderVal));
  
  //Serial.println(rotationSpeed);
  
  //if(encoderValTime - prevEncoderValTime != 0)
    //Serial.println(encoderValTime - prevEncoderValTime);
    
 
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
      if(result != 0)
        encoderValTime = micros();
    }
  }
  oldA = newA;
  oldB = newB;
  return result;
}
