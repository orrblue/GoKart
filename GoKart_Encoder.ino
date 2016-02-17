#define clkPin 2 //signal A
#define dtPin 3 //signal B

int encoderVal = 0; //encoder val calibrated or set to 0
int encoderValTime = 0;
int prevEncoderVal = 0;
int prevEncoderValTime = 0;
int rotationSpeed = 0;
int cycleCounter = 0;
unsigned long timeSum = 0;
//unsigned long avgTime = 0.0;



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
    timeSum += encoderValTime - prevEncoderValTime; //sum up time to calculate avg every 10 cycles
    
    prevEncoderVal = encoderVal;
    encoderVal = encoderVal + change;
    if(cycleCounter == 9)
    {
      //rotations / time * multiplier
      rotationSpeed = (1000000 / timeSum);
      //Serial.println(rotationSpeed);
      //Serial.println("SUPPPPPPPPPPPP");
      timeSum = 0.0;
      cycleCounter = 0;
    }
    
    //Printable things:
    //Serial.println(encoderVal);
    //Serial.println(encoderValTime - prevEncoderValTime);
    //Serial.println((encoderVal - prevEncoderVal));
  }

  //else
    //Serial.println("Hi ******************");

    Serial.println(encoderVal);
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
