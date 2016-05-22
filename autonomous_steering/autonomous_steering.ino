
#include <NewPing.h>

#define TARGET_DISTANCE             100                             // centimeters 
#define SENSOR_2_OFFSET             24                              // distance from the Kart midline to the front sensor
#define SENSOR_2_ECHO               [TODO]
#define SENSOR_2_TRIGGER            [TODO]
#define SENSOR_1_OFFSET             [TODO]
#define SENSOR_1_ECHO               [TODO]
#define SENSOR_1_TRIGGER            [TODO]
#define SENSOR_DISTANCE             [TODO]                          // horizontal distance between sensors on the side of the Kart
#define MAX_DISTANCE                200                             // maximum distance for the sonar sensors, in centimeters
#define K                           ( ( 360.0 / 100.0 ) * [TODO] )  // magic constant to convert from distances to angles
#define TICKS_PER_DEGREE            (800.0 / 65.0)                  // 800 ticks on the steering encoder gives us 65 degrees of turning

unsigned int L1, L2, wall_distance;
double diff, headingAngle, steering;
NewPing sensor1(SENSOR_1_TRIGGER, SENSOR_1_ECHO, MAX_DISTANCE),
        sensor2(SENSOR_2_TRIGGER, SENSOR_2_ECHO, MAX_DISTANCE);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
}

void loop() {
 restart:
  // read L1
  digitalWrite(SENSOR_1_ECHO, LOW);
  ms = sensor1.ping();
  L1 = sensor1.convert_cm();
  // if we have a problem, try again
  if (L1 > MAX_DISTANCE)
    goto restart;

  // read L2
  digitalWrite(SENSOR_2_ECHO, LOW);
  ms = sensor2.ping();
  L2 = sensor2.convert_cm();
  // if we have a problem, try again
  if (L2 > MAX_DISTANCE)
    goto restart;

  if ( L1 == L2 )
    diff = 90;
  else
    diff = atan( (double) SENSOR_DISTANCE / ( L1 - L2 ) );

  headingAngle = diff - 90;

  // compute distance from wall;
  wall_distance = sin( diff ) * L2;

  error_distance = wall_distance - TARGET_DISTANCE;

  steering = K * error_distance - headingAngle;

  // do steering [TODO]
  
}
