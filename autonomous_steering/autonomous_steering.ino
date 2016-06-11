
#include <math.h>
#include <NewPing.h>

// centimeters
#define TARGET_DISTANCE             100
// distance from the Kart midline to the front sensor
#define SENSOR_2_OFFSET             0

// front sensor echo pin number
#define SENSOR_2_ECHO               12

// front sensor trigger pin number
#define SENSOR_2_TRIGGER            11

// distance from the rear sensor to the kart midline
#define SENSOR_1_OFFSET             0

// rear sensor echo pin number
#define SENSOR_1_ECHO               9

// rear sensor trigger pin number
#define SENSOR_1_TRIGGER            8

// horizontal distance between sensors on the side of the Kart
#define SENSOR_DISTANCE             120

// maximum distance for the sonar sensors, in centimeters
#define MAX_DISTANCE                200

// heading angle beyond which the sonar start to bug out (theoretically 27, but braindamage is bad)
#define MAX_HEADING                 20.0

// maximum steering angle tolerance for the Kart (each way) (theoretically 67, but it's stupid to blow out motors)
#define MAX_STEERING                60.0

// magic constant to convert from distances to angles
#define K                           ( -(degrees2radians(MAX_STEERING - MAX_HEADING))  / 100.0 )

// 800 ticks on the steering encoder gives us 65 degrees of turning
#define TICKS_PER_DEGREE            (800.0 / 65.0)
#define radians2degrees(a)          ((360.0 / (2 * M_PI)) * (a))
#define degrees2radians(a)          (((2*M_PI) / 360.0) * (a))

int L1, L2, wall_distance, ms, error_distance;
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
  L1 = sensor1.convert_cm(ms);
  // if we have a problem, try again
  if (L1 > MAX_DISTANCE) {
    Serial.println("Error reading from sensor 1");
    goto restart;
  }

  // read L2
  digitalWrite(SENSOR_2_ECHO, LOW);
  ms = sensor2.ping();
  L2 = sensor2.convert_cm(ms);
  // if we have a problem, try again
  if (L2 > MAX_DISTANCE) {
    Serial.println("Error reading from sensor 2");
    goto restart;
  }

  Serial.print("L1: ");
  Serial.print(L1);
  Serial.print("cm\t");
  Serial.print("L2: ");
  Serial.print(L2);
  Serial.print("cm\n");

  if ( L1 == L2 )
    diff = (M_PI/2);
  else
    diff = atan2( (double) SENSOR_DISTANCE, L1 - L2 );

  headingAngle = diff - (M_PI/2);

  // compute distance from wall;
  wall_distance = sin( diff ) * L2;

  error_distance = wall_distance - TARGET_DISTANCE;

  steering = K * error_distance - headingAngle;

  Serial.print("da: ");
  Serial.print(wall_distance);
  Serial.print("cm\t");
  Serial.print("de: ");
  Serial.print(error_distance);
  Serial.print("cm\n");
  Serial.print("theta_h: ");
  Serial.print(radians2degrees(headingAngle));
  Serial.print(" degrees\n");
  Serial.print("'theta_s' (bogus): ");
  Serial.print(radians2degrees(steering));
  Serial.print(" degrees\n");

  // do steering [TODO]

  delay(250);
}
