/*
Robotics, Lab 4
The Artificers
Andrew Scibelli, Steve Boucher Marguerite McGahay
Due: 4-22-24 11:55PM 

*/

#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include <time.h>

using namespace Pololu3piPlus32U4;
Encoders encoders;
Buzzer buzzer;
Motors motors;
Servo headServo;

//UltraSonics--------------------------------------------------------------

//intialize UltraSonic 
const int ECHO_PIN = 4; //mine, invert for others 
const int TRIG_PIN = 5;

//ultrasonic maxes 
const int MAX_DISTANCE = 200; // (200cm /2 meters)

//ultrasonic timing
unsigned long usCm;
unsigned long usPm;
const unsigned long US_PERIOD = 100;

//current US distance reading
float distance = 0;

//Head-----------------------------------------------------------------
const int HEAD_SERVO_PIN = 11; //change for andrew 22/margerate/Steve 11
int check = 0;
int checkfrequency = 0;
int importantdistance = 0;

const int checkdefault = 1000;
const int distancedefault = 30;

int headPositions[2];
int headReads[2];

const int HEAD_LEFT_ANGLE = 60;
const int HEAD_CENTER_ANGLE = 90;
const int HEAD_RIGHT_ANGLE = 120;


int counter = 0;
bool returno = false;

// Define the minimum and maximum values for check frequency and important distance
const int minCheckFrequency = 50;    // Minimum allowed check frequency
const int maxCheckFrequency = 1000;  // Maximum allowed check frequency
const int minImportantDistance = 10; // Minimum allowed important distance
const int maxImportantDistance = 200; // Maximum allowed important distance

//-------------------------VARS---------------------------

//--------------------------------------check encoders
unsigned long currentMillis;
unsigned long encoder_prevMillis;
unsigned long calc_prevMillis;
unsigned long check_prevMillis;
unsigned long calc1_prevMillis;
unsigned long calc2_prevMillis;
unsigned long check2_prevMillis;
unsigned long move_prevMillis;
unsigned long print_prevMillis;
unsigned long us_sensor_prvMillis;

//left and right
long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

//wheel measurements
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.531;
const float WHEEL_DIS = 9.0; //possibly 8.2

float Sl = 0.0F;
float Sr = 0.0F;

//timing
const unsigned long ENCODER_PERIOD = 20;
const unsigned long CALC_PERIOD = 20;
const unsigned long CHECK_PERIOD = 20;
const unsigned long MOVE_PERIOD = 20;

//timing bools
bool encoder_done = false;
bool calc_done = false;
bool move_done = true; // to start the loop...
bool check_done = false;

//---------------------------speeds
int MAX_SPEED = 100;
float MIN_SPEED = 40;
float BASE_SPEED = 70;

//--------------------------PID & CALC

//position 

float dS = 0;
float dT = 0;
float dX = 0;
float dY = 0;
float currx = 0;
float curry = 0;
float currtheta = 1.57;

//--------------------------pid
float pidResult = 0;
double kiTotal = 0.0;
double previousError = 0.0;
float targetError = 0;
double proportional = 0;
const double KP = -55.0; // Adjust kp for quicker response
const double KI = 0;     // Reduce ki for less integration
const double KD = 0;     // Increase kd for faster damping

//----------------------------GOALS
const int NUMBER_OF_GOALS = 8;
float xGoals[NUMBER_OF_GOALS] = { 30, 0, -30, 0, -30, 0, 30, 0 };
float yGoals[NUMBER_OF_GOALS] = { 30, 0, -30, 0, 30, 0, -30, 0 };

int goalnum = 0;
float targetX = xGoals[goalnum];
float targetY = yGoals[goalnum];
float targetTheta = 0; //starts at 0 regardless 
float targetDistance = 1000000000000000000000;

//end
bool end = false;
const int margin_of_error = 1.0;

//-------------------------LOOP---------------------------
void setup() {
  Serial.begin(57600);
  
  motors.flipLeftMotor(true);
  motors.flipLeftMotor(true);

  checkfrequency = checkdefault;
  importantdistance = distancedefault;

  //initalize head position to start 
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(headpos);
  
  //initalize ultra sonic
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  Serial.print("STARTING CF");
  Serial.println(checkfrequency);

  headPositions[0] = HEAD_LEFT_ANGLE;
  headPositions[1] = HEAD_CENTER_ANGLE;
  headPositions[2] = HEAD_RIGHT_ANGLE;

  Serial.println("------------------------------------STARTING UP---------------------------------------------");
  buzzer.play("c32");

  delay(1000);
}

void loop() {
  if (!end) {
    //checkEncoders();  //checks the distance  ENCODERS
    //calculatepos(); //CALC  CALC
    //checkTarget(); //CHECK
    //Move(); //MOVE 
    scan();
    //PRINT();
  } else {
    motors.setSpeeds(0, 0);
  }
}

//---------------------METHODS----------------------------

//-------------------------HEAD---------------------------

void scan()  {
  if (counter == 0){
    returno = false; 
    headServo.write(headPositions[0]);
    headReads[0] = averageScan();

  }
  else if ( counter == 3){
    Serial.println("Center");
    headServo.write(headPositions[1]);
    headReads[1] = averageScan();
  }
  else if ( counter == 6){
    Serial.println("Right");
    returno = true; 
    headServo.write(headPositions[2]);
    headReads[2] = averageScan();
  }

  if (returno){
    counter -= 1;
  }
  else{
    counter += 1;
  }
  Serial.println(counter);
}

//-------------------------ENCODERS---------------------------
void checkEncoders() {
  currentMillis = millis();
  if (currentMillis - encoder_prevMillis >= ENCODER_PERIOD && move_done) {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    // Calculate the distance traveled based on the encoder counts
    float distanceLeft = (((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE)*-1);
    float distanceRight = (((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE)*-1);

    // Update the values of Sl and Sr, but not accumulating
    Sl = distanceLeft;
    Sr = distanceRight;

    // Update the previous counts
    prevLeft = countsLeft;
    prevRight = countsRight;

    //reset hard time
    encoder_done = true;
    move_done = false;
    encoder_prevMillis = currentMillis;
  }
}

//-------------------------CALC---------------------------
void calculatepos() {
  currentMillis = millis();
  if (currentMillis - calc_prevMillis >= CALC_PERIOD && encoder_done) {
    float xdiff = targetX - currx;
    float ydiff = targetY - curry;

    dS = (Sr + Sl) / 2;
    dT = (Sl - Sr) / WHEEL_DIS;

    dX = (dS * cos(currtheta + (dT / 2.0)));  //should be cos
    dY = dS * sin(currtheta + (dT / 2.0));     // should be sin

    targetDistance = sqrt(xdiff * xdiff + ydiff * ydiff);
    targetTheta = atan2(ydiff, xdiff);

    currx = currx + dX;
    curry = curry + dY;
    currtheta = currtheta + dT;
    if (currtheta > 3.14) {
      currtheta = -3.14;
    }

    if (currtheta < -3.14) {
      currtheta = 3.14;
    }

    targetError = targetTheta - currtheta;
    proportional = KP * targetError;
    pidResult = proportional;
    calc_done = true;
    encoder_done = false;
    calc_prevMillis = currentMillis;
  }
}

//-------------------------CHECK---------------------------
void checkTarget() {
  currentMillis = millis();
  if (currentMillis - check_prevMillis >= CHECK_PERIOD && calc_done) {
    if (0 - margin_of_error <= targetDistance && targetDistance <= 0 + margin_of_error) {
      motors.setSpeeds(0, 0);
      buzzer.play("c32");
      delay(100);
      buzzer.play("e32");
      delay(100);
      buzzer.play("g32");
      delay(1000);
      goalnum++;
      targetX = xGoals[goalnum];
      targetY = yGoals[goalnum];
    }

    if (goalnum > (NUMBER_OF_GOALS - 1)) {
      buzzer.play("c32");
      delay(400);
      buzzer.play("e32");
      delay(400);
      buzzer.play("g32");
      delay(600);
      buzzer.play("d32");
      delay(600);
      buzzer.play("c33");
      delay(800);
      Serial.println("--------------------------DONE---------------------------");
      end = true;
    }

    check_done = true;
    calc_done = false;
    check_prevMillis = currentMillis;
  }
}

//-------------------------MOVE---------------------------
void Move() {
  currentMillis = millis();
  if (currentMillis - move_prevMillis >= MOVE_PERIOD && check_done) {
    if (targetDistance < 10) {
      BASE_SPEED = constrain(BASE_SPEED / 2, MIN_SPEED, 400);
    } else {
      BASE_SPEED = 70;
    }
    motors.setSpeeds(BASE_SPEED - pidResult, BASE_SPEED + pidResult);
    check_done = false;
    move_done = true;
    move_prevMillis = currentMillis;
  }
}

//--ULTRASONIC
float usReadCm() {
  usCm = millis();
  if (usCm > usPm + US_PERIOD) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 380000);

    distance = duration * 0.034 / 2;
    if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    if (distance == 0) {
    // Serial.println(" Pulse failed!");
      distance = MAX_DISTANCE;
    }
    usPm = usCm;
  }
  return distance;
}

//gets average scan for pid 
float averageScan() {
  float a1 = usReadCm();
  float a2 = usReadCm();
  float a3 = usReadCm();
  float a4 = usReadCm();
  float a5 = usReadCm();
  float a6 = usReadCm();
  float a7 = usReadCm();

  float average = ((a1 + a2 + a3 + a4 + a5 + a6 + a7) / 7);
 // Serial.print("| Average Read: ");
  //Serial.print(average);

  return average;
}

//--------------------------PRINT-----------------------------
void PRINT() {
  currentMillis = millis();
  if (currentMillis - print_prevMillis >= ENCODER_PERIOD) {
    Serial.print("|currY: ");
    Serial.print(curry);
    Serial.print("|currX: ");
    Serial.print(currx);
    Serial.print("|currTheta: ");
    Serial.print(currtheta);
    Serial.print("|TargetANGLE: ");
    Serial.print(targetTheta);
    Serial.print("|Piddiff: ");
    Serial.print(pidResult);
    Serial.print("|distnace: ");
    Serial.print(targetDistance);
    Serial.print("|dT: ");
    Serial.print(dT);
    Serial.print("|Tarx: ");
    Serial.print(targetX);
    Serial.print("|targetY: ");
    Serial.print(targetY);
    Serial.println("");
    print_prevMillis = currentMillis;
  }
}
