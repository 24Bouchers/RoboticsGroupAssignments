/*
Robotics, Lab 4
The Artificers
Andrew Scibelli, Steve Boucher Marguerite McGahay
Due: 4-22-24 11:55PM 

//PRIOVATE VERSION */

#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include <time.h>

using namespace Pololu3piPlus32U4;
Encoders encoders;
Buzzer buzzer;
Motors motors;
Servo headServo;


//-------------------------VARS---------------------------
//-------------------------uLTRA sERVO


//intialize UltraSonic 
const int ECHO_PIN = 5; //mine, invert for others 
const int TRIG_PIN = 4;

//ultrasonic maxes 
const int MAX_DISTANCE = 200; // (200cm /2 meters)

//ultrasonic timing
unsigned long usCm;
unsigned long usPm;
const unsigned long US_PERIOD = 100;

//current US distance reading
float distance = 0;

//Head-----------------------------------------------------------------
const int HEAD_SERVO_PIN = 22; //change for andrew 22/margerate/Steve 11
int check = 0;
int checkfrequency = 0;
int importantdistance = 0;

const int checkdefault = 1000;
const int distancedefault = 30;

const int defaulthead = 90;
const int fronthead = 90;
int headpos = 90;
bool front = false;
int check2 = 0;
int frontScan = 90;


// Define the minimum and maximum values for check frequency and important distance
const int minCheckFrequency = 50;    // Minimum allowed check frequency
const int maxCheckFrequency = 1000;  // Maximum allowed check frequency
const int minImportantDistance = 10; // Minimum allowed important distance
const int maxImportantDistance = 200; // Maximum allowed important distance


//STEVE TIME

int headPositions[3];
int headReads[3];


//left goes wayy further
const int HEAD_LEFT_ANGLE = 50;
const int HEAD_CENTER_ANGLE = 90;
const int HEAD_RIGHT_ANGLE = 130;
int currAngle = 0; 


int counter = 0;
bool returno = false;



//-------------------------uLTRA sERVO
//------------------------move
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
unsigned long scan_prevMillis;


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
const float WHEEL_DIS = 9.0;  //possibly 8.5


float Sl = 0.0F;
float Sr = 0.0F;

//timing
const unsigned long ENCODER_PERIOD = 20;
const unsigned long CALC_PERIOD = 20;
const unsigned long CHECK_PERIOD = 20;
const unsigned long MOVE_PERIOD = 20;
const unsigned long SCAN_PERIOD = 50; 

//timing bools  //testing with true
bool encoder_done = false;  //f
bool calc_done = false;     //f
bool move_done = true;      // to start the loop...
bool check_done = false;    //f

//---------------------------speeds
int MAX_SPEED = 100;
float MIN_SPEED = 40;
float BASE_SPEED = 70;
float leftSpeed = 0; 
float rightSpeed =0; 

//--------------------------PID & CALC

//position

float dS = 0;
float dT = 0;
float dX = 0;
float dY = 0;
float currx = 0;
float curry = 0;
float currtheta = 1.57;//90 degrees

//--------------------------pid
float pidResult = 0;
double kiTotal = 0.0;
double previousError = 0.0;
float error = 0;
double proportional1 = 0;
const double KP1 = -55.0;  // Adjust kp for quicker response, 30 works
const double KI = 0;     // Reduce ki for less integration
const double KD = 0;     // Increase kd for faster damping

//----------------------------GOALS
const int NUMBER_OF_GOALS = 5;
float xGoals[NUMBER_OF_GOALS] = { 30, 30, -30, -30, 0 };
float yGoals[NUMBER_OF_GOALS] = { 30, -30, 30, -30, 0 };

int goalnum = 0;
float targetX = xGoals[goalnum];
float targetY = yGoals[goalnum];
float targetTheta = 0;  //starts at 0 regardless
float targetDistance = 1000000000000000000000;

//end
bool end = false;
const int margin_of_error = 1.0;


//------------OBSTACLE AVOIDANCE

int KP2 = -8; 

float error_angle = 0; 
float desired_distance = 20;
float measured_distance = 0;
float proportional2 = 0;
//-------------------------LOOP---------------------------
void setup() {
  Serial.begin(57600);
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

    headpos = defaulthead;
  checkfrequency = checkdefault;
  importantdistance = distancedefault;

  //initalize head position to start 
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(90);
  
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

    servo(); //scan 
     measured_distance = averageScan();
    checkEncoders();  //checks the distance  ENCODERS
    calculatepos();   //CALC  CALC
    checkTarget();    //CHECK
    Move();           //MOVE

    //PRINT();
  }  //if
  else {
    motors.setSpeeds(0, 0);
  }  //else
}  //loop
//---------------------METHODS----------------------------
//-------------------------CALC---------------------------
void calculatepos() {
  currentMillis = millis();
  if (currentMillis - calc_prevMillis >= CALC_PERIOD && encoder_done) {
    float xdiff = targetX - currx;
    float ydiff = targetY - curry;

    dS = (Sr + Sl) / 2;
    dT = (Sl - Sr) / WHEEL_DIS;// forward would be sr - sl 

    //the first dT in these may need to be current theta, but IDK
    dX = (dS * cos(currtheta + (dT / 2.0)));  //should be cos
    dY = dS * sin(currtheta + (dT / 2.0));    // should be sin

    // Calculate the difference between current position and target position in x and y directions
    targetDistance = sqrt(xdiff * xdiff + ydiff * ydiff);

    targetTheta = atan2(ydiff, xdiff);


    //update the values
    currx = currx + dX;
    curry = curry + dY;


    currtheta = currtheta + dT;
    if (currtheta > 3.14) {
      currtheta = -3.14;
    }  //if its too big

    if (currtheta < -3.14) {
      currtheta = 3.14;
    }  //if its too big

    //PID THINGS

    error = targetTheta - currtheta;
    //Kp get the proportional correction
    proportional1 = KP1 * error;

    if (measured_distance < 50)
    {

    error_angle = desired_distance - measured_distance;

    proportional2 = KP2 * error_angle;
    }//
    else 
    {
      proportional2 =0; 
    }


    if (currAngle < 90)

    {proportional2 = proportional2 * -1;}//if

    //Ki get intergral correction
    //kiTotal += error;
    //double integral = KI * kiTotal;
    //Kd derivative
    //float derivative = KD * (error - previousError);
    //previousError = error;
    //sum
    pidResult = proportional1 + proportional2;  //+intergal + derivative;
    //pidResult = constrain(pidResult, -3.14, 3.14);
    //apply the sum to the motors, one will be +pidSUm, the other -pidSum
    calc_done = true;
    encoder_done = false;
    calc_prevMillis = currentMillis;
  }  //timer
}  //calculate pos
//-------------------------CHECK---------------------------

void checkTarget() {
  currentMillis = millis();
  if (currentMillis - check_prevMillis >= CHECK_PERIOD && calc_done) {
    //we can just (theorically) check if the distance is within -2 - 2
    if (0 - margin_of_error <= targetDistance && targetDistance <= 0 + margin_of_error) {
      motors.setSpeeds(0, 0);
      BASE_SPEED = 70;
      buzzer.play("c32");
      delay(100);
      buzzer.play("e32");
      delay(100);
      buzzer.play("g32");
      delay(1000);
      // Serial.print("--------------------------CHECKPOINT #");
      //Serial.print(goalnum + 1);
      //Serial.println("--------------------------");
      goalnum++;
      //reassing
      targetX = xGoals[goalnum];
      targetY = yGoals[goalnum];

    }  //if

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
    }  //if
    //updating time
    check_done = true;
    calc_done = false;
    check_prevMillis = currentMillis;
  }  //timer
}  //check target


//-------------------------SEVRO
void servo()  {
  currentMillis = millis();
  if (currentMillis - scan_prevMillis >= SCAN_PERIOD) { 

  if (counter == 0){
    returno = false; 
    headServo.write(headPositions[0]);
    currAngle = headPositions[0];
    
  }
  else if (counter == 30){
    headServo.write(headPositions[1]);
    currAngle = headPositions[1];
    }
  else if ( counter == 60){
    returno = true; 
    headServo.write(headPositions[2]);
    currAngle = headPositions[2];
    }

  if (returno && counter > -1){
    counter = counter -1;
  }
  else{
    counter += 1;
  }
  
  
  Serial.print  ("Counter: " );
  Serial.println (counter);
  /* Serial.print  ("Head 0: " );
  Serial.print (headReads[0]);
  Serial.print  ("Head1:");
  Serial.print (headReads[1]);
  Serial.print  ("Head2:");
  Serial.println (headReads[2]);
  */
  scan_prevMillis = currentMillis;
  }  //timer
}//Scan 

float usReadCm() {
  //Serial.print("Starting");

  usCm = millis();
  if (usCm > usPm + US_PERIOD) {
    //clears the trig_pin (set low)
    digitalWrite(TRIG_PIN, LOW); 
    delayMicroseconds(2);

    //Set the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH); 
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW); 

    //Reads the ECHO_PIN returns the sound wave travel time in microseconds
    //note the duration (380000 microseconds) that will allow for reading up max distance supported by the sensor
    long duration = pulseIn(ECHO_PIN, HIGH, 380000);
    
    //Calculating the distance 
    distance = duration * 0.034 / 2; // time of flight equation, speed of sound wave divided by 2
    //apply limits
    if (distance > MAX_DISTANCE) distance = MAX_DISTANCE; 
    if (distance == 0){ 
    Serial.println (" Pulse failed!"); 
    distance = MAX_DISTANCE;
    }
    //update the prevmillis
    usPm = usCm; 
  }//if 
  return distance;
  //Serial.print("distance: ");
  //Serial.println(distance);
}// usReadcm

float averageScan() {
  float a1 = usReadCm();
  float a2 = usReadCm();
  float a3 = usReadCm();
  float a4 = usReadCm();
  float a5 = usReadCm();
  float a6 = usReadCm();
  float a7 = usReadCm();

  float average = ((a1 + a2 + a3 + a4 + a5 + a6 + a7) / 7);
  Serial.print("| Average Read: ");
  Serial.println (average);
 
  return average;



}//averageScan

//--------------------------SCAN
//---------------------------MOVEMENT
//-------------------------ENCODERS---------------------------
void checkEncoders() {
  currentMillis = millis();
  if (currentMillis - encoder_prevMillis >= ENCODER_PERIOD && move_done) {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    // Calculate the distance traveled based on the encoder counts
    float distanceLeft = (((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE)*-1);//for backwards
    float distanceRight = (((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE)*-1);//for backwards

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
  }  //timer
}  //encoders

//-------------------------MOVE---------------------------
void Move() {
  currentMillis = millis();
  if (currentMillis - move_prevMillis >= MOVE_PERIOD && check_done) {

    //Serial.println("DOING MOVE");
    //slowing down, but not slower than min speed
    if (targetDistance < 10) {
      BASE_SPEED = constrain(BASE_SPEED / 2, MIN_SPEED, MAX_SPEED);
    }  //i

    else {
      BASE_SPEED = 70;
    }
    leftSpeed = BASE_SPEED - pidResult;
    rightSpeed = BASE_SPEED + pidResult;
    motors.setSpeeds(leftSpeed,rightSpeed );
    check_done = false;
    move_done = true;
    move_prevMillis = currentMillis;
  }  //timer
}  //move

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
    //Serial.print("|Lmes: ");
    //Serial.print(Sl);
    //Serial.print("|Rmes: ");
    //Serial.print(Sr);
    Serial.print("|Piddiff: ");
    Serial.print(pidResult);
    Serial.print("|distnace: ");
    Serial.print(targetDistance);
    Serial.print("|LSpeed: ");
    Serial.print(leftSpeed);
    Serial.print("|RSpeed: ");
    Serial.print(rightSpeed);
    Serial.print("|dT: ");
    Serial.print(dT);
    Serial.print("|dX: ");
    Serial.print(dX);
    Serial.print("|dY: ");
    Serial.print(dY);
    Serial.print("|Tarx: ");
    Serial.print(targetX);
    Serial.print("|targetY: ");
    Serial.print(targetY);
    //Serial.print("|dS: ");
    //Serial.print(dS);
    Serial.println("");
    Serial.print("|Base_speed: ");
    Serial.print(BASE_SPEED);
    print_prevMillis = currentMillis;
  }  //timer
}  //print
