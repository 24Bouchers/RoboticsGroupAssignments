/*
Robotics, Lab 5
The Artificers
Andrew Scibelli, Steve Boucher Marguerite McGahay
Due: 4-6-24 11:55PM 

//expiermental 
//this version has some obstacle avoidance but im unsure of how to deal with it 
*/

//---------------------------------------------------------------------VARIABLE-----------------------------------------------------------------------//

#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include <time.h>

using namespace Pololu3piPlus32U4;
Encoders encoders;
Buzzer buzzer;
Motors motors;
Servo headServo;


//-------------------------ULTRASONICS---------------------------
//intialize UltraSonic
const int ECHO_PIN = 5;  //mine, invert for others
const int TRIG_PIN = 4;
//ultrasonic maxes
const int MAX_DISTANCE = 200;  // (200cm /2 meters)
//ultrasonic timing
unsigned long usCm;
unsigned long usPm;
const unsigned long US_PERIOD = 100;
//current US distance reading
float distance = 0;

//-------------------------SERVO---------------------------
const int HEAD_SERVO_PIN = 22;  //change for andrew 22/margerate/Steve 11
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
const int minCheckFrequency = 50;      // Minimum allowed check frequency
const int maxCheckFrequency = 1000;    // Maximum allowed check frequency
const int minImportantDistance = 10;   // Minimum allowed important distance
const int maxImportantDistance = 200;  // Maximum allowed important distance
int currAngle = 0;
int counter = 90;
bool returno = false;

//-------------------------CHECK ENCODERS---------------------------
unsigned long currentMillis;
unsigned long encoder_prevMillis;
unsigned long calc_prevMillis;
unsigned long check_prevMillis;
unsigned long calc1_prevMillis;
unsigned long calc2_prevMillis;
unsigned long check2_prevMillis;
unsigned long move_prevMillis;
unsigned long print_prevMillis;
unsigned long servo_prevMillis;
unsigned long avoid_prevMillis;
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
float pidResult = 0;
double kiTotal = 0.0;
double previousError = 0.0;
float error = 0;
double proportional1 = 0;
const double KP1 = -50.0;  // Adjust kp for quicker response, 30 works
const double KI = 0;       // Reduce ki for less integration
const double KD = 0;       // Increase kd for faster damping

//-------------------------TIMING---------------------------
const unsigned long ENCODER_PERIOD = 20;
const unsigned long CALC_PERIOD = 20;
const unsigned long CHECK_PERIOD = 20;
const unsigned long MOVE_PERIOD = 20;
const unsigned long SCAN_PERIOD = 60;
//timing bools  //testing with true
bool encoder_done = false;  //f
bool calc_done = false;     //f
bool move_done = true;      // to start the loop...
bool check_done = false;    //f
bool servo_done = false;// should start false, for testing
bool average_done = false;

//-------------------------Speeds---------------------------
int MAX_SPEED = 100;
float MIN_SPEED = 30;
float BASE_SPEED = 50;
float leftSpeed = 0;
float rightSpeed = 0;
//-------------------------GOALS AND POSITION ---------------------------
const int NUMBER_OF_GOALS = 5;
float xGoals[NUMBER_OF_GOALS] = { 0, 30, -30, -30, 0 };
float yGoals[NUMBER_OF_GOALS] = { 60, -30, 30, -30, 0 };

int goalnum = 0;
float targetX = xGoals[goalnum];
float targetY = yGoals[goalnum];
float targetTheta = 0;  //starts at 0 regardless
float targetDistance = 1000000000000000000000;

//end
bool end = false;
const int margin_of_error = 1.0;
//position
float dS = 0;
float dT = 0;
float dX = 0;
float dY = 0;
float currx = 0;
float curry = 0;
float currtheta = 1.57;  //90 degrees

//-------------------------AVOIDING---------------------------


int KPfront = -4;
int KPright = -2;
int KPleft = -2;
float prev_proportional_front = 0;
float prev_proportional_left = 0;
float prev_proportional_right = 0;
float error_angle = 0;
float desired_distance = 10;
float measured_distance_front = 0;
float measured_distance_left = 0;
float measured_distance_right = 0;
float proportional_front = 0;
float proportional_left = 0;
float proportional_right = 0;
float avoidance_speed = 0;

//--------------------------------------------------------------------LOOP-----------------------------------------------------------------------//
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

  Serial.println("------------------------------------STARTING UP---------------------------------------------");
  buzzer.play("c32");
  delay(1000);
}
void loop() {
  if (!end) {

    servo();          //scan
    usReadCm();       //
    checkEncoders();  //checks the distance  ENCODERS
    calculatepos();   //CALC  CALC
    checkTarget();    //CHECK
    Move();           //MOVE

    PRINT();
  }  //if
  else {
    motors.setSpeeds(0, 0);
  }  //else
}  //loop
//---------------------------------------------------------------------METHODS-----------------------------------------------------------------------//


//-------------------------SERVO---------------------------
void servo() {
  currentMillis = millis();
  if (currentMillis - servo_prevMillis >= SCAN_PERIOD && move_done) {

    headServo.write(counter);
    currAngle = counter;
    if (counter == 60) {
      returno = false;

    } else if (counter == 120) {
      returno = true;
    }

    if (returno && counter > -1) {
      counter = counter - 30;
    } else {
      counter += 30;
    }
    servo_done = true;
    move_done = false;
    servo_prevMillis = currentMillis;
  }  //timer
}  //Scan
//-------------------------USREAD---------------------------
void usReadCm() {
  //Serial.print("Starting");
  usCm = millis();
  if (usCm > usPm + US_PERIOD && servo_done) {
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
    distance = duration * 0.034 / 2;  // time of flight equation, speed of sound wave divided by 2
    //apply limits
    if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    if (distance == 0) {
      Serial.println(" Pulse failed!");
      distance = MAX_DISTANCE;
    }
    if (currAngle > 90) {
      measured_distance_right = distance;
    } else if (currAngle < 90) {
      measured_distance_left = distance;
    } else {
      measured_distance_front = distance;
    }

    //update the prevmillis
    usPm = usCm;
    average_done = true;
    servo_done = false;
  }  //if
  //  return distance;
  //Serial.print("distance: ");
  //Serial.println(distance);
}  // usReadcm

//-------------------------CALC---------------------------
void calculatepos() {
  currentMillis = millis();
  if (currentMillis - calc_prevMillis >= CALC_PERIOD && encoder_done) {
    float xdiff = targetX - currx;
    float ydiff = targetY - curry;

    dS = (Sr + Sl) / 2;
    dT = (Sl - Sr) / WHEEL_DIS;  // forward would be sr - sl

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
    //Ki get intergral correction
    //kiTotal += error;
    //double integral = KI * kiTotal;
    //Kd derivative
    //float derivative = KD * (error - previousError);
    //previousError = error;
    //sum
    if (measured_distance_front < 20) {
      error_angle = desired_distance - measured_distance_front;
      proportional_front = KPfront * error_angle;
    }  //
    else {
      proportional_front = prev_proportional_front;
    }
    if (measured_distance_left < 15) {
      error_angle = desired_distance - measured_distance_left;
      proportional_left = KPleft * error_angle * -1;
    }  //
    else {
      proportional_left = prev_proportional_left;
    }
    if (measured_distance_right < 15) {
      error_angle = desired_distance - measured_distance_right;
      proportional_right = KPright * error_angle;
    }  //
    else {
      proportional_right = prev_proportional_right;
    }
  




    avoidance_speed = proportional_front + proportional_left + proportional_right;  //+intergal + derivative;
    //"saving" older values, the /2 two so that way its not constant, need a better fix for it thou 
    prev_proportional_front = proportional_front/2;
    prev_proportional_left = proportional_left/2;
    prev_proportional_right = proportional_right/2;
    pidResult = proportional1 + avoidance_speed;  //+intergal + derivative;
    //pidResult = constrain(pidResult, -3.14, 3.14);
    //apply the sum to the motors, one will be +pidSUm, the other -pidSum
    calc_done = true;
    encoder_done = false;
    calc_prevMillis = currentMillis;
  }  //timer
}  //calculate pos

//-------------------------ENCODERS---------------------------
void checkEncoders() {
  currentMillis = millis();
  if (currentMillis - encoder_prevMillis >= ENCODER_PERIOD && average_done) {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    // Calculate the distance traveled based on the encoder counts
    float distanceLeft = (((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE) * -1);     //for backwards
    float distanceRight = (((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE) * -1);  //for backwards

    // Update the values of Sl and Sr, but not accumulating
    Sl = distanceLeft;
    Sr = distanceRight;

    // Update the previous counts
    prevLeft = countsLeft;
    prevRight = countsRight;
    //reset hard time
    encoder_done = true;
    average_done = false;
    encoder_prevMillis = currentMillis;
  }  //timer
}  //encoders

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
    motors.setSpeeds(leftSpeed, rightSpeed);
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
    Serial.print("|Base_speed: ");
    Serial.print(BASE_SPEED);
    Serial.print("|avoid speed: ");
    Serial.print(avoidance_speed);
    Serial.println("");
    print_prevMillis = currentMillis;
  }  //timer
}  //print
