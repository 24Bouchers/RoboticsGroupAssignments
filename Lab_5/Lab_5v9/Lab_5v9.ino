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
//current US distance reading
float distance = 0;

//-------------------------SERVO---------------------------
const int HEAD_SERVO_PIN = 22;  //change for andrew 22/margerate/Steve 11
int check = 0;
const int defaulthead = 90;
// Define the minimum and maximum values for check frequency and important distance
int currAngle = 0;
int counter = 90;
bool returno = false;

//-------------------------CHECK ENCODERS---------------------------
unsigned long currentMillis;
unsigned long encoder_prevMillis;
unsigned long calc_prevMillis;
unsigned long check_prevMillis;
unsigned long move_prevMillis;
unsigned long print_prevMillis;
unsigned long servo_prevMillis;
unsigned long us_prevMillis;

//left and right
long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;
//wheel measurements
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2; //TODO Verify if I can remove
const float WHEEL_CIRCUMFERENCE = 10.531;
const float WHEEL_DIS = 12;  //possibly 8.5
float Sl = 0.0F;
float Sr = 0.0F;
float pidResult = 0;
float error = 0;
double proportional1 = 0;
const double KP1 = -90.0;  // Adjust kp for quicker response, 30 works

//-------------------------TIMING---------------------------
const unsigned long ENCODER_PERIOD = 20;
const unsigned long CALC_PERIOD = 20;
const unsigned long CHECK_PERIOD = 20;
const unsigned long MOVE_PERIOD = 20;
const unsigned long SCAN_PERIOD = 300;
const unsigned long US_PERIOD = 100;
//timing bools  //testing with true
bool encoder_done = false;  // false;  //f
bool calc_done = false;     // false;     //f
bool move_done = true;      // to start the loop...
bool check_done = false;    //f
bool servo_done = false;    // should start false, for testing
bool us_done = true;

//-------------------------Speeds---------------------------
int MAX_SPEED = -250;
float MIN_SPEED = 40;
float BASE_SPEED = 50;
float leftSpeed = 0;
float rightSpeed = 0;
//-------------------------GOALS AND POSITION ---------------------------
const int NUMBER_OF_GOALS = 1;

float xGoals[NUMBER_OF_GOALS] = {100};
float yGoals[NUMBER_OF_GOALS] = {400};

/*LOCALIZATION TEST
float xGoals[NUMBER_OF_GOALS] = { 30, 30, -30, -30, 0 };
float yGoals[NUMBER_OF_GOALS] = { 30, -30, 30, -30, 0 };
*/

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

int KPfront = -3;
int KPright = -2;
int KPleft = -2;
int KP_far_right = -1.5;
int KP_far_left = -1.5;

float prev_proportional_front;
float prev_proportional_left;
float prev_proportional_right;  // testing these as unsinged,
float prev_proportional_far_left;
float prev_proportional_far_right;  // testing these as unsinged,

float error_angle = 0;
float desired_distance = 35;
float measured_distance_front = 0;
float measured_distance_left = 0;
float measured_distance_far_left = 0;
float measured_distance_right = 0;
float measured_distance_far_right = 0;

float important_front = 35;
float important_right = 35;
float important_left = 35;

float bleed_amount = 200;

float proportional_front = 0;
float proportional_left = 0;
float proportional_right = 0;
float proportional_far_left = 0;
float proportional_far_right = 0;

float prefer_left;
float prefer_right;

float left_weight;
float right_weight;

float avoidance_speed = 0;

//--------------------------------------------------------------------STARTUP-----------------------------------------------------------------------//
void setup() {
  Serial.begin(57600);
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

  //initalize head position to start
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(90);

  //initalize ultra sonic
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  Serial.println("------------------------------------STARTING UP---------------------------------------------");
  buzzer.play("c32");
  delay(1000);
}

//--------------------------------------------------------------------LOOP-----------------------------------------------------------------------//
void loop() {
  if (!end) {

    servo();          //Move the head Position
    usReadCm();       //Read the US Distance
    checkEncoders();  //checks the distance
    calculatepos();   //Calculate our position
    checkTarget();    //Check to see if we are at our goal
    Move();           //Move
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
  if (currentMillis - servo_prevMillis >= SCAN_PERIOD && us_done) {

    headServo.write(counter);
    currAngle = counter;
    if (counter == 50) {
      returno = false;

    } else if (counter == 130) {
      returno = true;
    }

    if (returno && counter > -1) {
      counter = counter - 20;
    } else {
      counter += 20;
    }
    servo_done = true;
    us_done = false;
    servo_prevMillis = currentMillis;
  }  //timer
}  //Scan

//-------------------------USREAD---------------------------
void usReadCm() {
  //Serial.print("Starting");
  currentMillis = millis();
  if (currentMillis > us_prevMillis + US_PERIOD && servo_done) {
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
    if (currAngle == 110) {
      measured_distance_right = distance;
    } else if (currAngle == 70) {
      measured_distance_left = distance;
    } else if (currAngle == 50) {
      measured_distance_far_left = distance;
    } else if (currAngle == 130) {
      measured_distance_far_left = distance;
    } else {
      measured_distance_front = distance;
    }

    //update the prevmillis
    us_prevMillis = currentMillis;
    us_done = true;
    servo_done = false;
  }  //if
}  // usReadcm

//-------------------------ENCODERS---------------------------
void checkEncoders() {
  currentMillis = millis();
  if (currentMillis - encoder_prevMillis >= ENCODER_PERIOD && move_done) {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    // Calculate the distance traveled based on the encoder counts
    float distanceLeft = (((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE) * -1);     // -1 to account for sensor orientation
    float distanceRight = (((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE) * -1);  // -1 to account for sensor orientation

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
      currtheta = currtheta - 6.28;
    }  //if its too big
    if (currtheta < -3.14) {
      currtheta = currtheta + 6.28;
    }  //if its too big
    //PID THINGS
    error = targetTheta - currtheta;
    //Kp get the proportional correction
    proportional1 = KP1 * error;

    //----------------CALCULATIONS FOR AVOIDANCE----------------
    prefer_left = abs(targetTheta - currtheta);
    prefer_right = abs(targetTheta - -currtheta);

    if (measured_distance_front < important_front && measured_distance_front > 0) {
      error_angle = measured_distance_front - desired_distance;
      proportional_front = KPfront * error_angle;

      //which way is closer to the target, left or right?
      //negattive is right
      // Check if the current position is within the specified margin of error for the x-axis
      if (!(prefer_left - .1 <= prefer_right && prefer_right <= prefer_left + .1)) {
        if (prefer_left > prefer_right) {
          proportional_front = proportional_front * -1;
        }
      } else if (left_weight < right_weight) {
        proportional_front = proportional_front * -1;
      }
    } 
    else {
      proportional_front = prev_proportional_front;
    }
    if (measured_distance_left < important_left && measured_distance_left > 0) {
      error_angle = measured_distance_left - desired_distance;
      proportional_left = KPleft * error_angle * -1; // *-1 for left values
    }  //
    else {
      proportional_left = prev_proportional_left;
    }
    if (measured_distance_right < important_right && measured_distance_right > 0) {
      error_angle = measured_distance_right - desired_distance;
      proportional_right = KPright * error_angle;  // * -1; Testing comment
    }                                              
    else {
      proportional_right = prev_proportional_right;
    }

    //FAR VALUES
    if (measured_distance_far_left < important_left && measured_distance_far_left > 0) {
      error_angle = measured_distance_far_left - desired_distance;  // flip these back if it doesnt work
      proportional_far_left = KP_far_left * error_angle * -1; // *-1 for left values
    }  
    else {
      proportional_far_left = prev_proportional_far_left;
    }
    if (measured_distance_far_right < important_right && measured_distance_far_right > 0) {
      error_angle = measured_distance_far_right - desired_distance;
      proportional_far_right = KP_far_right * error_angle;  // * -1; Testing comment
    }                                                       
    else {
      proportional_far_right = prev_proportional_far_right;
    }

    avoidance_speed = proportional_left + proportional_right + proportional_far_left + proportional_far_right + proportional_front;  //+intergal + derivative;

    // Bleed Values to decrease overtime 
    prev_proportional_front = proportional_front - proportional_front / bleed_amount;
    prev_proportional_left = proportional_left - proportional_left / bleed_amount;
    prev_proportional_right = proportional_right - proportional_right / bleed_amount;
    prev_proportional_far_right = proportional_far_right - proportional_far_right / bleed_amount;
    prev_proportional_far_left = proportional_far_left - proportional_far_left / bleed_amount;
    //not saving far values

    left_weight = proportional_left + proportional_far_left + prev_proportional_left + prev_proportional_far_left;
    right_weight = proportional_right + proportional_far_right + prev_proportional_right + prev_proportional_far_right;

    pidResult = proportional1 + avoidance_speed;  //+intergal + derivative;
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
      buzzer.play("c32");
      delay(200);
      buzzer.play("e32");
      delay(200);
      buzzer.play("g32");
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

    //slowing down, but not slower than min speed
    if (targetDistance < 10) {
      BASE_SPEED = constrain(BASE_SPEED / 2, MIN_SPEED, MAX_SPEED);
    }
    else {
      BASE_SPEED = 70;
    }
    leftSpeed = BASE_SPEED - pidResult;
    rightSpeed = BASE_SPEED + pidResult;

    if ((leftSpeed < MAX_SPEED) || (rightSpeed < MAX_SPEED)) {  ///mosty recent
      leftSpeed = leftSpeed * .5;
      rightSpeed = rightSpeed * .5;
    }

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

    Serial.print("|Lmes: ");
    Serial.print(Sl);

    Serial.print("|Rmes: ");
    Serial.print(Sr);

    Serial.print("|Piddiff: ");
    Serial.print(pidResult);

    Serial.print("|distnace: ");  // He He He Distnace
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

    Serial.print("|dS: ");
    Serial.print(dS);

    Serial.print("|Base_speed: ");
    Serial.print(BASE_SPEED);

    Serial.print("|currAngle: ");
    Serial.print(currAngle);

    Serial.print("|avoid speed: ");
    Serial.print(avoidance_speed);

    Serial.print("|mesured front: ");
    Serial.print(measured_distance_front);

    Serial.print("|mesured right: ");
    Serial.print(measured_distance_right);

    Serial.print("|mesured leftt: ");
    Serial.print(measured_distance_left);

    Serial.print("|mesured far left: ");
    Serial.print(measured_distance_far_left);

    Serial.print("|mesured far right: ");
    Serial.print(measured_distance_far_right);

    Serial.print("|pro front: ");
    Serial.print(proportional_front);

    Serial.print("|pro right: ");
    Serial.print(proportional_right);

    Serial.print("|pro leftt: ");
    Serial.print(proportional_left);

    Serial.print("|pro far left: ");
    Serial.print(proportional_far_left);

    Serial.print("|pro far right: ");
    Serial.print(proportional_far_right);

    Serial.print("|pro prev front: ");
    Serial.print(prev_proportional_front);

    Serial.print("|pro prev right: ");
    Serial.print(prev_proportional_right);

    Serial.print("|pro prev leftt: ");
    Serial.print(prev_proportional_left);

    Serial.print("|pro prev far left: ");
    Serial.print(prev_proportional_far_left);

    Serial.print("|pro prev far right: ");
    Serial.print(prev_proportional_far_right);

    Serial.println("");
    print_prevMillis = currentMillis;
  }  //timer
}  //print