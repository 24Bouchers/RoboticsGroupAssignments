
/*
Robotics, Lab 4
The Artificers
Andrew Scibelli, Steve Boucher Margarite 
Due: 4-12-24 11:55PM 

slowing down fucntion 
if statment current time  greater every 20 milliseconds I check to see encoders
set timer on each function?

*/

#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include <time.h>

using namespace Pololu3piPlus32U4;
Encoders encoders;
Buzzer buzzer;
Motors motors;

//-------------------------VARS---------------------------

//check encoders
unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 10;

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.531;

float Sl = 0.0F;
float Sr = 0.0F;

//speeds
int MAX_SPEED = 400;
float MIN_SPEED = -75;
float BASE_SPEED = 50;

float rightSpeed = 0;
float leftSpeed = 0;

//PID
const double KP = 0.8;  // Adjust kp for quicker response
const double KI = 0.0;  // Reduce ki for less integration
const double KD = 0.0;  // Increase kd for faster damping

//GOALS
const int NUMBER_OF_GOALS = 3;
float xGoals[NUMBER_OF_GOALS] = { 30, 60, 20 };
float yGoals[NUMBER_OF_GOALS] = { 30, 30, -30 };

int goalnum = 0;
float targetX = xGoals[goalnum];
float targetY = yGoals[goalnum];
float targetTheta = 0;//starts at 0 regardless 
float targetDistance = 0;

const float WHEEL_DIS = 8.5;//possibly 8.2

//deltas

float dS = 0;
float dT = 0;
float dX = 0;
float dY = 0;

//



//end
bool end = false;
const int margin_of_error = 2.0;


//-------------------------LOOP---------------------------
void setup() {
 Serial.begin(57600);

}

void loop() {
  if (!end) {
    checkEncoders();  //checks the distance
    calculatepos();
    Move(pidcalc());
    checkTarget();
    PRINT();
  }  //if
}  //loop

//-------------------------MOVE---------------------------

void Move(float piddiff) {
  //slowing down, but not slower than min speed 
  if (targetDistance < 10)
    {
      MAX_SPEED = constrain (MAX_SPEED/2, MIN_SPEED, 400);
    }//i

  //rightSpeed = rightSpeed - piddiff;
  //leftSpeed = leftSpeed + piddiff; 
  rightSpeed = constrain(rightSpeed - piddiff , MIN_SPEED, MAX_SPEED);
  leftSpeed = constrain(leftSpeed + piddiff , MIN_SPEED, MAX_SPEED);
  motors.setSpeeds(leftSpeed + BASE_SPEED, rightSpeed+ BASE_SPEED);
}  //move


//-------------------------MATH---------------------------

void calculatepos() {
   float xdiff = targetX - dX;
  float ydiff = targetY - dY;
  
  dS = (Sr + Sl) / 2;
  dT = (Sl - Sr) / WHEEL_DIS;
  //the first dT in these may need to be current theta, but IDK
  dX = (dS * cos(dT + (dT / 2.0)));  //should be cos
  dY = dS * sin(dT + (dT / 2.0));         // should be sin

  // Calculate the difference between current position and target position in x and y directions
  targetDistance = sqrt(xdiff * xdiff + ydiff * ydiff);
  Serial.print("|distnace: ");
  Serial.print(targetDistance);
  targetTheta = atan2(ydiff, xdiff);
  Serial.print("|TargetANGLE: ");
  Serial.print(targetTheta);
}  //calculate pos

//-------------------------PID---------------------------
float pidcalc() {
  double kiTotal = 0.0;
  double previousError = 0.0;
  double error = targetTheta - dT;
  //Kp get the proportional correction
  double proportional = KP * error;
  //Ki get intergral correction
  kiTotal += error;
  double integral = KI * kiTotal;
  //Kd derivative
  float derivative = KD * (error - previousError);
  previousError = error;
  //sum
  float pidResult = proportional + integral + derivative;
  //pidResult = constrain(pidResult, -3.14, 3.14);
  //apply the sum to the motors, one will be +pidSUm, the other -pidSum
  Serial.print("|Piddiff: ");
  Serial.print(pidResult);
  return pidResult;
}  //PID

//-------------------------CHECK---------------------------

void checkTarget() {
  //we can just (theorically) check if the distance is within -2 - 2
  if (targetRange()) {
    motors.setSpeeds(0, 0);
    buzzer.play("c32");
    delay(100);
    buzzer.play("e32");
    delay(100);
    buzzer.play("g32");
    delay(1000);
    Serial.print("--------------------------CHECKPOINT #");
    Serial.print(goalnum);
    Serial.println("--------------------------");
    goalnum++;
    //reassing
    targetX = xGoals[goalnum];
    targetY = yGoals[goalnum];
    MAX_SPEED = 200;
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
}  //check target


bool targetRange() {
  if (0 - margin_of_error <= targetDistance && targetDistance <= 0 + margin_of_error) {
    return true;
  }
  else {
  return false;
  }
}//targetRange

//-------------------------ENCODERS---------------------------

void checkEncoders() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= PERIOD) {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    // Calculate the distance traveled based on the encoder counts
    float distanceLeft = (((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE));
    float distanceRight = (((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE));

    // Update the values of Sl and Sr
    Sl += distanceLeft;
    Sr += distanceRight;

    // Update the previous counts
    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMillis = currentMillis;
  }//timer
}//encoders

//--------------------------PRINT-----------------------------

void PRINT() {
  Serial.print("|Lmes: ");
  Serial.print(Sl);
  Serial.print("|Rmes: ");
  Serial.print(Sr);
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
  Serial.println(" ");
}  //print
