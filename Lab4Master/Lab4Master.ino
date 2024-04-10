/*
Robotics, Lab 4
The Artificers
Due: 4-1-24 11:55PM 
*/

#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include <time.h>

using namespace Pololu3piPlus32U4;
Encoders encoders;
Buzzer buzzer; 
Motors motors;

//check encoders
unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 50;  

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

//PID 
int SPEED_LIMIT = 200;

//GOALS 
const int NUMBER_OF_GOALS = 3;
float xGoals[NUMBER_OF_GOALS] = {30, 30, 0};
float yGoals[NUMBER_OF_GOALS] = {30, 60, 0};

float currentX;
float currentY;
int goalnum;
float rightSpeed;
float leftSpeed;
float targetX;
float targetY;


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}//loop

// Function to calculate change in distance traveled by the center point of the robot between the wheels
float calculateDeltaS(float rightDistance, float leftDistance) {
    return (rightDistance + leftDistance) / 2.0;
}//calculateDeltaS

// Function to calculate change in angle
float calculateDeltaTheta(float rightDistance, float leftDistance, float wheelSeparation) {
    return (rightDistance - leftDistance) / wheelSeparation;
}//calculateDeltaTheta

// Function to calculate change in x and y position
void calculateDeltaXY(float deltaS, float deltaTheta, float& deltaX, float& deltaY, float currentTheta) {
    deltaX = deltaS * cos(currentTheta + deltaTheta / 2.0);
    deltaY = deltaS * sin(currentTheta + deltaTheta / 2.0);
}//calculateDeltaXY

float calculateDistanceToTarget(float targetX, float targetY) {
    // Calculate the difference between current position and target position in x and y directions
    float dx = targetX - currentX; 
    float dy = targetY - currentY; 

    float distance = sqrt(dx * dx + dy * dy);

    return distance;
}//calculateDistanceToTarget

float calculateAngleToTarget(float targetX, float targetY) {
    // Calculate the difference between current position and target position in x and y directions
    float dx = targetX - currentX; 
    float dy = targetY - currentY; 

 
    float angle = atan2(dy, dx);

    // Adjust the angle to be in the range [0, 2pi]
    if (angle < 0) {
        angle += 2 * PI;
    }//if 
    return angle;
}//calculateAngleToTarget

void checkTarget(float currentX, float currentY) {
    if (((targetX - 2) < currentX) && (currentX < (targetX + 2)) && ((targetY - 2) < currentY) && (currentY < (targetY + 2))) {
        motors.setSpeeds(0, 0);
        delay(1000);
        goalnum++;
    }//if
}//checkTarget
//--------------------------------------------CHECK_ENCODERS--------------------------------------------------------------
void checkEncoders() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= PERIOD) {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    //add the *-1 to accomdate for backwards direction
    
    Sl += (((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE)* -1);
    Sr += (((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE)*-1);

    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMillis = currentMillis;

    Serial.print("Left: ");
    Serial.print(Sl);
    Serial.print(" Right: ");
    Serial.println(Sr);//making it negtive for direction 
  }//if
}//check Encoders


//--------------------------------------------PID&MOVE--------------------------------------------------------------
//---PID 
float pidcalc (float measuredangle, float desiredangle) {
  const double kp = 0.0; // Adjust kp for quicker response    
  const double ki = 0.0; // Reduce ki for less integration   
  const double kd = 0.0; // Increase kd for faster damping  
  double kiTotal = 0.0; 
  double previousError = 0.0; 
  double error = desiredangle - measuredangle; 
  //Kp get the proportional correction 
  double proportional = kp * error;  
  //Ki get intergral correction 
  kiTotal += error;
  double integral = ki * kiTotal; 
  //Kd derivative
  float derivative = kd * (error - previousError); 
  previousError = error; 
  //sum 
  float pidResult = proportional + integral + derivative; 
  //apply the sum to the motors, one will be +pidSUm, the other -pidSum 
  Serial.print("|Piddiff: "); 
  Serial.print(pidResult); 
  return pidResult;
}//PID
//---MOVEMENT
void Move(float piddiff){
//left is right and right is left 
  rightSpeed = rightSpeed - piddiff; 
  leftSpeed = leftSpeed + piddiff; 
  //limtor to prvent values from scalling too much 
  if ((leftSpeed < SPEED_LIMIT)||(rightSpeed < SPEED_LIMIT))
    {
      leftSpeed = leftSpeed * .5;
      rightSpeed = rightSpeed * .5; 
    }//if
  Serial.print ("|RightSpeed: ");
  Serial.print (leftSpeed);
  Serial.print ("|LeftSpeed: ");
  Serial.print (rightSpeed);
  motors.setSpeeds(rightSpeed, leftSpeed); 
}//move


