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

//PID 
int SPEED_LIMIT = 200;

//GOALS 
const int NUMBER_OF_GOALS = 3;
float xGoals[NUMBER_OF_GOALS] = {30, 30, 0};
float yGoals[NUMBER_OF_GOALS] = {30, 60, 0};

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

// Function to calculate change in distance traveled by the center point of the robot between the wheels
float calculateDeltaS(float rightDistance, float leftDistance) {
    return (rightDistance + leftDistance) / 2.0;
}

// Function to calculate change in angle
float calculateDeltaTheta(float rightDistance, float leftDistance, float wheelSeparation) {
    return (rightDistance - leftDistance) / wheelSeparation;
}

// Function to calculate change in x and y position
void calculateDeltaXY(float deltaS, float deltaTheta, float& deltaX, float& deltaY, float currentTheta) {
    deltaX = deltaS * cos(currentTheta + deltaTheta / 2.0);
    deltaY = deltaS * sin(currentTheta + deltaTheta / 2.0);
}

float calculateDistanceToTarget(float targetX, float targetY) {
    // Calculate the difference between current position and target position in x and y directions
    float dx = targetX - currentX; 
    float dy = targetY - currentY; 

    float distance = sqrt(dx * dx + dy * dy);

    return distance;
}

float calculateAngleToTarget(float targetX, float targetY) {
    // Calculate the difference between current position and target position in x and y directions
    float dx = targetX - currentX; 
    float dy = targetY - currentY; 

 
    float angle = atan2(dy, dx);

    // Adjust the angle to be in the range [0, 2pi]
     // Might have to change if the angle is less than -2pi or greater than 2pi??? I'm not sure yet lol
    if (angle < 0) {
        angle += 2 * PI;
    }
    return angle;
}

int checkTarget (currentX, currentY){
  if (((targetX - 2) < currentX) && (currentX < (targetX + 2)) && ((targetY - 2) < currentY) && (currentY < (targetY + 2))){
    motors.setSpeeds(0,0);
    delay (1000);
    goalnum++;
  }//if
}//checkTarget
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


