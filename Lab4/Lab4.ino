/*
Robotics, Lab 4
The Artificers
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

//PID 
int MAX_SPEED = 400;
float MIN_SPEED = -400;

float rightSpeed = 0;
float leftSpeed = 0;

const double KP = 1.0; // Adjust kp for quicker response    
const double KI = 0.0; // Reduce ki for less integration   
const double KD = 0.2; // Increase kd for faster damping  

//GOALS 
const int NUMBER_OF_GOALS = 3;
float xGoals[NUMBER_OF_GOALS] = {30, 30, 0};
float yGoals[NUMBER_OF_GOALS] = {30, 60, 0};

int goalnum = 0;
float targetX =xGoals[goalnum];
float targetY =yGoals[goalnum];

//notes

float currx = 0;
float curry = 0;
float currtheta = -1.571;//0;//1.571; //90 in radians?

const float WHEEL_DIS = 8.5;

//deltas

float dS = 0;
float dT = 0;
float dX = 0;
float dY = 0;

//end
bool end = false;
const int margin_of_error = 2;

//SETUP&LOOP-----------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(57600);
  motors.flipRightMotor(true);
  motors.flipLeftMotor(true);
  Serial.println("------------------------------------STARTING UP---------------------------------------------");

  rightSpeed = MIN_SPEED;
  leftSpeed = MIN_SPEED;

  buzzer.play("c32");
  delay (1000);
}//setup

void loop() {
  if(!end){
  checkEncoders();//checks the distance
  calculatepos();
  updatepos();
  Move(pidcalc(targetAngle()));
  checkTarget(targetDistance());
  PRINT();
}//if
}//loop

//METHIDS------------------------------------------------------------------------------
void PRINT(){
  Serial.print("|Lmes: ");
  Serial.print(Sl);
  Serial.print("|Rmes: ");
  Serial.print(Sr);
  
  
  //Serial.print("|LSpeed: ");
  //Serial.print(leftSpeed);
  //Serial.print("|RSpeed: ");
  //Serial.print(rightSpeed);
  Serial.print("|currX: ");
  Serial.print(currx);
  Serial.print("|curry: ");
  Serial.print(curry);
  Serial.print("|currtheta: ");
  Serial.print(currtheta);
  Serial.print("|Tarx: ");
  Serial.print(targetX);
  Serial.print("|targetY: ");
  Serial.print(targetY);
  /*Serial.print("|dS: ");
  Serial.print(dS);
  Serial.print("|dT: ");
  Serial.print(dT);
  Serial.print("|dX: ");
  Serial.print(dX);
  Serial.print("|dY: ");
  Serial.print(dY);
 */
 Serial.println(" ");
}//print


void calculatepos (){
  dS = (Sr + Sl)/2;
  dT = (Sr - Sl)/WHEEL_DIS;
  dX = (dS * sin(currtheta + (dT/2.0))) *-1; //should be cos
  dY = dS * cos(currtheta + (dT/2.0)); // should be sin 
  }//calculate pos

//this updates the values of pos for the next calculation
void updatepos (){
  currx = dX;
  curry = dY;
  currtheta = dT; 
}//deltaXY

//this one gives the distnce, alllowing us to slow down
float targetDistance() {
  float distance = 0;
// Calculate the difference between current position and target position in x and y directions
    float xdiff = targetX - currx; 
    float ydiff = targetY - curry; 
    distance = sqrt(xdiff * xdiff + ydiff * ydiff);
    Serial.print("|distnace: ");
    Serial.print(distance);
  return distance;
}//target

//NOW THAT WE HAVE POS, WE CAN CALCULATE THINGS BASED ON IT we want PID to move to the corret orientation
//MOST WONT BE USED BUT GOOD TO HAVE
//takes the actual theta, and then determines desidredtheta, and to get desired theta, we need to calculate that based on the slope of a line
//yeilds the target angle we neeed for pid calc
float targetAngle() {
    // Calculate the difference between current position and target position in x and y directions
    float dx = targetX - currx; 
    float dy = targetY - curry; 
    float angle = atan2(dy, dx);
    //
    //angle = (angle*PI)/180;
    Serial.print("|TargetANGLE: ");
    Serial.print(angle);
    return angle;
}//targetangle


//---PID 
float pidcalc (float desiredangle) {  
  double kiTotal = 0.0; 
  double previousError = 0.0; 
  double error = desiredangle - currtheta; 
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

  pidResult = constrain(pidResult, -3.14, 3.14);
  //apply the sum to the motors, one will be +pidSUm, the other -pidSum 
  Serial.print("|Piddiff: "); 
  Serial.print(pidResult); 
  return pidResult;
}//PID

//CHECKTARGET--------------------------------------------------------------
void checkTarget(float distance) {
    //we can just (theorically) check if the distance is within -2 - 2
    
    if (targetRange()){ 
        motors.setSpeeds(0,0);
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
  
    }//if 

    if (goalnum > (NUMBER_OF_GOALS - 1))    
    {
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
    }//if
}//check target 


bool targetRange() {
  bool x_bounds = false;
  bool y_bounds = false;

  // Check if the current position is within the specified margin of error for the x-axis
  if (targetX - margin_of_error <= currx && currx <= targetX + margin_of_error) {
    x_bounds = true;
  }

  // Check if the current position is within the specified margin of error for the y-axis
  if (targetY - margin_of_error <= curry && curry <= targetY + margin_of_error) {
    y_bounds = true;
  }
  if(x_bounds && y_bounds){
    return true;
  }
  else
  {
    return false;
  }
}

//--------------------------------------------CHECK_ENCODERS--------------------------------------------------------------
void checkEncoders() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= PERIOD) {
    countsLeft += encoders.getCountsAndResetRight();
    countsRight += encoders.getCountsAndResetLeft();

    // Calculate the distance traveled based on the encoder counts
    float distanceLeft = (((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE)*-1);
    float distanceRight = (((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE)*-1);
    
    // Update the values of Sl and Sr
    Sl += distanceLeft;
    Sr += distanceRight;

    // Update the previous counts
    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMillis = currentMillis;
  }
}



//slowing down function 


//--------------------------------------------PID&MOVE--------------------------------------------------------------

//---MOVEMENT
void Move(float piddiff){
  
//left is right and right is left 
  rightSpeed = constrain(rightSpeed + piddiff, MIN_SPEED, MAX_SPEED);
  leftSpeed = constrain(leftSpeed - piddiff, MIN_SPEED, MAX_SPEED);
  motors.setSpeeds(rightSpeed, leftSpeed); 
}//move
