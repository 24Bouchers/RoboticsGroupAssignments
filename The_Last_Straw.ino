
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
unsigned long encoder_prevMillis;
unsigned long calc_prevMillis;
unsigned long check_prevMillis;
unsigned long calc1_prevMillis;
unsigned long calc2_prevMillis;
unsigned long check2_prevMillis;
unsigned long move_prevMillis;
unsigned long print_prevMillis;


const unsigned long ENCODER_PERIOD = 10;
const unsigned long CALC_PERIOD = 10;
const unsigned long CHECK_PERIOD = 10;
const unsigned long MOVE_PERIOD = 10;


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
int MAX_SPEED = 100;
float MIN_SPEED = 0;
float BASE_SPEED = 50;

float rightSpeed = 0;
float leftSpeed = 0;

//PID
const double KP = .08; // Adjust kp for quicker response
const double KI = 0;  // Reduce ki for less integration
const double KD = 0;  // Increase kd for faster damping

//GOALS
const int NUMBER_OF_GOALS = 4;
float xGoals[NUMBER_OF_GOALS] = { 60, 0, -90, 0};
float yGoals[NUMBER_OF_GOALS] = { 30, -50, 0, 0};

int goalnum = 0;
float targetX = xGoals[goalnum];
float targetY = yGoals[goalnum];
float targetTheta = 0;//starts at 0 regardless 
float targetDistance = 1000000000000000000000;

const float WHEEL_DIS = 8.5;//possibly 8.2

//deltas

float dS = 0;
float dT = 0;
float dX = 0;
float dY = 0;

//position 

float currx = 0; 
float curry = 0;
float currtheta = 1.57;

float pidResult = 0;

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
    calculatepos(); //
    updatepos();
    pidcalc();
    Move();
    checkTarget();
  //  PRINT();
  }  //if
}  //loop
//---------------------METHODS----------------------------
//this updates the values of pos for the next calculation
void updatepos (){
  currentMillis = millis();
  if (currentMillis - calc2_prevMillis >= CALC_PERIOD) {
  Serial.println("DOING UPDATE");
  
  currx = currx + dX;
  curry = curry + dY;
  currtheta = currtheta + dT; 

  calc2_prevMillis = currentMillis;
  }//timer  

}//deltaXY

//-------------------------MATH---------------------------

void calculatepos() {
  currentMillis = millis();
  if (currentMillis - calc_prevMillis >= CALC_PERIOD) {
    Serial.println("DOING CALC");
  float xdiff = targetX - currx;
  float ydiff = targetY - curry;
  
  dS = (Sr + Sl) / 2;
  dT = (Sr - Sl) / WHEEL_DIS;
  
  //the first dT in these may need to be current theta, but IDK
  dX = (dS * cos(currtheta + (dT / 2.0)));  //should be cos
  dY = dS * sin(currtheta + (dT / 2.0));         // should be sin

  // Calculate the difference between current position and target position in x and y directions
  targetDistance = sqrt(xdiff * xdiff + ydiff * ydiff);

  targetTheta = atan2(ydiff, xdiff);
  calc_prevMillis = currentMillis;
  }//timer
}  //calculate pos

//-------------------------MOVE---------------------------

void Move() {
  currentMillis = millis();
  if (currentMillis - move_prevMillis >= MOVE_PERIOD) {
  Serial.println("DOING MOVE");
  //slowing down, but not slower than min speed 
  if (targetDistance < 10)
    {
      MAX_SPEED = constrain (MAX_SPEED/2, MIN_SPEED, 400);
    }//i

  rightSpeed = rightSpeed + pidResult;
  leftSpeed = leftSpeed - pidResult; 
  //rightSpeed = constrain(rightSpeed - piddiff, MIN_SPEED, MAX_SPEED);
  //leftSpeed = constrain(leftSpeed + piddiff, MIN_SPEED, MAX_SPEED);
  motors.setSpeeds(leftSpeed + BASE_SPEED, rightSpeed + BASE_SPEED);
  move_prevMillis = currentMillis;
  Serial.println("DONE MOVE");
  }//timer
}  //move


//-------------------------PID---------------------------
float pidcalc() {
  currentMillis = millis();
  if (currentMillis - calc1_prevMillis >= CALC_PERIOD) {
    Serial.println("DOING PID");

  double kiTotal = 0.0;
  double previousError = 0.0;
  double error = targetTheta - currtheta;
  //Kp get the proportional correction
  double proportional = KP * error;
  //Ki get intergral correction
  //kiTotal += error;
  //double integral = KI * kiTotal;
  //Kd derivative
  //float derivative = KD * (error - previousError);
  //previousError = error;
  //sum
  pidResult = proportional; //+intergal + derivative;
  //pidResult = constrain(pidResult, -3.14, 3.14);
  //apply the sum to the motors, one will be +pidSUm, the other -pidSum
  calc1_prevMillis = currentMillis;
  Serial.println("DONE CALC1");
  }//timer
}  //PID

//-------------------------CHECK---------------------------

void checkTarget() {
  currentMillis = millis();
  if (currentMillis - check_prevMillis >= CHECK_PERIOD) {
    Serial.println("DOING TARGET");
  //we can just (theorically) check if the distance is within -2 - 2
  if (targetRange()) {
    motors.setSpeeds(0, 0);
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
  check_prevMillis = currentMillis;
  Serial.println("DONE CHECK");
  }//timer
}  //check target


bool targetRange() {
   currentMillis = millis();
  if (currentMillis - check2_prevMillis >= CHECK_PERIOD) {
    Serial.println("DOING TARGET2");
  if (0 - margin_of_error <= targetDistance && targetDistance <= 0 + margin_of_error) {
    return true;
  }
  else {
  return false;
  }
  check2_prevMillis = currentMillis;
  Serial.println("DONE TARGET 2");
  }//timer
}//targetRange

//-------------------------ENCODERS---------------------------

void checkEncoders() {
  currentMillis = millis();
  if (currentMillis - encoder_prevMillis >= ENCODER_PERIOD) {
    Serial.println("DOING ENCODERS");
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    // Calculate the distance traveled based on the encoder counts
    float distanceLeft = (((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE));
    float distanceRight = (((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE));

    // Update the values of Sl and Sr, but not accumulating
    Sl = distanceLeft;
    Sr = distanceRight;

    // Update the previous counts
    prevLeft = countsLeft;
    prevRight = countsRight;
    encoder_prevMillis = currentMillis;
  Serial.println("DONE ENCODERS");
  }//timer
}//encoders

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
    //Serial.print("|LSpeed: ");
    //Serial.print(leftSpeed);
    //Serial.print("|RSpeed: ");
    //Serial.print(rightSpeed);
    Serial.print("|dT @ print: ");
    Serial.print(dT);
    //Serial.print("|dX: ");
    //Serial.print(dX);
    //Serial.print("|dY: "); 
    //Serial.print(dY);
    Serial.print("|Tarx: ");
    Serial.print(targetX);
    Serial.print("|targetY: ");
    Serial.print(targetY);
    
  
  //Serial.print("|dS: ");
  //Serial.print(dS);
  Serial.println("");
  print_prevMillis = currentMillis;
  }//timer

}  //print
