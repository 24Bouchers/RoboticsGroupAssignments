/*
Robotics, Lab 3B
The Artificers
Andrew Scibelli
Due: 4-1-24 11:55PM 
*/

//--------------------------------------------Variables--------------------------------------------------------------
#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include <time.h>

using namespace Pololu3piPlus32U4;
Buzzer buzzer; 
Motors motors;
Servo headServo;



//UltraSonics--------------------------------------------------------------

      //intialize UltraSonic 
      const int ECHO_PIN = 4;//mine, invert for others 
      const int TRIG_PIN = 5; 

      //ultrasonic maxes 
      const int MAX_DISTANCE =200;// (200cm /2 meters)

      //ultrasonic timing
      unsigned long usCm; 
      unsigned long usPm;
      const unsigned long US_PERIOD = 100; 

      //current US distance reading
      float distance = 0; 

//Movement--------------------------------------------------------------

      //Motor timing
      unsigned long motorCM; 
      unsigned long motorPM;
      const unsigned long MOTOR_PERIOD = 20; 

      //Determine the normalization factor based of MAX_DISTANCE
      const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
      const float STOP_DISTANCE = 30.0; 

      //Motor constants 
      const float MOTOR_BASE_SPEED = 300.0;
      const int MOTOR_MIN_SPEED = 30; 
      //determine the normalization factor based on MOTOR_BASE_SPEED
      const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100; 

      //Motor compensation 
      const float L_MOTOR_FACTOR = 1.0;
      const float R_MOTOR_FACTOR = 1.0;
      const float L_MOTOR_FACTOR_THRESHOLD = 80;
      const float R_MOTOR_FACTOR_THRESHOLD = 80;


//Head-----------------------------------------------------------------

      //Head Servo Timing 
      unsigned long headCm;
      unsigned long headPm;
      const unsigned long HEAD_MOVEMENT_PERIOD = 500; //half second

      //head servo constants 
      const int HEAD_SERVO_PIN = 22; //change for andrew/margerate 11
      const int NUM_HEAD_POSITIONS = 2;
      const int HEAD_POSITION[NUM_HEAD_POSITIONS] = {93, 63}; //93 forward, //60 is wall 

      //head servo data 
      boolean headDirectionClockwise = true; 
      int currentHeadPosition = 0; 

      //switches 
      const boolean HEAD_DEBUG = true;// for print statements 


//--------------------------------------------Variables--------------------------------------------------------------


//--------------------------------------------METHODS--------------------------------------------------------------
void setup() {
  Serial.begin(57600);

  //initalize head position to start 
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(93);
  //initalize ultra sonic
  pinMode (ECHO_PIN, INPUT); 
  pinMode (TRIG_PIN, OUTPUT);
}//setup 

void loop (){
setMotors();
}//loop

//---PID 
void pidcalc(float currentReading) {

  const double desiredState = 30; 

  const double kp = 1;
  const double ki = 1;
  const double kd = 1;

  double kiTotal = 0.0; 
  double previousError = 0.0; 
  long prevTime = millis();

  // fetch an ultrasonic sample 
  //get the error
  double error = desiredState - currentReading; 

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


}//PID 





//-----ULTRASONIC
float usReadCm() {
  usCm = millis();
  if (usCm> usPm + US_PERIOD) {
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
    //Displays the distance on the Serial Monitor

    Serial.print (" Distance: ");
    Serial.println (distance);
    
    //update the prevmillis
    usPm = usCm; 

  }//if 
  return distance;
}// usReadcm








//---MOVEMENT
void setMotors(){
  //Start out with the MOTOR_BASE_SPEED
  float leftSpeed = MOTOR_BASE_SPEED;
  float rightSpeed = MOTOR_BASE_SPEED;

  //Check to see if the most current distance measurement is less than / equal to MAX_DISTANCE
  if (distance <= MAX_DISTANCE) {

    /*Determine the magintude of the distance by takin g the difference (short distance = high magnitude)
    divide by the DISTANCE_FACTOR to ensure uniform response as MAX_DISTANCE changes
    this maps the distance range (1 - MAX_RANGE ) to 0 -100 for the magnitude    
    */
    float magnitude = (float)(MAX_DISTANCE - distance) / DISTANCE_FACTOR; 
    //ex 1 MAX_DISTANCE = 80 distance = 40 80-40/ .8 = 50 mid range
    //ex 2 MAX_DISTANCE = 160 distance = 40 80-160/ 1.6 = 75 top 1/4

    //Mutliple the magnitude by the MOTOR_FACTOR to map the magnitude range (0-100) to the motors
    //(0 = MOTOR_BASE_SEPEED)

    leftSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);
    rightSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);
    // The motors on the robot we were testing seemed fine, so no compensation was needed

  }//if

  //Lower limts check 
  if(leftSpeed < MOTOR_MIN_SPEED) leftSpeed = MOTOR_MIN_SPEED;
  if(rightSpeed < MOTOR_MIN_SPEED) rightSpeed = MOTOR_MIN_SPEED;

  //Add in motor compensation 
  if(leftSpeed <= L_MOTOR_FACTOR_THRESHOLD){
    leftSpeed *= L_MOTOR_FACTOR; 
  }//if


  if(rightSpeed <= R_MOTOR_FACTOR_THRESHOLD){
    rightSpeed *= R_MOTOR_FACTOR; 
  }//if


  //Checking stop distance 

  if(leftSpeed <= STOP_DISTANCE) leftSpeed = 0;
  if(rightSpeed <= STOP_DISTANCE) rightSpeed = 0;

  //Change it by making it negative because the mount is on backwards
  leftSpeed = leftSpeed * -1; 
  rightSpeed = rightSpeed * -1;

  motors.setSpeeds(leftSpeed, rightSpeed); 
  motorPM = motorCM;
}//setMotors






//---Head 
void moveHead() {
  headCm = millis();
  if(headCm > headPm + HEAD_MOVEMENT_PERIOD) {
    //head debug output
    if (HEAD_DEBUG) {
      Serial.print("Position #: ");
      Serial.print(currentHeadPosition + 1);//to display as 1,2,3 instead of 0,1,2
      Serial.print(" Angle: ");
      Serial.print(HEAD_POSITION[currentHeadPosition]);
    }//if

    //position head to the current position in the array 
    headServo.write (HEAD_POSITION[currentHeadPosition]); 

    /**
     *Set next head position and changes direction when needed
     *Moves servo to the next position and changes direction when needed. 
     *Delays were added to ensure ultrasonic can recieve pings before moving; will need adjustment for wall follow
     */

    if (headDirectionClockwise) {
          if(currentHeadPosition >= (NUM_HEAD_POSITIONS -1)) {
            headDirectionClockwise = !headDirectionClockwise;
            delay(100);//adjust need to be made for PID systems
            usReadCm();
            delay(100);
            currentHeadPosition--;
            }//if
          else {
            delay(100);
            usReadCm();
            delay(100);
            currentHeadPosition++;
          }//else   
    }//if
    
    else {
       if (currentHeadPosition <= 0){
         headDirectionClockwise = !headDirectionClockwise;     
         delay(100);
         usReadCm();
         delay(100);
         currentHeadPosition++;   
        }//if  
      else {
        delay(100);
        usReadCm();
        delay(100);
        currentHeadPosition--;
      }//else
    }//else
  //reset previous millis 
  headPm = headCm;
  }//if
}//moveHead

