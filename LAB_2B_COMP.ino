/*
Robotics, Lab 2A 
The Artificers
Andrew Scibelli, Steve Boucher Andrew Hatch 
Due: 2-25-24 11:55PM 
*/

#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4; 

Buzzer buzzer;
Motors motors;

//Intialize UltraSonic 

const int ECHO_PIN = 5; 
const int TRIG_PIN = 4; 

//Ultrasonic Maxes 

const int MAX_DISTANCE = 100.0; 

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

//Ultrasonic timing
unsigned long usCM; 
unsigned long usPM;
const unsigned long US_PERIOD = 50; 

//Motor timing
unsigned long motorCM; 
unsigned long motorPM;
const unsigned long MOTOR_PERIOD = 20; 

//Current US distance reading
float distance = 0; 


void setup() {

pinMode (ECHO_PIN, INPUT); 
pinMode (TRIG_PIN, OUTPUT);

delay (1000);
buzzer.play ("c32"); 
}//setup

void loop() {
  //update the current distance
  //Serial.println ("usREADCM"); for testing
  usReadCM();



  //Update the motor speeds
  //Serial.println ("SETMOTORS"); for testing
  setMotors();

}//loop

void usReadCM() {
  usCM = millis();
  if (usCM > usPM + US_PERIOD) {
    //Clears the TRIG_PIN (set low)
    digitalWrite(TRIG_PIN, LOW); 
    delayMicroseconds(2);

    //Set the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH); 
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW); 

    //Reads the ECHO_PIN returns the sound wave travel time in microseconds
    //Note the duration (380000 microseconds) that will allow for reading up max distance supported by the sensor
    long duration = pulseIn(ECHO_PIN, HIGH, 380000);
    //Calculating the distance 

    distance = duration * 0.034 / 2; // time of flight equation, speed of sound wave divided by 2
    
    //Apply limits
    if (distance > MAX_DISTANCE) distance = MAX_DISTANCE; 
    if (distance == 0) distance = MAX_DISTANCE;

    //Displays the distance on the Serial Monitor

    Serial.println ("Distance: "); 
    Serial.print (distance);
    Serial.print (" cm");
    Serial.print ("");

    //Update the prevmillis
    usPM = usCM; 
  }//if 
}// usReadcm

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

