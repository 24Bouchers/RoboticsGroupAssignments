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

      //speeds

      float leftSpeed = 0;
      float rightSpeed = 0;
      float speedLimit = -200;// to prevent large numbers 
     
      //Motor constants 
      const float MOTOR_BASE_SPEED = -50.0; // for consistency

//Head-----------------------------------------------------------------
      const int HEAD_SERVO_PIN = 11; //change for andrew/margerate/Steve 11
      int check = 0;  
      int checkfrequency = 1000;
      int importantdistance = 100; 

      const int checkdefault = 1000;
      const int distancedefault = 100; 
//--------------------------------------------Variables--------------------------------------------------------------


//--------------------------------------------METHODS--------------------------------------------------------------
void setup() {
  Serial.begin(57600);

  //initalize head position to start 
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(160);
  //initalize ultra sonic
  pinMode (ECHO_PIN, INPUT); 
  pinMode (TRIG_PIN, OUTPUT);

  buzzer.play("c32");
  //givng base speed
}//setup 

void loop (){
  leftSpeed = leftSpeed + MOTOR_BASE_SPEED;
  rightSpeed = rightSpeed + MOTOR_BASE_SPEED;
  motors.setSpeeds(rightSpeed, leftSpeed);
  Move2(pidcalc(averageScan()));
  checkServo();
  
}//loop

void checkServo(){
  //Head Pos
  if (check == 0)
  {
    buzzer.play("c32");
    headServo.write(160);
  }
  if(check == 1000)
  {
    buzzer.play("g32");
    headServo.write(90);
    check = 0; 
  }

  //Servo Reading
  float reading = averageScan();

  Serial.print ("|Important Distance: ");
  Serial.print (importantdistance);

  Serial.print ("|Check: : ");
  Serial.print (check);

  Serial.print ("|check frequnecy: ");
  Serial.println (checkfrequency);
  check ++;

}//checkfront


//---PID 
float pidcalc(float currentReading) {

    if (currentReading > 60)//limiting the input
    {currentReading = 60;}//if

  const double desiredState = 30; 

  const double kp = .005;
  const double ki = 0.50;
  const double kd = 0.25;

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

  Serial.print("|Piddiff: "); 
  Serial.print(pidResult); 

  return pidResult;
     
}//PID 


//---MOVEMENT
void Move2(float piddiff){

  //speeds will be negative, make them positive for calculations 
  leftSpeed = abs(leftSpeed);
  rightSpeed = abs(rightSpeed);

//left is right and right is left 

  rightSpeed = rightSpeed - piddiff; 
  leftSpeed = leftSpeed + piddiff; 
  
  //Change it by making it negative because the mount is on backwards
  leftSpeed = leftSpeed * -1; 
  rightSpeed = rightSpeed * -1;

  //limtor to prvent values from scalling too much 
  if ((leftSpeed < speedLimit)||(rightSpeed < speedLimit))
    {
      leftSpeed = leftSpeed * .5;
      rightSpeed = rightSpeed * .5; 
    }//if
  
  
  
  if (piddiff > -1 && piddiff < 1)
    {//Serial.print("IDEAL ACHIEVED");
    }//if 

  //Serial.print (" RightSpeed: ");
  //Serial.print (leftSpeed);
  //Serial.print ("LeftSpeed: ");
  //Serial.println (rightSpeed);


  motors.setSpeeds(rightSpeed, leftSpeed); 

}//move2


//--ULTRASONIC
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
    //update the prevmillis
    usPm = usCm; 

  }//if 
  return distance;
}// usReadcm


//gets average scan for pid 
float averageScan (){

  float a1 = usReadCm();
  float a2 = usReadCm();
  float a3 = usReadCm();
  float a4 = usReadCm();
  float a5 = usReadCm();

  float average = ((a1 + a2 + a3 + a4 + a5)/5); 
  Serial.print("| Average Read: ");
  Serial.print (average); 

  return average; 
}//averageScan
