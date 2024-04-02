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

//Vars
unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 20;

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.0531;

float Sl = 0.0F;
float Sr = 0.0F;

bool doItRunpt1 = true; //move 12 boolean
bool doItRunpt2 = false; //move back boolean
bool doItRunpt3 = false; //move 18 boolean

void setup() {
  Serial.begin(57600);

  startingUp();
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);
  startSong();
}
void loop() {
  // put your main code here, to run repeatedly:
  wheelEncoders(30.5);
}

void wheelEncoders(int goal) {
  currentMillis = millis();
  if (currentMillis > prevMillis + PERIOD) {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    //moving forwards so we count 30.5 cm
    int wheelSpeed = 75;
    if(Sl < goal) {
      //slows down our bot for a smooth transition into the next movement phase
      if(Sl > goal*.6) {
        wheelSpeed = 75 * ((goal - Sl) / 10);
        if(wheelSpeed < 20) wheelSpeed = 20;
      }
      //vroom vroom yaaaaayyyy!
      motors.setSpeeds(wheelSpeed, wheelSpeed);
    }
    //if we've reached our target distance, temporarily shut off the motors, disable our current function and configure our booleans to swap movement methods
    else {
      motors.setSpeeds(0, 0);
      Serial.println("Done!"); //testing
    }
  Serial.print("Left: ");
  Serial.print(Sl);
  Serial.print(" Right: ");
  Serial.println(Sr);

  prevLeft = countsLeft;
  prevRight = countsRight;
  prevMillis = currentMillis;
  
  }  
}


void startingUp(){
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
}

void startSong(){
  buzzer.play("c32");
  delay(100);
  buzzer.play("e32");
  delay(100);
  buzzer.play("g32");
}