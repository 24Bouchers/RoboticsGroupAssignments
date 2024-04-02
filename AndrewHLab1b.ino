#include <Pololu3piPlus32U4.h> //import library

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;

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
  // put your setup code here, to run once:
  Serial.begin(57600);
  delay(1000);
  buzzer.play("c32");
}

void loop() {
  // put your main code here, to run repeatedly:

  //if statements to compare if we actually want to run the program or not, each statement shuts off the previous one
  if(doItRunpt1 == true) {
    //move ahead 12
    wheelEncoders1();
  }
  if(doItRunpt2 == true) {
    //move back 12
    wheelEncoders2();
  }
  if(doItRunpt3 == true) {
    //move ahead 18
    wheelEncoders3();
  }
  //buzzer.play("c32"); //this is really loud, refer to the beep in wheelEncoders3
}


//move forward 12 inches
void wheelEncoders1() {
  currentMillis = millis();
  if (currentMillis > prevMillis + PERIOD) {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    //moving forwards so we count 30.5 cm
    int wheelSpeed = 75;
    if(Sl < 30.5) {
      //slows down our bot for a smooth transition into the next movement phase
      if(Sl > 20) {
        wheelSpeed = 75 * ((30 - Sl) / 10);
        if(wheelSpeed < 20) wheelSpeed = 20;
      }
      //vroom vroom yaaaaayyyy!
      motors.setSpeeds(wheelSpeed, wheelSpeed);
    }
    //if we've reached our target distance, temporarily shut off the motors, disable our current function and configure our booleans to swap movement methods
    else {
      motors.setSpeeds(0, 0);
      doItRunpt1 = false; //shut off our first function
      doItRunpt2 = true; //turn on our second function
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

//move back 12 inches
void wheelEncoders2() {

  currentMillis = millis();
  if (currentMillis > prevMillis + PERIOD) {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

  //negative numbers make the bot go backwards
  int wheelSpeed = -75;
    //target of 0 cm since we want to reverse the distance we traveled in our first method
    if (Sl > 0) {
      if (Sl < 10.5) {
        wheelSpeed = -75 * ((-30 - Sl) / -10);
        if (wheelSpeed < -40) wheelSpeed = -40; //adjust this if too fast
      }
      motors.setSpeeds(wheelSpeed, wheelSpeed);
      }
    else {
      motors.setSpeeds(0,0);
      doItRunpt2 = false; //shut off our second function
      doItRunpt3 = true; //turn on our third function
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

//move forward 18 inches and die
   void wheelEncoders3() {

  currentMillis = millis();
  if (currentMillis > prevMillis + PERIOD) {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    int wheelSpeed = 75;
    //target of 46 cm since it = 18 inches
    if (Sl < 46) {
      if (Sl > 35) {
        wheelSpeed = 70 * ((30 - Sl) / 10);
        if(wheelSpeed < 40) wheelSpeed = 40;
      }
      motors.setSpeeds(wheelSpeed, wheelSpeed);
    }
    else {
      motors.setSpeeds(0,0);
      doItRunpt3 = false; //shut off our third function
      Serial.println("Done!"); //testing
      buzzer.play("c32"); //beep
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