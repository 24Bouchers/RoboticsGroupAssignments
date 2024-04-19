#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motor;
ButtonA buttonA;

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

int wheelSpeed = 100;

bool forwardComplete = false;
bool backwardComplete = false;
bool finalForwardComplete = false;

void setup() {
  Serial.begin(57600);
  delay(1000);
  buzzer.play("c32");
}

void loop() {
  checkEncoders();

  if (!forwardComplete) {   
    if(Sl > 20){
        wheelSpeed = 100 * ((30 - Sl) / 10);
        //if(wheelSpeed < 20) wheelSpeed = 20;
        if (Sl > 25){
          wheelSpeed = 30;
        }
      }
      
    motor.setSpeeds(wheelSpeed, wheelSpeed);

  } else if (!backwardComplete) {
    int wheelSpeed = -100;
    /*if(Sl > 0){
        wheelSpeed = 100 * ((Sl - 30) / 10);
        //if(wheelSpeed < 20) wheelSpeed = 20;
        if (Sl < 10){
          wheelSpeed = -20;
        }
      }*/
    motor.setSpeeds(wheelSpeed, wheelSpeed);
    
  } else if (!finalForwardComplete) {
    int wheelSpeed = 100;
    if(Sl > 30){
        wheelSpeed = 100 * ((43 - Sl) / 10);
        //if(wheelSpeed < 20) wheelSpeed = 20;
        if (Sl > 40){
          wheelSpeed = 30;
        }
      }
    motor.setSpeeds(wheelSpeed, wheelSpeed);
  } else {
    motor.setSpeeds(0, 0);
    buzzer.play("c32");
    while (1) {
      // Stay in this loop after completing all maneuvers
    }
  }
}

void checkEncoders() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= PERIOD) {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    if (!forwardComplete && Sl >= 30) {
      forwardComplete = true;
      Serial.println("Forward maneuver complete");
    } else if (forwardComplete && !backwardComplete && Sl <= 0.0) {
      backwardComplete = true;
      Serial.println("Backward maneuver complete");
    } else if (forwardComplete && backwardComplete && !finalForwardComplete && Sl >= 43) {
      finalForwardComplete = true;
      Serial.println("Final forward maneuver complete");
    }

    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMillis = currentMillis;

    Serial.print("Left: ");
    Serial.print(Sl);
    Serial.print(" Right: ");
    Serial.println(Sr);
  }
}
