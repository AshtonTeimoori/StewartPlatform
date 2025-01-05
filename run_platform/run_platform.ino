#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <Adafruit_PWMServoDriver.h>

#include <math.h>
#include <stdio.h>
#include "platform.h"

platform p = platform();

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int maxminArray[6][2] = {
  {165, 555}, // Broken
  {580, 190}, 
  {170, 555}, //165, 555
  {605, 105}, 
  {175, 570}, 
  {560, 175}
};

int mult = 1;
double angle = 0.0;
double angle2 = 5.0;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.setTimeout(10);
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);
}
// uint8_t servonum = 0;
void loop() {
  BLA::Matrix<3,1,double> n = {0, 1, 0.0};
  // BLA::Matrix<3,1,double> n = {sin(angle*M_PI/180.0), cos(angle*M_PI/180.0), 0.0};
  // BLA::Matrix<3,1,double> n = {-0.88, -0.47, 0.0};
  
  Serial.print("Inputs: ");
  Serial.print(n.storage[0]);
  Serial.print(", ");
  Serial.print(n.storage[1]);
  Serial.print(", ");
  Serial.print(n.storage[2]);
  Serial.print(", ");
  Serial.println(angle2);

  angle += double(mult)*.25;

  if (angle >= 5.0) {
    mult = -1;
  } else if (angle <= -5.0) {
    mult = 1;
  }
  double* servo_angles = p.getServos(n, angle);
  
  // angle += 1.0;
  // if (angle >= 360.0) {
  //   angle = 0.0;
  // }

  // double* servo_angles = p.getServos(n, angle2);

  // pwm.setPWM(servonum, 0, map(0, 90, -90, maxminArray[servonum][0], maxminArray[servonum][1]));
  // pwm.setPWM(servonum, 0, 165);

  for (uint8_t servonum = 0; servonum < 6; servonum++) {
    // pwm.setPWM(servonum, 0, map(85, 90, -90, maxminArray[servonum][0], maxminArray[servonum][1]));
    pwm.setPWM(servonum, 0, map(servo_angles[servonum] * (180.0/M_PI), 90, -90, maxminArray[servonum][0], maxminArray[servonum][1]));
    Serial.print(servo_angles[servonum] * (180.0/M_PI), 2);
    Serial.print(", ");
  }
  Serial.println();
  
  Serial.println(angle);
  delay(10);
}
