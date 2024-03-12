#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
double err = 0.00; // proportional present error
double preverr = 0.00; //integral past error
double err_sum = 0.00; // derivative future error

int kp = 1;
double ki = 0.00001;
double kd = 20.00;

double curd_output = 0.00;//currently desired angle
double a_output;
double pid_output = 0.00;

int ena = 10;
int in1 = 9;
int in2 = 8;
int in3 = 7;
int in4 = 6;
int enb = 5;

double speeda = 0.00;
double speedb = 0.00;

long int start_time;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  Wire.begin();
  mpu.begin();
  delay(1000);
  mpu.calcOffsets();

  pinMode(in1, OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(ena,OUTPUT);
  pinMode(enb,OUTPUT);

  start_time = millis(); // to record the time the loop began
}
//a function to redirecitt the car by an angle given certain conditions
void loop(){
  if (millis()- start_time >= 4000 && err>5){
    curd_output += 90.00;//currently desired angle
    // show the angle the robot is to move if the aboove condition is met. the car is to move in a rectangular form
    start_time = millis();
  }
  pidcontrol(curd_output); 
}
//a function to get angle form the gyroscope
float get_angle(){
  double angz = 0.00;
  mpu.update();
	return mpu.getAngleZ();
}

double d_output = 0.00; //desired angle
//a function to calculate errors and pid values given the current angles
void pidcontrol(double d_output) {
preverr = err;
err_sum = err_sum + err;
err_sum = constrain(err_sum, -1000.00, 1000.00);

err = d_output - get_angle(); // desired angle - angle form the gyroscope
pid_output = kp * err + ki * err_sum + kd * (err - preverr); // control given by the mcu to the robot car
pid_output = constrain(pid_output, -100, 100);
//negatve means speeda should be more than speedb and vice versa 

if (pid_output >= 0){
speedb = 150 - (180.00+150.00)*(pid_output/100);
speeda = 120 + (180.00-150.00)*(pid_output/100);
}
else {
speedb = 120 - (180.00-150.00)*(pid_output/100);
speeda = 150 + (0.00+150.00)*(pid_output/100);
}
forward(speeda,speedb);
}
//a function that dictates how the car moves
void forward(int pwm_left,int pwm_right){
analogWrite(ena, abs(pwm_left));
analogWrite(enb,abs(pwm_right));
if (pwm_left >0){
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
}
else{
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
}

if (pwm_right >0){
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);
}
else{
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
}
}


