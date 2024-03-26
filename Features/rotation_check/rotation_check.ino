//*******************************************************************************
/*
 keyestudio 4wd BT Car
 lesson 13
 Avoiding Car
 http://www.keyestudio.com
*/ 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Includes and Definitions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include "SR04.h"//define the library of ultrasonic sensor
#define TRIG_PIN 12// set the signal output of ultrasonic sensor to D12 
#define ECHO_PIN 13//set the signal input of ultrasonic sensor to D13 
#define SCL_Pin  A5  //Set the clock pin to A5
#define SDA_Pin  A4  //Set data pin to A4
#include <Wire.h>
//control pins for left and right motors
const int leftSpeed = 9; //means pin 9 on the Arduino controls the speed of left motor
const int rightSpeed = 5;
const int left1 = 3; //left 1 and left 2 control the direction of rotation of left motor
const int left2 = 2;
const int right1 = 8;
const int right2 = 4;

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ; //linear acceleration
float GyroX, GyroY, GyroZ; //angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

const int maxSpeed = 255; //max PWM value written to motor speed pin. It is typically 255.
const int minSpeed = 160; //min PWM value at which motor moves
float angle; //due to how I orientated my MPU6050 on my car, angle = roll

const int servopin = A3;//set the pin of servo to A3 
int left_ctrl = 2;//define the direction control pins of group B motor
int left_pwm = 5;//define the PWM control pins of group B motor
int right_ctrl = 4;//define the direction control pins of group A motor
int right_pwm = 6;//define the PWM control pins of group A motor

SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Global Variables
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

long distance,a1,a2;//define three distance
float step_size=0.1;
float x_init;
float y_init;
float x_pos;
float y_pos;
float theta;
float active_wp[2] = {0,0};
String ang;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
   Serial.begin(9600);//start serial comms with set baud
   
  pinMode(left_ctrl,OUTPUT);//set direction control pins of group B motor to OUTPUT
  pinMode(left_pwm,OUTPUT);//set PWM control pins of group B motor to OUTPUT
  pinMode(right_ctrl,OUTPUT);//set direction control pins of group A motor to OUTPUT
  pinMode(right_pwm,OUTPUT);//set PWM control pins of group A motor to OUTPUT
  pinMode(TRIG_PIN, OUTPUT); //Set the trig pin to output
  pinMode(ECHO_PIN, INPUT); //Set the echo pin to input
//  servopulse(servopin,90);//the angle of servo is 90 degree
//  delay(300);
  pinMode(SCL_Pin,OUTPUT);// Set the clock pin to output
  pinMode(SDA_Pin,OUTPUT);//Set the data pin to output

   Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Call this function if you need to get the IMU error values for your module
//  calculateError();
  delay(20);
  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(rightSpeed, OUTPUT);
  currentTime = micros();

  x_init = 0;
  y_init = 0;
}


void loop()
 {
//  car_right(3000);


  if (Serial.available()) {
    ang = Serial.readString();
    Serial.println(ang);
    car_right(ang.toInt());
    car_Stop();
  }
  
}

void car_front()//car goes forward
{
    digitalWrite(left_ctrl,LOW);
    analogWrite(left_pwm,155);
    digitalWrite(right_ctrl,LOW);
    analogWrite(right_pwm,155);
    delay(step_size);
//    d_trav += step_size;
}


void car_back()//go back
{
  
  digitalWrite(left_ctrl,LOW);
  analogWrite(left_pwm,200);
  digitalWrite(right_ctrl,LOW);
  analogWrite(right_pwm,200);
}
void car_left(float len)//car turns left
{
  //float rads = 0;
  //while(rads != angle){
    digitalWrite(left_ctrl, LOW);
    analogWrite(left_pwm, 200);  
    digitalWrite(right_ctrl, HIGH);
    analogWrite(right_pwm, 250);
    delay(len);

    analogWrite(left_pwm,0);//set the PWM control speed of B motor to 0
    analogWrite(right_pwm,0);//set the PWM control speed of A motor to 0
    //stop
    delay(10);//delay in 2s
    //rads += 1;
  //}
}
void car_right(float len)//car turns right
{
//  float rads = 0;
//  while(rads != angle){
   // === Read accelerometer (on the MPU6050) data === //
    readAcceleration();
    // Calculating Roll and Pitch from the accelerometer data
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; //AccErrorX is calculated in the calculateError() function
    digitalWrite(left_ctrl, HIGH);
    analogWrite(left_pwm, 255);
    digitalWrite(right_ctrl, LOW);
    analogWrite(right_pwm, 205);
    while(accAngleX < len){
      delay(10);
       // === Read accelerometer (on the MPU6050) data === //
      readAcceleration();
      // Calculating Roll and Pitch from the accelerometer data
      accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; //AccErrorX is calculated in the calculateError() function
      Serial.println(accAngleX);
    }
    
    

//    analogWrite(left_pwm,0);//set the PWM control speed of B motor to 0
//    analogWrite(right_pwm,0);//set the PWM control speed of A motor to 0
//    //stop
//    delay(10);//delay in 2s
  //}
}
void car_Stop()//stop
{
  digitalWrite(left_ctrl,LOW);
  analogWrite(left_pwm,0);
  digitalWrite(right_ctrl,LOW);
  analogWrite(right_pwm,0);
  delay(2000);
}

void readAcceleration() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the MPU6050 datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
}
//*******************************************************************************
