/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Includes and Definitions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
//#include "SR04.h"//define the library of ultrasonic sensor
#define TRIG_PIN 12// set the signal output of ultrasonic sensor to D12 
#define ECHO_PIN 13//set the signal input of ultrasonic sensor to D13 
#define SCL_Pin  A5  //Set the clock pin to A5
#define SDA_Pin  A4  //Set data pin to A4
const int servopin = A3;//set the pin of servo to A3 
int left_ctrl = 2;//define the direction control pins of group B motor
int left_pwm = 5;//define the PWM control pins of group B motor
int right_ctrl = 4;//define the direction control pins of group A motor
int right_pwm = 6;//define the PWM control pins of group A motor

//SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Global Variables
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


Adafruit_MPU6050 mpu;

float step_size=0.1;
float d_trav;
int iters = 10;
int count = 1;


void setup(){
  init_serial();
  init_motors();
  init_mpu();

  
}


//void echo(){
//    if (Serial.available()) {
//  //character = Serial.read();
////  Serial.println((char)character);
//  //Serial.println((char)character);
////    String data = Serial.readStringUntil(' ');
//    String data = Serial.readString();
//    Serial.println("Got it");
//    Serial.println(data);
//  
////    Serial.print(data);
//    }
//}

void loop(){
  if (Serial.available()) {
    String data = Serial.readString();
    Serial.print("Message RCVD : ");
    Serial.println(data);
    count = 1;
    //if(data == "go"){
      while(count <= iters){
          Serial.println("-- Left --");
          car_left(100);
          get_mpu_data();
          delay(50);
          Serial.println("-- Stopped --");
          car_Stop();
          get_mpu_data();
          delay(50);
          count++;
      }
    //}
    
//    Serial.println(data);
  
//    Serial.print(data);
    }
//  car_left(50);
//  get_mpu_data();
//  delay(10);
  
 
}


void init_serial(){
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
}

void init_mpu(){

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  
}

void get_mpu_data(){
   /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}

void init_motors(){
  pinMode(left_ctrl,OUTPUT);//set direction control pins of group B motor to OUTPUT
  pinMode(left_pwm,OUTPUT);//set PWM control pins of group B motor to OUTPUT
  pinMode(right_ctrl,OUTPUT);//set direction control pins of group A motor to OUTPUT
  pinMode(right_pwm,OUTPUT);//set PWM control pins of group A motor to OUTPUT
}

void car_front()//car goes forward
{
    digitalWrite(left_ctrl,LOW);
    analogWrite(left_pwm,155);
    digitalWrite(right_ctrl,LOW);
    analogWrite(right_pwm,155);
    delay(step_size);
    d_trav += step_size;
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
    analogWrite(left_pwm, 205);  
    digitalWrite(right_ctrl, HIGH);
    analogWrite(right_pwm, 255);
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
    digitalWrite(left_ctrl, HIGH);
    analogWrite(left_pwm, 255);
    digitalWrite(right_ctrl, LOW);
    analogWrite(right_pwm, 205);
    delay(len);

    analogWrite(left_pwm,0);//set the PWM control speed of B motor to 0
    analogWrite(right_pwm,0);//set the PWM control speed of A motor to 0
    //stop
    delay(10);//delay in 2s
  //}
}
void car_Stop()//stop
{
  digitalWrite(left_ctrl,LOW);
  analogWrite(left_pwm,0);
  digitalWrite(right_ctrl,LOW);
  analogWrite(right_pwm,0);
}
