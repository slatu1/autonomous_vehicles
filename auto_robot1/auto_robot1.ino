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
const int servopin = A3;//set the pin of servo to A3 
int left_ctrl = 2;//define the direction control pins of group B motor
int left_pwm = 5;//define the PWM control pins of group B motor
int right_ctrl = 4;//define the direction control pins of group A motor
int right_pwm = 6;//define the PWM control pins of group A motor

SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

//Array, used to store the data of pattern, can be calculated by yourself or obtained from the modulus tool
unsigned char front[] = {0x00,0x00,0x00,0x00,0x00,0x24,0x12,0x09,0x12,0x24,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char left[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x28,0x10,0x44,0x28,0x10,0x44,0x28,0x10,0x00};
unsigned char right[] = {0x00,0x10,0x28,0x44,0x10,0x28,0x44,0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char STOP01[] = {0x2E,0x2A,0x3A,0x00,0x02,0x3E,0x02,0x00,0x3E,0x22,0x3E,0x00,0x3E,0x0A,0x0E,0x00};
unsigned char clear[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Global Variables
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

long distance,a1,a2;//define three distance
float step_size=0.1;
float x_init;
float y_init;
float x_pos;
float y_pos;
float d_rem;
float d_trav;
float theta;
float active_wp[2] = {0,0};
int done = 0;
String room_num;

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
  matrix_display(clear);

  x_init = 0;
  y_init = 0;
}

void localize(){
   // determine distance traveled in each direction
   x_pos = d_trav*cos(theta) + x_init;
   y_pos = (d_trav*sin(theta) + y_init)*-1;
   Serial.println("Localizing");
   Serial.print("x_pos = ");
   Serial.println(x_pos,3);
   Serial.print("y_pos = ");
   Serial.println(y_pos,3);
   //return d;
}

void loop()
 {
  if (Serial.available()) {
    room_num = Serial.readString();
    Serial.println(room_num);
    if(room_num == "201"){
      active_wp[0] = 20;
      active_wp[1] = 30;
      go(20,30, 0);
    }
    else if(room_num == "203"){
      active_wp[0] = -20;
      active_wp[1] = 70;
      go(-20,70,0);
    }
    else if(room_num == "206"){
      active_wp[0] = 10;//[20,30];
      active_wp[1] = 5;
      go(10,5,0);
    }
    else{
      Serial.println("Invalid room number");
    }
  }
  
}

String verify(){
  car_Stop();
  Serial.println("Check Desitination"); // send to raspi to trigger qr code capture
  int rx = 0;
  String dest;
  while(rx == 0){
    if (Serial.available()){
      dest = Serial.readString();
      rx = 1;
     }
  }
  return dest;
}

void go(float xr, float yr, int det){
  Serial.print("Waypoint = {");
  Serial.print(xr,3);
  Serial.print(",");
  Serial.print(yr,3);
  Serial.println("}");
  while(done != 1){
    // orient
    Serial.println("- Setting Orientation -");
    delay(200);
    theta = (atan((yr-y_pos)/(xr-x_pos))*180)/PI;
    Serial.print("theta = ");
    Serial.println(theta,3);
    if(det == 0){
      if(theta < 0){
        car_left(abs(theta));
      }
      else{
        car_right(abs(theta));
      }
    }
    else{
      if(theta < 0){
        car_left(20);
      }
      else{
        car_right(20);
      }
    }
    Serial.println("- Orientation Achieved -");
    delay(1500);
    //determine distance
    Serial.println("- Calculating Distance to Waypoint -"); // from initial position
    float d_rem = sqrt(((xr-x_init)*(xr-x_init))+((yr-y_init)*(yr-y_init)));
    Serial.print("d_rem = ");
    Serial.println(d_rem);
    //loop until waypoint is reached 
    if(det == 1){
      Serial.println("- Detour -");
    }
    else{
      Serial.println("- Traveling to Waypoint -");
    }
    while(d_rem > 0.7){            
      //check distance from objects (if objects need to be avoided)
      avoid(); 
      car_front();//go forward
      localize(); //update x and y pos
      d_rem = sqrt(((xr-x_pos)*(xr-x_pos))+((yr-y_pos)*(yr-y_pos))); // distance from current position
      Serial.print("d = ");
      Serial.println(d_rem,3);
    }
    if(det == 0){
      String loc = verify();
      if(loc == room_num){
      Serial.println("- Waypoint Reached -");
      x_init = x_pos;
      y_init = y_pos;
      }
      else{
        Serial.println("- Incorrect Location Reached -");
        Serial.print("Robot found at room ");
        Serial.print(loc);
      }
    }
    else{
      Serial.println("- Detour Complete -");
      delay(1000);
      go(active_wp[0],active_wp[1],0);
    }
    
  }
  //avoid();//run the main program
}

//choose a direction to avoid obstacle
void avoid()
{
  distance=sr04.Distance(); //obtain the value detected by ultrasonic sensor 

  if((distance < 15)&&(distance != 0))//if the distance is greater than 0 and less than 10  
  {
    Serial.println("- Object Detected -");
    car_Stop();//stop
    matrix_display(clear);
    matrix_display(STOP01);//show stop pattern
    delay(1000);
    servopulse(servopin,160);//servo rotates to 160°
    delay(500);
    a1=sr04.Distance();//measure the distance
    delay(100);
    servopulse(servopin,0);//rotate to 20 degree
    delay(500);
    a2=sr04.Distance();//measure the distance
    delay(100);
//    servopulse(servopin,70);  //Return to the 90 degree position
//    delay(500);

    if(active_wp[0] < x_pos){
        car_left(1800);//turn left
        matrix_display(clear);
        matrix_display(left);    //display left-turning pattern
        servopulse(servopin,90);//servo rotates to 90 degree
        delay(700); //turn left 700ms
        matrix_display(clear);
        matrix_display(front);  //show forward pattern
        detour(1);
    }
    else if(active_wp[0] > x_pos){
        car_right(1800);//turn right
        matrix_display(clear);
        matrix_display(right);  //display right-turning pattern
        servopulse(servopin,90);//servo rotates to 90 degree
        delay(700);
        matrix_display(clear);
        matrix_display(front);  //show forward pattern
        detour(0);
    }
    else{
      if(a1 > a2  )//compare the distance, if left distance is more than right distance
      {
        car_left(1800);//turn left
        matrix_display(clear);
        matrix_display(left);    //display left-turning pattern
        servopulse(servopin,90);//servo rotates to 90 degree
        delay(700); //turn left 700ms
        matrix_display(clear);
        matrix_display(front);  //show forward pattern
        detour(1);
      }
      else//if the right distance is greater than the left
      {
        car_right(1800);//turn right
        matrix_display(clear);
        matrix_display(right);  //display right-turning pattern
        servopulse(servopin,90);//servo rotates to 90 degree
        delay(700);
        matrix_display(clear);
        matrix_display(front);  //show forward pattern
        detour(0);
      }
    }
      
  }
  else//otherwise
  {
    //car_front(1);//go forward until we fully avoid obstacle and can course correct
    matrix_display(clear);
    matrix_display(front);  // show forward pattern
  }
}

// we have a direction to go to avoid obstacle
// need to move until object is cleared
void detour(int lr){
  // "transition triangle"
  // det theta
  // 0 = turning left (-90deg)
  // 1 = turning right (+90deg)
  Serial.println("- Entering Detour -");
  x_init = x_pos;
  y_init = y_pos;
  d_trav = 0;
  if(lr == 0){
    theta = theta - 70;
  }
  else{
    theta = theta + 70;
  }
  float y_det = x_init + 1;
  float x_det = y_init + 1;
  // store current waypoint choice and pass y_det and x_det to go?
  go(y_det,x_det, 1);
  avoid(); // check if we moved far enough away to clear obstacle, will complete another detour if needed
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

//void car_front(int avoiding)//car goes forward
//{
////  digitalWrite(left_ctrl,LOW);
////  analogWrite(left_pwm,155);
////  digitalWrite(right_ctrl,LOW);
////  analogWrite(right_pwm,155);
////  delay(step_size);
//`// if not avoiding an obstacle, count travel distnace toward destination
//  if (avoiding == 0){
//    digitalWrite(left_ctrl,LOW);
//    analogWrite(left_pwm,155);
//    digitalWrite(right_ctrl,LOW);
//    analogWrite(right_pwm,155);
//    delay(step_size);
//    d_trav += step_size;
//  }
//  else{ // if avoiding an obstacle, continue until object is cleared 
//    
//  }
//}
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
    digitalWrite(left_ctrl, HIGH);
    analogWrite(left_pwm, 250);
    digitalWrite(right_ctrl, LOW);
    analogWrite(right_pwm, 200);
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

void servopulse(int servopin,int myangle)//the running angle of servo
{
  for(int i=0; i<20; i++)
  {
    int pulsewidth = (myangle*11)+350;
    digitalWrite(servopin,HIGH);
    delayMicroseconds(pulsewidth);
    digitalWrite(servopin,LOW);
    delay(20-pulsewidth/1000);
  } 
}

//this function is used for dot matrix display
void matrix_display(unsigned char matrix_value[])
{
  IIC_start();  //the function that calls the data transfer start condition
  IIC_send(0xc0);  //select address

  for (int i = 0; i < 16; i++) //the pattern data is 16 bytes
  {
    IIC_send(matrix_value[i]); //Transmit the data of the pattern
  }
  IIC_end();   //End pattern data transmission
  IIC_start();
  IIC_send(0x8A);  //Display control, select 4/16 pulse width
  IIC_end();
}
//Conditions under which data transmission begins
void IIC_start()
{
  digitalWrite(SDA_Pin, HIGH);
  digitalWrite(SCL_Pin, HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin, LOW);
  delayMicroseconds(3);
  digitalWrite(SCL_Pin, LOW);
}
//Indicates the end of data transmission
void IIC_end()
{
  digitalWrite(SCL_Pin, LOW);
  digitalWrite(SDA_Pin, LOW);
  delayMicroseconds(3);
  digitalWrite(SCL_Pin, HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin, HIGH);
  delayMicroseconds(3);
}
//transmit data
void IIC_send(unsigned char send_data)
{
  for (byte mask = 0x01; mask != 0; mask <<= 1) //Each byte has 8 bits and is checked bit by bit starting at the lowest level
  {
    if (send_data & mask) { //Sets the high and low levels of SDA_Pin depending on whether each bit of the byte is a 1 or a 0
      digitalWrite(SDA_Pin, HIGH);
    } else {
      digitalWrite(SDA_Pin, LOW);
    }
    delayMicroseconds(3);
    digitalWrite(SCL_Pin, HIGH); //Pull the clock pin SCL_Pin high to stop data transmission
    delayMicroseconds(3);
    digitalWrite(SCL_Pin, LOW); //pull the clock pin SCL_Pin low to change the SIGNAL of SDA 
  }
}
//*******************************************************************************
