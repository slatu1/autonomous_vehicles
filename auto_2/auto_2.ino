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
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <time.h>
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

Adafruit_MPU6050 mpu;
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
float d_trav= 0;
float theta_d = 90;
float theta = 0;
float active_wp[2] = {0,0};
float xg;
float yg;
float xd;
float yd;
float xr, yr;
float servo_pos = 75;
int done = 0;
int num_wp = 10;
String input;
int detour_active = 0;
float* current_pos[2];
//float wp_map[num_wp][2]={{0,0},{2,3},{0,4},{2,4},{3,5},{1,1},{0,0},{2,3},{0,4},{2,4}};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  init_serial();
  init_motors();
  init_mpu();
  init_ultrasound();
  init_leds();
  x_init = 0;
  y_init = 0;
}

//void localize(){
//   // determine distance traveled in each direction
//   x_pos = d_trav*cos(theta) + x_init;
//   y_pos = (d_trav*sin(theta) + y_init)*-1;
//   Serial.println("Localizing");
//   Serial.print("x_pos = ");
//   Serial.println(x_pos,3);
//   Serial.print("y_pos = ");
//   Serial.println(y_pos,3);
//   //return d;
//}

float localize_x(float x, float t, float d){
    // determine distance traveled in each direction   
    float temp_rad, temp_x;//, temp_y;
    int quad;
//    # quad 1
    if ((t > 0) && (t < 45))
      quad = 1;
    else if((t > 45) && (t < 90))
      quad = 1;
    //# quad 2
    else if ((t > 90) && (t < 135))
      quad = 2;
    else if  ((t > 135) && (t < 180))
      quad = 2;
   // # quad 3
    else if ((t > 180) && (t < 225))
      quad = 3;
    else if  ((t > 225) && (t < 270))
      quad = 3;
   // # quad 4
    else if ((t > 270) && (t < 315))
      quad = 4;
    else if  ((t > 315) && (t < 360)) 
      quad = 4;

   // # convert temp_tetha to rad for python math
    temp_rad = t*(PI/180);
//    print("\n temp rad = " + str(temp_rad));
    
    //# temp_deg = temp_rad*(180/np.pi)
    //# print("\n temp deg = " + str(temp_deg) + "\n")
      
    temp_x = abs(d*(cos(temp_rad)));
//    temp_y = abs(d*(sin(temp_rad)));
    Serial.print("\ndist_x = ");
    Serial.println(temp_x, 4);// + "\tdist_y = " + str(temp_y));
    Serial.print("quad x = ");
    Serial.println(quad);
    switch(quad){
      case 1:
            temp_x += x;
//            temp_y += y;
            break;
      case 2: 
            temp_x = x - temp_x;
//            temp_y += y;
            break; 
      case 3:
            temp_x = x - temp_x;
//            temp_y = y - temp_y; 
            break; 
      case 4:
            temp_x += x;
            //# temp_y -= y
//            temp_y = y - temp_y; 
            break;
    }
    //# print("\ndist_x = " + str(temp_x) + "\tdist_y = " + str(temp_y))
//    print("\nQuad - " + str(quad) + "\n");
    //# print("\ntheta = " + str(temp_theta) + "\nobs_x = " + str(temp_x) + "\nobs_y = " + str(temp_y) + "\n")             
    
//    x_pos = temp_x;
//    y_pos = temp_y ;   
//    print("Localizing \n x_pos = " + str(x_pos) + "\ty_pos = " + str(y_pos) + "\ttheta_d " + str(t) + "\n");
//    wp_buff[0]=x_pos;
//    wp_buff[1]=y_pos;
    return temp_x; 
}

float localize_y(float y, float t, float d){
    // determine distance traveled in each direction   
    float temp_rad, temp_y;
    int quad;
//    # quad 1
    if ((t > 0) && (t < 45))
      quad = 1;
    else if((t > 45) && (t < 90))
      quad = 1;
    //# quad 2
    else if ((t > 90) && (t < 135))
      quad = 2;
    else if  ((t > 135) && (t < 180))
      quad = 2;
   // # quad 3
    else if ((t > 180) && (t < 225))
      quad = 3;
    else if  ((t > 225) && (t < 270))
      quad = 3;
   // # quad 4
    else if ((t > 270) && (t < 315))
      quad = 4;
    else if  ((t > 315) && (t < 360)) 
      quad = 4;

   // # convert temp_tetha to rad for python math
    temp_rad = t*(PI/180);
//    print("\n temp rad = " + str(temp_rad));
    
    //# temp_deg = temp_rad*(180/np.pi)
    //# print("\n temp deg = " + str(temp_deg) + "\n")
      
//    temp_x = abs(d*(cos(temp_rad)));
    temp_y = abs(d*(sin(temp_rad)));
//    print("\ndist_x = " + str(temp_x) + "\tdist_y = " + str(temp_y));
    Serial.print("quad y = ");
    Serial.println(quad);    
    switch(quad){
      case 1:
//            temp_x += x;
            temp_y += y;
            break;
      case 2:
//            temp_x = x - temp_x;
            temp_y += y;
            break; 
      case 3:
//            temp_x = x - temp_x;
            temp_y = y - temp_y; 
            break; 
      case 4:
//            temp_x += x;
            //# temp_y -= y
            temp_y = y - temp_y; 
            break;
    }
    //# print("\ndist_x = " + str(temp_x) + "\tdist_y = " + str(temp_y))
//    print("\nQuad - " + str(quad) + "\n");
    //# print("\ntheta = " + str(temp_theta) + "\nobs_x = " + str(temp_x) + "\nobs_y = " + str(temp_y) + "\n")             
    
//    x_pos = temp_x;
//    y_pos = temp_y ;   
//    print("Localizing \n x_pos = " + str(x_pos) + "\ty_pos = " + str(y_pos) + "\ttheta_d " + str(t) + "\n");
//    wp_buff[0]=x_pos;
//    wp_buff[1]=y_pos;
    return temp_y;  
}

//void localize(float x, float y, float t, float d, float* wp_buff[2]){
//    // determine distance traveled in each direction   
//    float temp_rad, temp_x, temp_y;
//    int quad;
////    # quad 1
//    if ((t > 0) && (t < 45))
//      quad = 1;
//    else if((t > 45) && (t < 90))
//      quad = 1;
//    //# quad 2
//    else if ((t > 90) && (t < 135))
//      quad = 2;
//    else if  ((t > 135) && (t < 180))
//      quad = 2;
//   // # quad 3
//    else if ((t > 180) && (t < 225))
//      quad = 3;
//    else if  ((t > 225) && (t < 270))
//      quad = 3;
//   // # quad 4
//    else if ((t > 270) && (t < 315))
//      quad = 4;
//    else if  ((t > 315) && (t < 360)) 
//      quad = 4;
//
//   // # convert temp_tetha to rad for python math
//    temp_rad = t*(PI/180);
////    print("\n temp rad = " + str(temp_rad));
//    
//    //# temp_deg = temp_rad*(180/np.pi)
//    //# print("\n temp deg = " + str(temp_deg) + "\n")
//      
//    temp_x = abs(d*(cos(temp_rad)));
//    temp_y = abs(d*(sin(temp_rad)));
////    print("\ndist_x = " + str(temp_x) + "\tdist_y = " + str(temp_y));
//    switch(quad){
//      case 1:
//            temp_x += x;
//            temp_y += y;
//            break;
//      case 2:
//            temp_x = x - temp_x;
//            temp_y += y;
//            break; 
//      case 3:
//            temp_x = x - temp_x;
//            temp_y = y - temp_y; 
//            break; 
//      case 4:
//            temp_x += x;
//            //# temp_y -= y
//            temp_y = y - temp_y; 
//            break;
//    }
//    //# print("\ndist_x = " + str(temp_x) + "\tdist_y = " + str(temp_y))
////    print("\nQuad - " + str(quad) + "\n");
//    //# print("\ntheta = " + str(temp_theta) + "\nobs_x = " + str(temp_x) + "\nobs_y = " + str(temp_y) + "\n")             
//    
////    x_pos = temp_x;
////    y_pos = temp_y ;   
////    print("Localizing \n x_pos = " + str(x_pos) + "\ty_pos = " + str(y_pos) + "\ttheta_d " + str(t) + "\n");
//    wp_buff[0]=x_pos;
//    wp_buff[1]=y_pos;
////    return x_pos, y_pos  
//}

void loop()
 {
    if (Serial.available()) {
        input = Serial.readString();
        Serial.println(input);

        int d_ta[6];
        String temp;
        int index = 0;

        for(int i = 0; i < input.length(); i++){
            if(input[i] == ' ' || input[i] == "/r"){
                //Serial.println(temp);
                d_ta[index] = temp.toInt();
                index +=1;
                temp = "";
            }
            else{
                temp.concat(input[i]);
            }
        //    Serial.print(data);
        }
        for(int k=0; k<6; k++){
            Serial.print("Parameter ");
            Serial.print(k);
            Serial.print(": ");
            Serial.println(d_ta[k]);
        }

        // rcv goal point
        // go to goal
        // report obstacle
        // rcv request for data
        // send data
        // rcv detour coordinates
        // report when detour complete
        // rcv cmd to goal -- or not
        // move toward goal
        // repeat until destination reached and report
        switch(d_ta[0]){
            case 1: 
                    Serial.println("Goal RCVD");
                    xr = float(d_ta[1])/100.0;
                    yr = float(d_ta[2])/100.0;
                    if(detour_active == 0){
                      xg = xr;
                      yg = xr;
                      go();
                    }
                    else{
                      xd = xr;
                      yd = yr;
                      go();
                    }
                    break;
            case 2:
                    Serial.println("Data Req RCVD");
                    // x_now
                    Serial.print(x_pos);
                    Serial.print(" ");
                    // y_now
                    Serial.print(y_pos);
                    Serial.print(" ");
                    // theta
                    Serial.println(theta);
                    break;   
            case 3:
                    scan();
                    break;                                             
        }
        // switch(d_ta[0]){
        //     case 1: 
        //             Serial.println("Move Forward");
        //             // car_front
        //             break;
        //     case 2:
        //             Serial.println("Move Back");
        //             break; 
        //     case 3:
        //             Serial.println("Turn Right");
        //             break;   
        //     case 4:
        //             Serial.println("Turn Left");
        //             break; 
        //     case 5:
        //             Serial.println("Check Surroundings");
        //             // if(avoid()){
        //             //     report_obstacle();
        //             // }
        //             break;   
        //     case 6:
        //             Serial.println("Report");
        //             break;                                        
        // }


    // destination = Serial.readString();
    // Serial.print("Received : ");
    // Serial.println(destination);

    // if(destination<0 | destination>num_wp){
    //     Serial.println("Wapypoint not found");
    // }
    // else {
    //     Serial.print("Waypoint located at : ");
    //     Serial.print(wp_map[destination]);
    //     go(wp_map[destination][0],wp_map[destination][1]);
    // }

  }
  
}




String verify(){
  car_stop();
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

void go(){
  float wp_buffer;
  int stat = 1;
  Serial.print("Waypoint = {");
  Serial.print(xr,3);
  Serial.print(",");
  Serial.print(yr,3);
  Serial.println("}");
  while(done != 1){
    // orient
    Serial.println("- Setting Orientation -");
    delay(200);
    theta = ((atan((yr-y_pos)/(xr-x_pos))*180)/PI);
    theta_d -= theta;
    Serial.print("theta = ");
    Serial.println(theta,3);
    if(theta < 0){
      car_left(abs(theta_d));
    }
    else{
      car_right(abs(theta_d));
    }
    car_stop();
    Serial.println("- Orientation Achieved -");
    delay(1500);
    //determine distance
    Serial.println("- Calculating Distance to Waypoint -"); // from initial position
    float d_rem = sqrt(((xr-x_init)*(xr-x_init))+((yr-y_init)*(yr-y_init)));
    Serial.print("d_rem = ");
    Serial.println(d_rem);
    //loop until waypoint is reached 
    if(detour_active == 1){
      Serial.println("- Detour -");
    }
    else{
      Serial.println("- Traveling to Waypoint -");
    }
//    while(d_rem > 0.7){            
//      //check distance from objects (if objects need to be avoided)
//      int obj_detected = avoid(); 
//      if(obj_detected == 1){
//        Serial.println("Obstacle Detected");
//        return 1;
//      }
    stat = car_front(d_rem);//go forward
    car_stop();
    if(stat == 1){
      Serial.println("Obstacle Detected");
      return;
    }
    if(detour_active == 0){
      Serial.println("Destination Reached");
      x_init = x_pos;
      y_init = y_pos;
      return;
    }
    else{
      Serial.println("- Detour Complete -");
      detour_active = 0;
      xr = xg;
      yr = yg;
      delay(1000);
      go();
    }
    
  }
  //avoid();//run the main program
}

void scan(){
  int theta = 160;
  int ack = 0;
  // send start signal
  Serial.println("Ready");
  // 20 deg intervals from -90 to +90
  // send angle and distance to raspi
  while (theta >= 0){
    ack = 0; // reset ack
    servopulse(servopin,theta);//servo rotates to 160°
    delay(50);
    float d=sr04.Distance();//measure the distance
    // send theta and distance to raspi
    switch(theta){
      case 160:
              Serial.print("-90");
              break;
      case 140:
              Serial.print("-80");
              break;       
      case 130:
              Serial.print("-60");
              break;
      case 120:
              Serial.print("-40");
              break;       
      case 100:
              Serial.print("-20");
              break;
      case 80:
              Serial.print("0");
              break;  
      case 60:
              Serial.print("50");
              break;
      case 40:
              Serial.print("60");
              break;   
      case 20:
              Serial.print("70");
              break;
      case 0:
              Serial.print("90");
              break;                                                                               
    }
    Serial.print(" ");
    Serial.println(d);
    // wait for acknoledgement from raspi
    while(ack != 1){
      if (Serial.available()){
        ack = Serial.readString().toInt();;
        Serial.println(ack);
      }
    }

    if(theta == 0){
      Serial.println("Done");
      servopulse(servopin,servo_pos);//the angle of servo is 90 degree
      return;
    }
    else{
      theta -= 20; // decrement 10 degrees for next meaurement   
    }
  }
}

//read data from ultrasonic sensor and determine if an obstacle exists
int avoid()
{
  distance=sr04.Distance(); //obtain the value detected by ultrasonic sensor 

  if((distance < 25)&&(distance != 0))//if the distance is greater than 0 and less than 15 
  {
    Serial.println("Object Detected");  // report to raspi
    car_stop();//stop
    matrix_display(clear);
    matrix_display(STOP01);//show stop pattern
    return 1;
  }
  else//otherwise
  {
    matrix_display(clear);
    matrix_display(front);  // show forward pattern
    return 0;
  }
}


int car_front(float dist)//car goes forward
{
    sensors_event_t a, g, temp;
//    float len_rads = len * 3.14159/180;
    float previous_t,curr_t = 0;
    long int dt = 10;
    float inc = 1;
    float stp = 0;
    
    digitalWrite(left_ctrl,LOW);
    analogWrite(left_pwm,155);
    digitalWrite(right_ctrl,LOW);
    analogWrite(right_pwm,155);
    delay(step_size);
    float dist_rem = dist;

    curr_t = millis();
    while(dist_rem > 0){            
      //check distance from objects (if objects need to be avoided)
      int obj_detected = avoid(); 
      if(obj_detected == 1){
        return 1;
      }
      if(dist_rem >= 0 && dist_rem <= 0.0007){
       return 0;
      }
      delay(inc);
      previous_t = curr_t;
      curr_t = millis();
      float dt = (curr_t - previous_t)/1000; // s
      /* Get new sensor events with the readings */
      mpu.getEvent(&a, &g, &temp);

      Serial.print("Acceleration X: ");
      Serial.println(a.acceleration.x);

      float vel = float(a.acceleration.x)*float(dt);
      stp = vel*dt;
      dist_rem -= stp;

      x_pos = localize_x(x_pos,abs(theta_d),stp); //update x and y pos
      y_pos = localize_y(y_pos,abs(theta_d),stp); //update x and y pos
      float d_check = sqrt(((xr-x_pos)*(xr-x_pos))+((yr-y_pos)*(yr-y_pos))); // distance from current position
      Serial.print("x = ");
      Serial.println(x_pos*100.0,3);
      Serial.print("y = ");
      Serial.println(y_pos*100.0,3);
      Serial.print("d = ");
      Serial.println(dist_rem,3);
      Serial.print("d(check) = ");
      Serial.println(d_check,3);
//      car_front();//go forward
//      localize(x_pos,y_pos,theta,step_size,&wp_buffer); //update x and y pos
//      x_pos = localize_x(x_pos,theta,step_size); //update x and y pos
//      y_pos = localize_y(y_pos,theta,step_size); //update x and y pos
//      d_rem = sqrt(((xr-x_pos)*(xr-x_pos))+((yr-y_pos)*(yr-y_pos))); // distance from current position
//      Serial.print("d = ");
//      Serial.println(d_rem,3);
    }
    
}


void car_back()//go back
{
  
  digitalWrite(left_ctrl,LOW);
  analogWrite(left_pwm,200);
  digitalWrite(right_ctrl,LOW);
  analogWrite(right_pwm,200);
}

float car_left(float len)//car turns left
{
  sensors_event_t a, g, temp;
  Serial.println(len);
  float len_deg = 0;
  float rads = 0;
  long int dt = 10;
  float inc = 1;
  float len_rads = len * 3.14159/180;
  float previous_t,curr_t = 0;
  digitalWrite(left_ctrl, LOW);
  analogWrite(left_pwm, 200);  
  digitalWrite(right_ctrl, HIGH);
  analogWrite(right_pwm, 200);
  curr_t = millis();
  while(len_deg <= len + 5){
    if(len_deg >= len-5 && len_deg <= len + 5){
       return len_deg;
    }
    delay(inc);
    previous_t = curr_t;
    curr_t = millis();
    float dt = (curr_t - previous_t)/1000; // s
    /* Get new sensor events with the readings */
    mpu.getEvent(&a, &g, &temp);
  
    /* Print out the values */
//    Serial.print(g.gyro.z);
//    Serial.println(" rad/s");
    rads += float(g.gyro.z)*float(dt);
//    Serial.print(" dt(s) = ");
//    Serial.println(dt);
//    Serial.print(" rad = ");
//    Serial.println(rads);
    len_deg = float(rads)*180.0/3.14159;
    Serial.print(" deg = ");
    Serial.println(len_deg);
  }

  return len_deg;
}

float car_right(float len)//car turns right
{
  sensors_event_t a, g, temp;
  Serial.println(len);
  float len_deg = 0;
  float rads = 0;
  long int dt = 10;
  float inc = 1;
  float len_rads = len * 3.14159/180;
  float previous_t,curr_t = 0;
  digitalWrite(left_ctrl, HIGH);
  analogWrite(left_pwm, 200);  
  digitalWrite(right_ctrl, LOW);
  analogWrite(right_pwm, 200);
  curr_t = millis();
  while(len_deg <= len + 5){
    if(len_deg >= len-5 && len_deg <= len + 5){
       return len_deg;
    }
    delay(inc);
    previous_t = curr_t;
    curr_t = millis();
    float dt = (curr_t - previous_t)/1000; // s
    /* Get new sensor events with the readings */
    mpu.getEvent(&a, &g, &temp);
  
    /* Print out the values */
//    Serial.print(g.gyro.z);
//    Serial.println(" rad/s");
    rads += float(g.gyro.z)*float(dt);
//    Serial.print(" dt(s) = ");
//    Serial.println(dt);
//    Serial.print(" rad = ");
//    Serial.println(rads);
    len_deg = -1*(float(rads)*180.0/3.14159);
    Serial.print(" deg = ");
    Serial.println(len_deg);
  }

  return len_deg;
}

void car_stop()//stop
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

void init_motors(){
  pinMode(left_ctrl,OUTPUT);//set direction control pins of group B motor to OUTPUT
  pinMode(left_pwm,OUTPUT);//set PWM control pins of group B motor to OUTPUT
  pinMode(right_ctrl,OUTPUT);//set direction control pins of group A motor to OUTPUT
  pinMode(right_pwm,OUTPUT);//set PWM control pins of group A motor to OUTPUT
}

void init_ultrasound(){
    pinMode(TRIG_PIN, OUTPUT); //Set the trig pin to output
    pinMode(ECHO_PIN, INPUT); //Set the echo pin to input
    servopulse(servopin,servo_pos);//the angle of servo is 90 degree
    delay(300);
}

void init_leds(){
  pinMode(SCL_Pin,OUTPUT);// Set the clock pin to output
  pinMode(SDA_Pin,OUTPUT);//Set the data pin to output
  matrix_display(clear);
}
