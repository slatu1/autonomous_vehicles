// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

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

void loop(){
  if (Serial.available()) {
  //character = Serial.read();
//  Serial.println((char)character);
  //Serial.println((char)character);
//    String data = Serial.readStringUntil(' ');
    String data = Serial.readString();
    Serial.println(data);
    int d_ta[6];
    String temp;
    int index = 0;
    
    for(int i = 0; i < data.length(); i++){
      if(data[i] == ' ' || data[i] == "/r"){
        //Serial.println(temp);
        d_ta[index] = temp.toInt();
        index +=1;
        temp = "";
      }
      else{
        temp.concat(data[i]);
      }
//    Serial.print(data);
    }
    for(int k=0; k<6; k++){
      Serial.print("Parameter ");
      Serial.print(k);
      Serial.print(": ");
      Serial.println(d_ta[k]);
    }

    switch(d_ta[0]){
      case 1:
            Serial.println("Move Left");
            car_left(d_ta[1]);
            break;
      case 2:
            Serial.println("Move Right");
            car_right(d_ta[1]);
            break;           
    }
}
}


// void loop() {

//     if (Serial.available()) {
//     ang = Serial.readString();
//     Serial.println(ang);
//     car_right(ang.toInt());
//     car_Stop();
//   }

//   // /* Get new sensor events with the readings */
//   // sensors_event_t a, g, temp;
//   // mpu.getEvent(&a, &g, &temp);

//   // /* Print out the values */
//   // Serial.print("Acceleration X: ");
//   // Serial.print(a.acceleration.x);
//   // Serial.print(", Y: ");
//   // Serial.print(a.acceleration.y);
//   // Serial.print(", Z: ");
//   // Serial.print(a.acceleration.z);
//   // Serial.println(" m/s^2");

//   // Serial.print("Rotation X: ");
//   // Serial.print(g.gyro.x);
//   // Serial.print(", Y: ");
//   // Serial.print(g.gyro.y);
//   // Serial.print(", Z: ");
//   // Serial.print(g.gyro.z);
//   // Serial.println(" rad/s");

//   // Serial.print("Temperature: ");
//   // Serial.print(temp.temperature);
//   // Serial.println(" degC");

//   // Serial.println("");
//   // delay(500);
// }

void car_left(float len)//car turns left
{
  //float rads = 0;
  //while(rads != angle){
    digitalWrite(left_ctrl, LOW);
    analogWrite(left_pwm, 200);  
    digitalWrite(right_ctrl, HIGH);
    analogWrite(right_pwm, 250);
    delay(len);

    // analogWrite(left_pwm,0);//set the PWM control speed of B motor to 0
    // analogWrite(right_pwm,0);//set the PWM control speed of A motor to 0
    // //stop
    // delay(10);//delay in 2s
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

    // analogWrite(left_pwm,0);//set the PWM control speed of B motor to 0
    // analogWrite(right_pwm,0);//set the PWM control speed of A motor to 0
    // //stop
    // delay(10);//delay in 2s
  //}
}