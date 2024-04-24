
void setup(){
  Serial.begin(9600);

  
}


void echo(){
    if (Serial.available()) {
  //character = Serial.read();
//  Serial.println((char)character);
  //Serial.println((char)character);
//    String data = Serial.readStringUntil(' ');
    String data = Serial.readString();
    Serial.println("Got it");
    Serial.println(data);
  
//    Serial.print(data);
    }
}

void loop(){
  if (Serial.available()) {
    String data = Serial.readString();
    Serial.print("Message RCVD : ");
    Serial.println(data);
//    Serial.println(data);
  
//    Serial.print(data);
    }
   
}
