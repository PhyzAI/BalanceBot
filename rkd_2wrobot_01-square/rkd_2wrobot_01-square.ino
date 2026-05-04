void setup() {
    //Setup Channel A motor
    pinMode(12, OUTPUT); //Initiates Motor Channel A pin
    pinMode(9, OUTPUT); //Initiates Brake Channel A pin
  
    //Setup Channel B motor
    pinMode(13, OUTPUT); //Initiates Motor Channel B pin
    pinMode(8, OUTPUT);  //Initiates Brake Channel B pin



  }
  
void loop(){



  //Motor A forward @ full speed
  digitalWrite(12, HIGH); //Establishes forward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, 250);   //Spins the motor on Channel A at full speed

  // //Motor B fwd
  digitalWrite(13, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, 250);    //Spins the motor on Channel B at half speed
  
  delay(1000);
  
  digitalWrite(9, HIGH);  //Engage the Brake for Channel A
  digitalWrite(8, HIGH);  //Engage the Brake for Channel B

  delay(1000);
  
  digitalWrite(9, LOW); //disengage brake
  digitalWrite(8, LOW); 
  delay(1000);
  digitalWrite(9, HIGH);  //Engage the Brake for Channel A
  digitalWrite(8, HIGH);  //Engage the Brake for Channel B
  delay(1000);
  

  digitalWrite(12, LOW); //Establishes reverse direction of Channel A

  digitalWrite(9, LOW); //disengage brake
  digitalWrite(8, LOW); 
  delay(1000);
  digitalWrite(9, HIGH);  //Engage the Brake for Channel A
  digitalWrite(8, HIGH);  //Engage the Brake for Channel B








  // //Motor A forward @ full speed
  // digitalWrite(12, HIGH); //Establishes forward direction of Channel A
  // digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  // analogWrite(3, 250);   //Spins the motor on Channel A at full speed

  // // //Motor B fwd
  // digitalWrite(13, HIGH);  //Establishes backward direction of Channel B
  // digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  // analogWrite(11, 250);    //Spins the motor on Channel B at half speed
  
  // delay(1000);
  
  // digitalWrite(9, HIGH);  //Engage the Brake for Channel A
  // digitalWrite(8, HIGH);  //Engage the Brake for Channel B
  
  // delay(1000);
  
  // digitalWrite(9, LOW);
  // digitalWrite(8, LOW);


  // //Motor A forward @ full speed
  // digitalWrite(12, LOW);  //Establishes backward direction of Channel A
  // digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  // analogWrite(3, 123);    //Spins the motor on Channel A at half speed
  
  // //Motor B forward @ full speed
  // digitalWrite(13, HIGH); //Establishes forward direction of Channel B
  // digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  // analogWrite(11, 255);   //Spins the motor on Channel B at full speed
  
  // delay(3000);
  
  // digitalWrite(9, HIGH);  //Engage the Brake for Channel A
  // digitalWrite(8, HIGH);  //Engage the Brake for Channel B
  
  delay(1000);
}