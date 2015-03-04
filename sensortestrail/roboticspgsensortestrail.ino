
int motor1Pin1 = 3; // pin 2 on L293D
int motor1Pin2 = 4; // pin 7 on L293D
int enablePin = 9; // pin 1 on L293D
int analogInPin = A0;


    int sensorValue = 0;        // value read from the pot
    int outputValue = 0;        // value output to the PWM (analog out)
void setup() {
  
    Serial.begin(9600); 


    // set all the other pins you're using as outputs:
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    //pinMode(enablePin, OUTPUT);

    // set enablePin high so that motor can turn on:
    digitalWrite(enablePin, HIGH);
    

}

void loop() {
  sensorValue = analogRead(analogInPin);            
    // if the switch is high, motor will turn on one direction:
  if (sensorValue<500) {
      digitalWrite(motor1Pin1, LOW); // set pin 2 on L293D low
      digitalWrite(motor1Pin2, HIGH); // set pin 7 on L293D high
      // map it to the range of the analog out:
      outputValue = map(sensorValue, 0, 500, 255, 0);  
      // change the analog out value:
      analogWrite(enablePin, outputValue);           

    }
    // if the switch is low, motor will turn in the opposite direction:
  else {
      digitalWrite(motor1Pin1, HIGH); // set pin 2 on L293D high
      digitalWrite(motor1Pin2, LOW); // set pin 7 on L293D low
      // map it to the range of the analog out:
      outputValue = map(sensorValue, 500, 1023, 0, 255);  
      // change the analog out value:
      analogWrite(enablePin, outputValue);           

    }          

  // print the results to the serial monitor:
  Serial.print("sensor = " );                       
  Serial.print(sensorValue);      
  Serial.print("\t output = ");      
  Serial.println(outputValue);   

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);                     
}
