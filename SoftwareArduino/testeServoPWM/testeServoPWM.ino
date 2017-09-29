#include <PWM.h>

int pos = 0;
int servo = 9;
int32_t frequency = 50;
bool success = false;
int pot = 0;

void setup() {
  InitTimersSafe();
  success = SetPinFrequencySafe(servo, frequency);
  Serial.begin(38400);
  
  if(success) {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);    
  }
} 
 
void loop() {
  if (success) { 
    for(pos = 1; pos <= 25; pos += 1) {
      pwmWrite(servo, pos);
      pot = analogRead(A0);
      Serial.print(pot);
      Serial.print("\t");
      Serial.print(pos);
      Serial.println();
      
      delay(100);
    } 
    for(pos = 25; pos>=1; pos-=1) {
      pwmWrite(servo, pos);
      pot = analogRead(A0);
      Serial.print(pot);
      Serial.print("\t");
      Serial.print(pos);
      Serial.println(); 
      delay(100);
    }
  }
} 

