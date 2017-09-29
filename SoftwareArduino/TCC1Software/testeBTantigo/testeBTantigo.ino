/*The command AT disconects the bluetooth in case it is connected
Clear the serial buffer before cheking the message >> mySerial.flsuh()
BLUETOOTH CARRO ==> 8818,56,6898EB    ,1F00,7FFF
Protocol of Misubishi Pagero TR4 ==> ISO 14230-4 (KWP FAST)
*/

#include <SoftwareSerial.h>
SoftwareSerial mySerial(7, 8); // RX, TX
const int botao = 9;     // the number of the pushbutton pin
//const int key = 10;     // the number of the pushbutton pin
String check = "";
char c;
int flag = 0;


void setup()
{
  pinMode(botao, INPUT);
  //pinMode(key, OUTPUT);    // The Module does not need to be taken out form AT Mode! Once linked, it can mantain comunication at AT Mode
  //digitalWrite(key,HIGH);

  // Open serial communications and wait for port to open:
  Serial.begin(38400);
  while (!Serial) {
  // initialize the pushbutton pin as an input:

}

  // set the data rate for the SoftwareSerial port
  mySerial.begin(38400);
  delay(800);
  mySerial.flush(); 
  Serial.flush(); 
}

void loop() // run over and over
{

if (digitalRead(botao) == 1)  
  {
    // The Module does not need to be taken out form AT Mode! Once linked, it can mantain comunication at AT Mode
    //digitalWrite(key,LOW);  // Set HC-05 Com Mode
    if (mySerial.available())
      {  
        delay(800);  
        while (mySerial.available() > 0)  //transfere o buffer do mySerial para a string check
              {
                  c = mySerial.read();
                  check += c;
                  flag = 1;
              }
        if(flag==1) {
          Serial.print(check);
          Serial.write("\n");
          check.remove(0);
          flag = 0;
        }
      }
      
    if (Serial.available())
      {  
        delay(800);  
        while (Serial.available() > 0)  //transfere o buffer do mySerial para a string check
              {
                  c = Serial.read();
                  check += c;
                  flag = 1;
              }
        
          if(flag==1) {
            Serial.print(check);
            Serial.write("\n");
            mySerial.flush();
            //check += "\r\n";
            mySerial.print(check);
            check.remove(0);
            flag = 0;
          }
      }

  }
 
 
else
  {
    // The Module does not need to be taken out form AT Mode! Once linked, it can mantain comunication at AT Mode
    //digitalWrite(key,HIGH);    // Set HC-05 AT Mode
    if (mySerial.available())
      {  
        delay(800);  
        while (mySerial.available() > 0)  //transfere o buffer do mySerial para a string check
              {
                  c = mySerial.read();
                  check += c;
                  flag = 1;
              }
        if(flag==1) {
          Serial.print(check);
          Serial.write("\n");
          check.remove(0);
          flag = 0;
        }
      }
      
    if (Serial.available())
      {  
        delay(800);  
        while (Serial.available() > 0)  //transfere o buffer do mySerial para a string check
              {
                  c = Serial.read();
                  check += c;
                  flag = 1;
              }
        
          if(flag==1) {
            Serial.print(check);
            Serial.write("\n");
            mySerial.flush();
            check += "\r\n";
            mySerial.print(check);
            check.remove(0);
            flag = 0;
          }
      }
    
    
    
  }
 

}

