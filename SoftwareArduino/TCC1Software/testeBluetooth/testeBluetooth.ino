//***********************************************//
//******Car Engine RPM and Shift Light***********//
//******with Arduino, HC-05 Bluetooth Module*****//
//**********and ELM-327 OBDII Bluetooth**********//
//***********************************************//
//**********Designed and Programmed**************//
//************by Kostas Kokoras******************//
//************kostas@kokoras.com*****************//

#include <Timer.h>
#include <SoftwareSerial.h>

#define RxD 7                //Arduino pin connected to Tx of HC-05
#define TxD 8                //Arduino pin connected to Rx of HC-05
//#define Reset 9              //Arduino pin connected to Reset of HC-05 (reset with LOW)
//#define PIO11 A2             //Arduino pin connected to PI011 of HC-05 (enter AT Mode with HIGH)
#define BT_CMD_RETRIES 5     //Number of retries for each Bluetooth AT command in case of not responde with OK
#define OBD_CMD_RETRIES 3    //Number of retries for each OBD command in case of not receive prompt '>' char
#define THROTTLE_CMD_RETRIES 5    //Number of retries for RPM command
//#define sel_sw 12            //Arduino input for storing curent Shift Light RPM



unsigned int throttle;//Variables for RPM
boolean but_pressed_flag;    //Variable if RPM Shift Light button is pressed
boolean bt_error_flag;       //Variable for bluetooth connection error
boolean obd_error_flag;      //Variable for OBD connection error
boolean throttle_error_flag;      //Variable for RPM error
boolean throttle_retries;         //Variable for RPM cmd retries
                 
SoftwareSerial blueToothSerial(RxD,TxD);
Timer t;
                  
//-----------------------------------------------------//
void setup()
{
   
   //pinMode(RxD, INPUT);
   //pinMode(TxD, OUTPUT);
   //pinMode(PIO11, OUTPUT);
   //pinMode(Reset, OUTPUT);
   
   //digitalWrite(PIO11, LOW);    //Set HC-05 to Com mode
   //digitalWrite(Reset, HIGH);   //HC-05 no Reset
   
   //pinMode(sel_sw,INPUT);
   blueToothSerial.begin(38400);
   Serial.begin(38400);

  Serial.write("1");
   throttle_retries=0;
   but_pressed_flag=false;
 Serial.write("2");
   t.every(250, throttle_calc);//Every 250ms read RPM value from OBD
   
   //start Bluetooth Connection
   setupBlueToothConnection();
 Serial.write("3");
   //in case of Bluetoth connection error
   if (bt_error_flag){
     Serial.write("error BT");
   } 
 Serial.write("4");
   //OBDII initialitation
   obd_init();
   
   //in case of OBDII connection error   
   if (obd_error_flag){
     Serial.write("error OBD");
   }  
}


//----------------------------------------------------------//
//---------------------Send OBD Command---------------------//
//--------------------for initialitation--------------------//

void send_OBD_cmd(char *obd_cmd){
  char recvChar;
  boolean prompt;
  int retries;
 
   if (!(obd_error_flag)){                                        //if no OBD connection error
    
    prompt=false;
    retries=0;
    while((!prompt) && (retries<OBD_CMD_RETRIES)){                //while no prompt and not reached OBD cmd retries
      blueToothSerial.print(obd_cmd);                             //send OBD cmd
      blueToothSerial.print("\r");                                //send cariage return

      while (blueToothSerial.available() <= 0);                   //wait while no data from ELM
      
      while ((blueToothSerial.available()>0) && (!prompt)){       //while there is data and not prompt
        recvChar = blueToothSerial.read();                        //read from elm
        if (recvChar==62) prompt=true;                            //if received char is '>' then prompt is true
      }
      retries=retries+1;                                          //increase retries
      delay(2000);
    }
    if (retries>=OBD_CMD_RETRIES) {                               // if OBD cmd retries reached
      obd_error_flag=true;                                        // obd error flag is true
    }
  }
}
 
//----------------------------------------------------------//
//----------------initialitation of OBDII-------------------//
void obd_init(){
  
  obd_error_flag=false;     // obd error flag is false
  
  send_OBD_cmd("ATZ");      //send to OBD ATZ, reset
  delay(1000);
  send_OBD_cmd("ATSP5");    //send ATSP5, protocol 5

  send_OBD_cmd("0100");     //send 0100, retrieve available pid's 00-19
  delay(1000);
  send_OBD_cmd("0120");     //send 0120, retrieve available pid's 20-39
  delay(1000);
  send_OBD_cmd("0140");     //send 0140, retrieve available pid's 40-??
  delay(1000);
  send_OBD_cmd("010C1");    //send 010C1, RPM cmd
  delay(1000);
  
}

//----------------------------------------------------------//
//-----------start of bluetooth connection------------------//
void setupBlueToothConnection()
{
  
  bt_error_flag=false;                    //set bluetooth error flag to false
  
//  enterATMode();                          //enter HC-05 AT mode
  delay(500);
 Serial.write("wtf");
  sendATCommand("RESET");                  //send to HC-05 RESET
  delay(1000);
  Serial.write("wtf2");
  sendATCommand("ROLE=1");                 //send ROLE=1, set role to master
  sendATCommand("CMODE=0");                //send CMODE=0, set connection mode to specific address
  sendATCommand("INIT");                   //send INIT, cant connect without this cmd 
  sendATCommand("BIND=8818,56,6898EB");    //send BIND=??, bind HC-05 to OBD bluetooth address
  delay(1000); 
  sendATCommand("PAIR=8818,56,6898EB,20"); //send PAIR, pair with OBD address
  delay(1000);  
  sendATCommand("LINK=8818,56,6898EB");    //send LINK, link with OBD address
  delay(1000); 
//  enterComMode();                          //enter HC-05 comunication mode
Serial.write("wtf3");
}

//----------------------------------------------------------//
//------------------reset of HC-05--------------------------//
//-------set reset pin of HC-05 to LOW for 2 secs-----------//
//void resetBT()
//{
// digitalWrite(Reset, LOW);
// delay(2000);
// digitalWrite(Reset, HIGH);
//}

//----------------------------------------------------------//
//--------Enter HC-05 bluetooth moduel command mode---------//
//-------------set HC-05 mode pin to LOW--------------------//
//void enterComMode()
//{
// blueToothSerial.flush();
// delay(500);
// digitalWrite(PIO11, LOW);
// resetBT();
// delay(500);
// blueToothSerial.begin(38400); //default communication baud rate of HC-05 is 38400
//}

//----------------------------------------------------------//
//----------Enter HC-05 bluetooth moduel AT mode------------//
//-------------set HC-05 mode pin to HIGH--------------------//
//void enterATMode()
//{
// blueToothSerial.flush();
// delay(500);
// digitalWrite(PIO11, HIGH);
// resetBT();
// delay(500);
// blueToothSerial.begin(38400);//HC-05 AT mode baud rate is 38400
//}

//----------------------------------------------------------//

void sendATCommand(char *command)
{
  
  char recvChar;
  char str[2];
  int i,retries;
  boolean OK_flag;
  
  if (!(bt_error_flag)){                                  //if no bluetooth connection error
    retries=0;
    OK_flag=false;
    
    while ((retries<BT_CMD_RETRIES) && (!(OK_flag))){     //while not OK and bluetooth cmd retries not reached
      
       blueToothSerial.print("AT");                       //sent AT cmd to HC-05
       if(strlen(command) > 1){
         blueToothSerial.print("+");
         blueToothSerial.print(command);
       }
       blueToothSerial.print("\r\n");
      
      while (blueToothSerial.available()<=0);              //wait while no data
      
      i=0;
      while (blueToothSerial.available()>0){               // while data is available
        recvChar = blueToothSerial.read();                 //read data from HC-05
          if (i<2){
            str[i]=recvChar;                               //put received char to str
            i=i+1;
          }
      }
      retries=retries+1;                                  //increase retries 
      if ((str[0]=='O') && (str[1]=='K')) OK_flag=true;   //if response is OK then OK-flag set to true
      delay(1000);
    }
  
    if (retries>=BT_CMD_RETRIES) {                        //if bluetooth retries reached
      bt_error_flag=true;                                 //set bluetooth error flag to true
    }
  }
  
}

//THROTTLE LOAD
void throttle_calc(){
   boolean prompt,valid;  
   char recvChar;
   char bufin[12];
   int i;
  
  if (!(obd_error_flag)){                                   //if no OBD connection error
     valid=false;
     prompt=false;
     blueToothSerial.print("0111");                        //send to OBD PID command 0104 is for LOAD
     blueToothSerial.print("\r");                           //send to OBD cariage return char
     while (blueToothSerial.available() <= 0);              //wait while no data from ELM
     i=0;
     while ((blueToothSerial.available()>0) && (!prompt)){  //if there is data from ELM and prompt is false
       recvChar = blueToothSerial.read();                   //read from ELM
       if ((i<12)&&(!(recvChar==32))) {                     //the normal respond to previus command is 0104/r41 04 ??>, so count 12 chars and ignore char 32 which is space
         bufin[i]=recvChar;                                 //put received char in bufin array
         i=i+1;                                             //increase i
       }  
       if (recvChar==62) prompt=true;                       //if received char is 62 which is '>' then prompt is true, which means that ELM response is finished 
     }

    if ((bufin[5]=='4') && (bufin[6]=='1') && (bufin[7]=='0') && (bufin[8]=='4')){ //if first four chars after our command is 410D
      valid=true;                                                                  //then we have a correct LOAD response
    } else {
      valid=false;                                                                 //else we dont
    }
    if (valid){                                                                    //in case of correct LOAD response
      throttle_retries=0;                                                               //reset to 0 retries
      throttle_error_flag=false;                                                        //set LOAD error flag to false
      
//Cálculo:
String loadHex(bufin[9]); 
String loadHex2(bufin[10]); 
String loadHexTotal=loadHex+loadHex2;
int DecimalDecode=hexToDec(loadHexTotal);
throttle=round((float(DecimalDecode)/255)*100); //Arredonda e devolve valor final
     }
    if (!valid){                                              //in case of incorrect LOAD response
      throttle_error_flag=true;                                    //set LOAD error flag to true
      throttle_retries+=1;                                         //add 1 retry
      throttle=0;                                                  //set LOAD to 0
      //Serial.println("RPM_ERROR");
      if (throttle_retries>=THROTTLE_CMD_RETRIES) obd_error_flag=true;  //if retries reached LOAD_CMD_RETRIES limit then set obd error flag to true
    }    
      if ((obd_error_flag==true)){throttle=0; throttle_retries=0;}                              //if no OBD connection error !GrcByte!   
}
}

// Converting from Hex to Decimal:
// NOTE: This function can handle a positive hex value from 0 - 65,535 (a four digit hex string).
//       For larger/longer values, change "unsigned int" to "long" in both places.
unsigned int hexToDec(String hexString) {
  
  unsigned int decValue = 0;
  int nextInt;
  
  for (int i = 0; i < hexString.length(); i++) {
    
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    
    decValue = (decValue * 16) + nextInt;
  }
  
  return decValue;
}

//----------------------------------------------------------//

void loop(){
  Serial.write("entrou"); 
  while (!(obd_error_flag)){            //while no OBD comunication error  
    if ((throttle>=0) && (throttle<100)){       //if rpm value is between 0 and 10000 

      //rpm_to_disp=int(rpm/100);         //devide by 100, cause we have only two 7-seg disps
      Serial.write(throttle);   
     
    
      if (throttle_error_flag){              //if rpm error yellow led ON
        Serial.write("error throttle");
      }
       
      //if (!(digitalRead(sel_sw))) but_pressed_flag=false;
    
    }
    else //if no correct rpm value received
    {
      Serial.write("incorrect throttle");
    }
    t.update();  //update of timer for calling rpm_calc
  }
  if (obd_error_flag) Serial.write("error OBD");;    //if OBD error flag is true
}
