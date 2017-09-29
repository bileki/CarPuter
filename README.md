# CarPuter
Development of an Automatic Speed Control for Vehicles

This project consists of developing a speed controller for a Mitsubishi Pagero TR4, based on an Arduino.
-	The Arduino Speed Control system gets the current speed, and the state of the throttle and break pedals from the car diagnostic port (OBD2).
-	The speed of the car is controlled by pulling the throttle cable with a servo motor.
-	The speed of the car and pedal state are obtained using a Bluetooth adapter module (ELM 327) plugged to the OBD2 car interface
-	The Arduino Speed Control system is connected to the ELM 327 Bluetooth module using a HC-05 Bluetooth module (BTH07 MODULE)
-	Warning: The HM-10 Bluetooh Module is a Bluetooth 4 and cannot connect to Bluetoth 2 devices such as the ELM 327 Bluetooth adapter!!!


***> How to connect Arduino or RaspberyPi to a HC-05 Bluetooh Module

-	We normally use the software serial  library (SoftwareSerial.h) to connect to the Bluetooth Module UART.

//Useful Commands:
-	SoftwareSerial mySerial(7, 8); // RX, TX
-	mySerial.begin(9600);
-	mySerial.flush();
-	mySerial.write("AT");  
-	while (mySerial.available())       Serial.write(mySerial.read());

WARNINGs: 

1)	There are many translation mistakes in the reference PDFs (HC-0305_serial_module_AT_commamd_set_201104_revised.PDF and Bluetooth4_en.pdf).
-	Always make sure to have the same version of the HC-05 software and the pdf manual!!

2)	To be able to send AT commands to the HC-05 Module, it has to be configured into "AT Mode" by keeping the KEY pin to 3,3V when power is connected to the Module, and the default baund rate has to be set to 38400
-	It does not need to be set back to "Com Mode"  (KEY pin LOW (0V)) to communicate to the ELM327. One can just leave it in AT Mode with baud rate 38400 forever!!!

3)	Note that the module will not give any reply if it does not understand a given command.

4)	If the module is turnned off in Master mode (Role=1), it will remain in Master mode when turnned on again.

5)	Warning: Differently from the HM-10 Bluetooth Module, the HC-05 Module commands have to end with both CR and NL ("\r\n").
-	The ELM327 Module commands also needs to end with both CR and NL ("\r\n").

6)	The proper way to manage the connection to the ELM327 Bluetooth Module:

 
-	Enter HC-05 AT mode		// the KEY pin has to be set to HIGH (3,3V) When power is connected to the Module and the default baund rate is 38400
-	AT+RESET				//send to Reset the HC-05 Module
-	delay(1000);
-	AT+ROLE=1               //send ROLE=1, set role to master || Role = 1 will stay even if power is turned off
-	AT+CMODE=0              //send CMODE=0, set connection mode to specific address || CMODE = 0 will stay even if power is turned off
-	AT+INIT                 //send INIT, cannot connect without this cmd. Initialize the SPP lib
-	delay(1000); 
-	AT+BIND=8818,56,6898EB  //send BIND=Mac"Adress", bind HC-05 to OBD bluetooth address ("8818,56,6898EB" is my ELM327 Bluetooth MAC address)
-	delay(3000); 
-	AT+PAIR=8818,56,6898EB,10 //send PAIR, pair with OBD address - Try it for 10 secconds
-	delay(11000);  
-	AT+LINK=8818,56,6898EB     //send LINK, link with OBD address
-	delay(3000); 

-	** Now the HC-05 is LINKED to the ELM327 **		// But ELM327 will still have difficulties to find the right protocol to comunicate to the OBD2
-	** The default protocol is set to AUTO... But it sometimes do not connect to the car... And olny works if engine is turned ON.. **
-	** To go arround this problem, we should force ELM327 to use the car specific Protocol **
-	** In my case it is the nr. 5: ISO 14230-4 (KWP FAST), so we used the command ATSP5 **

-	ATSP5					// Set ELM327 to use the Protocol nr. 5: ISO 14230-4 (KWP FAST) for a Mitsubishi Pagero TR4 (2010) 

-	** Then... Begin to send and receive commands to the OBD2 Interface **


7)	Exemple of a communication attempt via Arduino terminal that acctually worked (sent commands and HC-05/ELM327/OBD2 reply): 

-	AT+INIT		Reply: OK		// Note that the HC-05 Module remembered ROLE=1 and CMODE=0 commands.

-	AT+INQ		Reply: +INQ:8818:56:6898EB,1F00,7FFF		OK

-	AT+BIND=8818,56,6898EB		Reply: OK

-	AT+PAIR=8818,56,6898EB,20		Reply: OK

-	AT+LINK=8818,56,6898EB		Reply: OK

-	ATSP5		Reply: coacsATSP5		OK

-	0111		Reply: 0111		BUS INIT: OK	41 11 00
-	0111		Reply: 0111		41 11 00
-	010C		Reply: 010C		41 0C 00 00
-	010D		Reply: 010D		41 0D 00 
-	0111		Reply: 0111		41 11 00 
-	0111		Reply: 0111		41 11 27 
-	010C		Reply: 010C		41 0C 0C 10 
-	010C		Reply: 010C		41 0C 0C 18 
