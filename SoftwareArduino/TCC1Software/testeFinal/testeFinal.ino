  #include <SoftwareSerial.h>
  #include <Timer.h>
  
  #define R 2
  #define PWM 3
  #define L 4 
  #define MIN 40
  #define SETTOL 10

  float ontem = millis();
  
  int i, j, posicaoCabo = 0;
  int tolerancia = 0, toleranciaki = 20;
  int targetAceleracao = 500, targetVelocidade = 50;
  float soma = 0, FposicaoCabo = 0, lastError = 0;  
  float kc = 0.2, ki = 0.05, kd = 0;
  
  Timer t;
  SoftwareSerial mySerial(7, 8); // RX, TX
  
  String check = "";
  char c;
  int flag = 0, sair = 0;
  unsigned int throttle;
  
  void setup() {
      Serial.begin(38400);
      pinMode(R, OUTPUT);
      pinMode(L, OUTPUT);
      pinMode(PWM, OUTPUT); 
      
      t.every(300, getThrottle);      
    
      mySerial.begin(38400);
      delay(800);
      mySerial.flush(); 
      Serial.flush(); 
  
      while (1) {
          if (mySerial.available())
          {  
              delay(100);  
              //transfere o buffer do mySerial para a string check
              while (mySerial.available() > 0)  
              {
                  c = mySerial.read();
                  check += c;
                  flag = 1;
              }
              if(flag==1) {
                  Serial.print(check);
                  Serial.write("\n");
                  throttle_calc();
                  check.remove(0);
                  flag = 0;
              }
          }
      
          if (Serial.available()) {  
              delay(100);  
              while (Serial.available() > 0)  //transfere o buffer do mySerial para a string check
              {
                  c = Serial.read();
                  check += c;
                  flag = 1;
              }
          
              if(flag==1) {
                  if (check == "sair") {
                      mySerial.write("0111\r\n");
                      check.remove(0);
                      mySerial.flush();
                      break;
                  }
                  
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
  
  void loop() {  
  //media dos valores obtidos pra diminuir o ruido
      FposicaoCabo = analogRead(A0);
      for (i = 0; i < 50; i++) {    
          for (j = 0; j < 10; j++) {
              FposicaoCabo += analogRead(A0);
          }
          FposicaoCabo = FposicaoCabo/11;
      }
      posicaoCabo = (int)(FposicaoCabo);
      
      if (Serial.available() > 0) {
          delay(10);
          targetVelocidade = (Serial.read() - '0') * 100 + (Serial.read() - '0') * 10 + (Serial.read() - '0');
          
          if (targetVelocidade < 20) 
              targetVelocidade = 20;
              
          if (targetVelocidade > 80) 
              targetVelocidade = 80;      
      }
  
      int vel = (int)PID();
      
      t.update();
      
      if ((posicaoCabo > (targetAceleracao+toleranciaki)) || (posicaoCabo < (targetAceleracao-toleranciaki))) {
          soma = 0;
      }
  
      if (soma > 10) 
          soma = 10;
      if (soma < -10) 
          soma = -10;
      
      if (posicaoCabo < targetAceleracao-tolerancia) {
          //somando o valor do vel com o MIN para ele rodar, caso seja menor q MIN
          vel=vel+MIN;
          Serial.print(" R ");
          Serial.print(vel);
          
          digitalWrite(L, HIGH);
          digitalWrite(R, LOW);
          analogWrite(PWM, vel);
      
      } else if (posicaoCabo > targetAceleracao+tolerancia) {
          //como o vel fica negativo, subtraimos o MIN pra aumentar o valor negativo
          vel = vel-MIN;
          Serial.print(" L ");
          Serial.print(vel);
          
          digitalWrite(R, HIGH);
          digitalWrite(L, LOW);
          //multiplicamos por -1 pois o pwm precisa ser positivo
          analogWrite(PWM, -1*vel);
          //tolerancia para manter estabilizado em uma certa faixa
          
      } else {    
          digitalWrite(PWM, 0);
      }
  
      if (posicaoCabo == targetAceleracao)
          tolerancia = SETTOL; 
      
      if ((posicaoCabo > targetAceleracao +tolerancia) || (posicaoCabo < targetAceleracao - tolerancia))
          tolerancia = 0;
      
      Serial.println();
  }
  
  float PID() {
      float error, integral, proportional, derivative, dt;
      
      dt = (millis() - ontem)/1000;
      ontem  = millis();
      
      error = targetAceleracao-posicaoCabo;
      soma = soma + (error * dt);
      
      proportional = error * kc;  
      integral = ki * soma;
      derivative = kd * ((error - lastError)/dt);
      
      lastError = error;
      
      return proportional + integral + derivative;
  }
  
  float ajuste() {
      if(throttle < targetVelocidade-1) {
          if (targetAceleracao > 20) 
              targetAceleracao = targetAceleracao - 10;
      }
      else if(throttle > targetVelocidade+1) { 
          if (targetAceleracao < 980) 
              targetAceleracao = targetAceleracao + 10;
      }   
  }
  
  //THROTTLE LOAD
  void throttle_calc(){
      boolean valid;  
     
      valid=false;
      
      if ((check[5]=='4') && (check[6]=='1') && (check[8]=='1') && (check[9]=='1')){ 
          valid=true;
      } else {
          valid=false;
      }
      
      if (valid){
          String loadHex(check[11]); 
          String loadHex2(check[12]); 
          
          String loadHexTotal=loadHex+loadHex2;
          int DecimalDecode=hexToDec(loadHexTotal);
          throttle=round((float(DecimalDecode)/255)*100); //Arredonda e devolve valor final
      }   
  
  }
  
  unsigned int hexToDec(String hexString) {  
      unsigned int decValue = 0;
      int nextInt;
      
      for (int i = 0; i < hexString.length(); i++) {      
          nextInt = int(hexString.charAt(i));
          if (nextInt >= 48 && nextInt <= 57) 
              nextInt = map(nextInt, 48, 57, 0, 9);
          if (nextInt >= 65 && nextInt <= 70) 
              nextInt = map(nextInt, 65, 70, 10, 15);
          if (nextInt >= 97 && nextInt <= 102) 
              nextInt = map(nextInt, 97, 102, 10, 15);
              
          nextInt = constrain(nextInt, 0, 15);          
          decValue = (decValue * 16) + nextInt;
      }
      
      return decValue;
  }
  
  
  void getThrottle() {  
      while (mySerial.available() > 0)
      {
          c = mySerial.read();
          check += c;
          flag = 1;
      }
      
      if(flag==1) {
          throttle_calc();
          ajuste();
          check.remove(0);
          mySerial.flush(); 
          mySerial.write("0111\r\n");
          flag = 0;
      }
  }

