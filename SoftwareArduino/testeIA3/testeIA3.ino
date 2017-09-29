  #include <SoftwareSerial.h>
  #include <PWM.h>
  
  // Define pino 13 para o LED
  #define LED 13
  // Define um valor inicial para o erro
  #define LASTERRORINI 100000
  // Define o limite do freio na subida
  #define LIMTOP -10
  // Define o limite do freio na descida
  #define LIMDOWN 5
  // Define o limite do delta
  #define LIMDELTA 100
  // Define o limite da soma
  #define SUMMAX 10
  // Define o número de interações para cálculo do PID
  #define INTERACTIONS 10
  // Define o número de interações para resposta do motorista
  #define RESPONSETIME 5
  // Define a tolerância do valor da velocidade em relação ao target
  #define TOL 1
  // Define a faixa para manter a velocidade constante
  #define FAIXA 5
  // Define a tolerância da velocidade em relação ao target para o acúmulo da soma da componente integrativa do PID
  #define TOLKI 15
  
  
  SoftwareSerial mySerial(7, 8); // RX, TX
  Servo myServo; 
  
  String check = "";
  char c;
  int flag = 0, sair = 0;
  
  
  unsigned int throttle;
  
  
  // Variável para armazenar o valor do último tempo computado
  float lastProcess = 0;
  // Variável para armazenar o valor do último erro computado
  float lastError = LASTERRORINI;
  // Variáveis para armazenar os valores das contantes do PID(B)
  float kp = 0.06, ki = 0.003, kd = 0.0, kb = 0.3;
  // Variável para acumular a soma da componente integrativa do PID
  float sum = 0;
  // Variáveis para armazenar os valores que serão passados ao servo
  float servo = 35, pid = 0;
  int pwm = 0;
  // Variáveis para leitura do target da velocidade
  int targetVelocity = 100, lastTargetVelocity = 100;
  // Variáveis para leitura da velocidade
  int velocityBand = 100, velocity = 0;
  // Contadores de interações 
  int countInteractions = 0, count = 0;
  
  // Flags de controle
  boolean goingUp = true; // Flag para definir se a velocidade está subindo ou descendo
  boolean flagLight = false; // Flag para acionar o LED
  boolean flagBreak = true; // Flag para acionar o uso do freio
  boolean flagPID = false; // Flag para acionar o cálculo do PID  
  boolean flagActivatePID = false; // Flag para ativar o uso do PID
  int32_t frequency = 50;
  bool success = false;

  
  void setup() {
      InitTimersSafe();  
      success = SetPinFrequencySafe(9, frequency);
      Serial.begin(38400);    
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
                  check = "";
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
                      mySerial.write("ATSP5\r\n");
                      check = "";
                      mySerial.flush();
                      break;
                  }
                  
                  if (check == "auto") {
                      mySerial.flush();
                      check = "AT+RESET\r\n";
                      mySerial.print(check);
                      check = "";
                      delay(1000);
                      resposta();
                      
                      check = "AT+ROLE=1\r\n";
                      mySerial.print(check);
                      check = "";
                      delay(1000);
                      resposta();
                      
                      check = "AT+CMODE=0\r\n";
                      mySerial.print(check);
                      check = "";
                      delay(1000);
                      resposta();
                      
                      check = "AT+INIT\r\n";
                      mySerial.print(check);
                      check = "";
                      delay(1000);
                      resposta();
                      
                      check = "AT+BIND=8818,56,6898EB\r\n";
                      mySerial.print(check);
                      check = "";
                      delay(3000);
                      resposta();
                      
                      check = "AT+PAIR=8818,56,6898EB,10\r\n";
                      mySerial.print(check);
                      check = "";
                      delay(11000);
                      resposta();
                       
                      check = "AT+LINK=8818,56,6898EB\r\n";
                      mySerial.print(check);
                      check = "";
                      delay(11000);
                      resposta();
                      
                      mySerial.write("ATSP5\r\n");                  
                      delay(1000);
                      resposta();
                      
                      mySerial.write("0111\r\n");
                      resposta();
                      
                      check = "";
                      mySerial.flush();
                      break;
                      
                  }
                  
                  Serial.print(check);
                  Serial.write("\n");
                  mySerial.flush();
                  check += "\r\n";
                  mySerial.print(check);
                  check = "";
                  flag = 0;
              }
          }
      }

   // Inicializa a váriavel de tempo
   lastProcess = millis();

  }
	  
	void loop() {  

	  
	  getThrottle();  
	      
	  //throttle = analogRead(A0);
	  velocity = analogRead(A0);
	  // Mapeia os valores dos potenciômetros de 0-1023 para os ângulos de 0-179 
	  velocity = map(velocity, 240, 1000, 35, 179);

	  // Chama função que verifica quando o sistema deve entrar em ação
	  IA();
	  // Chama o cáculo do PID
	  pid = PID(); 
	  // Verifica se o PID será usado
	  if (flagLight) {
	          
	    // Soma o valor do ângulo atual do servo com a correção do PID    
	    if (flagActivatePID)  
	      servo += pid;      

	    // Define os limites de rotação do servo
	    if (servo > 167)
	      servo = 167;
	    if (servo < 35)
	      servo = 35;

	    // Zera a soma dos erros integrativos do PID caso a leitura do potenciômetro esteja fora da tolerância
	    if ((velocity > (targetVelocity + TOLKI)) || (velocity < (targetVelocity - TOLKI))) 
	      sum = 0;

	    // Define um intervalo máximo para a soma dos erros integrativos
	    if (sum > SUMMAX)
	      sum = SUMMAX;
	    if (sum < -SUMMAX)
	      sum = -SUMMAX;
	      
	    // Ajusta o valor do servo para inteiro
	    pwm = (int)servo;

	  } else {
	    // Define que o servo deve seguir o controle manual
	    pwm = throttle;
	  }

	  pwmWrite(9, pwm); 
	  
	// Debugging...
	   Serial.print(throttle);
	   Serial.print("\t");
	   Serial.print(targetVelocity);
	   Serial.print("\t");
	   Serial.print(velocity);      
	   Serial.print("\t");
	   Serial.print(velocityBand);      
	   Serial.print("\t");
	   Serial.print(pwm);
	   Serial.print("\t");
	   Serial.print(sum);
	   Serial.print("\t");
	   Serial.print(countInteractions);
	   Serial.print("\t");
	   Serial.print(flagLight);
	   Serial.print("\t");
	   Serial.println();
      
      
      delay(1000);
  }
  
  
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
       else throttle=-1;
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
          check = "";
          mySerial.flush(); 
          mySerial.write("0111\r\n");
          flag = 0;
      }
  }


void resposta() {
    while (mySerial.available() > 0)  
        {
            c = mySerial.read();
            check += c;
        }
    Serial.print(check);
    Serial.write("\n"); 
    check = "";    
 }



float PID() {
  // Variáveis para uso no cáculo do PID
  float P, I, D, B, myPID, dt, error;
  float breaker = 0, delta = 0;

  // Cáculo do erro associado as medições
  error = targetVelocity - velocity;

  // Cálculo do delta de tempo
  dt = (millis() - lastProcess) / 1000.0;

  // Cálculo da componente proporcional
  P = kp * error;

  // Cálculo da componente integrativa
  sum += (error * dt);
  I = ki * sum;

  // Cálculo da componente derivativa
  // Para evitar divisão por 0 confere-se o dt > 0 e se o lastError é o definido pela primeira vez
  if ((dt > 0) && (lastError != 100000)) {
    delta = ((error - lastError) / dt);
    D = kd * delta;
  }
  else 
    D = delta = 0;

  // Define que o freio não deve ser usado no caso de os sinais da última leitura e da atual forem diferentes(quando a curva muda de sinal), ou se a atual for 0(chegou no setpoint)
  if (((error > 0) && (lastError < 0)) || ((error < 0) && (lastError > 0)) || (error == 0) || (abs(delta) < LIMDELTA))
    flagBreak = false;

  // Define o uso do freio caso o setpoint mudar
  if ((targetVelocity != lastTargetVelocity))
    flagBreak = true;

  if ((targetVelocity - lastTargetVelocity) > 0)
    goingUp = true;
  if ((targetVelocity - lastTargetVelocity) < 0)
    goingUp = false;

  // Atualiza o último setpoint
  lastTargetVelocity = targetVelocity;

  // Caso o freio esteja definido pra ser usado e o setpoint não tenha sido alcançado
  if ((flagBreak) && (error != 0))
    // Define o sinal oposto a direção da curva para freá-la
    if (goingUp) breaker = -1 * abs(delta / (error * error));
    else breaker = abs(delta / (error * error));
  else
    breaker = 0;

  // Cálculo da componente do freio
  B = kb * breaker;

  // Define limites de atuação do freio
  if(B > LIMDOWN)
    B = LIMDOWN;      
  if(B < LIMTOP)
    B = LIMTOP;       
  
  // Atualização dos valores do último erro e tempo
  lastError = error;
  lastProcess  = millis();

  // Resultado do PID
  myPID = P + I + D + B;
  
  return myPID;
}



void IA() {
  // Caso o cálculo do PID não esteja habillitado
  if (!flagPID) {
    // Verifica se a velocidade está dentro de uma faixa constante
    if ((velocity < (velocityBand + FAIXA)) && (velocity > (velocityBand - FAIXA))) {
      // Incrementa as interações
      countInteractions++;
    } else {
      // Caso a velocidade saia da faixa constante, redefine-se a faixa e zera as interações
      countInteractions = 0;
      velocityBand = velocity;
    }
    // Caso o motorista mantenha a faixa constante de velocidade por 300 interações
    if (countInteractions > INTERACTIONS) {
      // Liga-se o LED
      digitalWrite(LED, HIGH);
      flagLight = true;
      // Redefine-se o target da velocidade
      targetVelocity = velocity;
      // Define o valor do servo com o último valor enviad ao servo
      servo = pwm;        
    }
    // Caso o LED esteja aceso
    if (flagLight) {
      // Espera 5 interações para ativar o uso do PID
      if (count > 3) { 
        flagActivatePID = true;
        flagPID = true;
      } else {
        // Caso contrário, zera a soma do fator integrativo, pois o erro acumulado pode ser muito grande e atrapalhar a primeira correção do PID
        sum = 0;
      }
      // Incrementa a contagem 
      count++;
    }
  } else {
    // Caso o cáculo do PID esteja habilitado e o motorista ultrapassar o target da velocidade
    if (throttle > targetVelocity) {
      // Desativa o cálculo e o uso do PID, apaga o LED, zera os contadores e redefine a faixa de velocidae
      flagPID = false;
      flagActivatePID = false;
      digitalWrite(LED, LOW);
      flagLight = false;
      countInteractions = 0;
      count = 0;
      velocityBand = velocity;
    }
  }
}
