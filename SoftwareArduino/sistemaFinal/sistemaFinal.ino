#include <SoftwareSerial.h>
#include <Servo.h>

#define RxD 7                       // Pino do Arduino conectado no Tx do HC-05
#define TxD 8                       // Pino do Arduino conectado no Rx do HC-05
#define servoPin 3                  // Pino do Arduino conectado no sinal do servo
#define Reset 10                    // Pino do Arduino conectado no Reset do HC-05 (reset com LOW)
#define BT_CMD_RETRIES 5            // Número de tentativas para cada comando AT do Bluetooth no caso de não responder OK
#define OBD_CMD_RETRIES 3           // Número de tentativas para cada comando OBD no caso de não responder com o carácter '>'
#define RPM_CMD_RETRIES 5           // Número de tentativas para o comando de obter RPM
#define VELOCITY_CMD_RETRIES 5      // Número de tentativas para o comando de obter RPM
#define THROTTLE_CMD_RETRIES 5      // Número de tentativas para o comando de obter o THROTTLE
#define LED 13                      // Define pino 13 para o LED
#define LASTERRORINI 100000         // Define um valor inicial para o erro
#define LIMTOP -10                  // Define o limite do freio na subida
#define LIMDOWN 5                   // Define o limite do freio na descida
#define LIMDELTA 100                // Define o limite do delta
#define SUMMAX 30                   // Define o limite da soma
#define INTERACTIONS 40              // Define o número de interações para cálculo do PID
#define TOL 1                       // Define a tolerância do valor da velocidade em relação ao target
#define FAIXA 5                     // Define a faixa para manter a velocidade constante
#define TOLKI 15                    // Define a tolerância da velocidade em relação ao target para o acúmulo da soma da componente integrativa do PID

int in = 1;                         // Variável de entrada da Serial
float lastProcess = 0;              // Variável para armazenar o valor do último tempo computado
float lastError = LASTERRORINI;     // Variável para armazenar o valor do último erro computado
float kp = 5, ki = 0.0, kd = 0.0, kb = 0.0; // Variáveis para armazenar os valores das contantes do PID(B)
float sum = 0;                      // Variável para acumular a soma da componente integrativa do PID
float servo = 1, pid = 0;           // Variáveis para armazenar os valores que serão passados ao servo
int pwm = 0;
int targetVelocity = 1, lastTargetVelocity = 1; // Variáveis para leitura do target da velocidade
int velocityBand = 1;               // Variáveis para leitura da velocidade
int countInteractions = 0, count = 0; // Contadores de interações 
int servoMax = 90, servoMin = 20;   // Variáveis para definir os limites de atuação do servo

// Flags de controle
boolean goingUp = true;             // Flag para definir se a velocidade está subindo ou descendo
boolean flagLight = false;          // Flag para acionar o LED
boolean flagBreak = true;           // Flag para acionar o uso do freio
boolean flagPID = false;            // Flag para acionar o cálculo do PID  
boolean flagActivatePID = false;    // Flag para ativar o uso do PID

boolean pair = false;             // Flag para sinalizar se é necessário ou não parear o Bluetooth
String check = "";                  // Variável para receber a entrada da Serial
char c;                             // Variável para receber cada byte da Serial
boolean bufferRead = false;         // Flag para sinalizar que a string foi completada

boolean bt_error_flag;              // Váriavel para o erro de comunicação com o Bluetooth
boolean obd_error_flag;             // Váriavel para o erro de comunicação com o ELM-327

boolean rpm_error_flag;             // Váriavel para erro do RPM
int rpm_retries = 0;                // Váriavel para o número de tentativas do comando de RPM
int rpm = 0;                        // Váriavel para o valor do RPM

boolean throttle_error_flag;        // Váriavel para erro do THROTTLE
int throttle_retries = 0;           // Váriavel para o número de tentativas do comando do THROTTLE
int throttle = 0;                   // Váriavel para o valor do THROTTLE

boolean velocity_error_flag;        // Váriavel para erro do THROTTLE
int velocity_retries = 0;           // Váriavel para o número de tentativas do comando do THROTTLE
int velocity = 0;                   // Váriavel para o valor do THROTTLE


SoftwareSerial blueToothSerial(RxD,TxD);     // Cria a entidade serial do Bluetooth
Servo myServo;                               // Cria entidade para controlar o servo


//----------------------------------------------------------//
//----------------Inicialização do OBDII--------------------//
//----------------------------------------------------------//
void obd_init() {
   // Define a flag de erro do OBD como FALSE
   obd_error_flag = false;
   // Envia comando para resetar o OBD
   send_OBD_cmd("ATZ");
   delay(1000);
   // Envia comando para definir o protocolo correto
   send_OBD_cmd("ATSP5");
   delay(1000);

}

//----------------------------------------------------------//
//-----------start of bluetooth connection------------------//
void setupBlueToothConnection() {
   // Define a flag de erro do OBD como FALSE
   bt_error_flag = false;
   // Reseta o HC-05
   sendATCommand("RESET");
   delay(1000);
   // Define função como MASTER
   sendATCommand("ROLE=1");
   // Define conexão para o modo de endereço específico
   sendATCommand("CMODE=0");
   // Inicia a conexão
   sendATCommand("INIT");
   delay(1000);
   // Vincula o HC-05 ao endereço do Bluetooth do ELM
   sendATCommand("BIND=8818,56,6898EB");
   delay(3000);
   // Pareia o HC-05 com o ELM
   sendATCommand("PAIR=8818,56,6898EB,10");
   delay(5000);
   // Linka o HC-05 ao endereço do ELM
   sendATCommand("LINK=8818,56,6898EB");
   delay(5000);
}

//----------------------------------------------------------//
//------------------Reseta o HC-05--------------------------//
//-------Seta o pino de reset do HC-05 para LOW por 2s------//
//----------------------------------------------------------//
void resetBT() {
   digitalWrite(Reset, LOW);
   delay(2000);
   digitalWrite(Reset, HIGH);
}

//----------------------------------------------------------//
//---------------------Envia comando OBD--------------------//
//----------------------------------------------------------//
void send_OBD_cmd(char *obd_cmd) {
   char recvChar;
   boolean prompt;
   int retries;
   Serial.println(obd_cmd);
   // Se não houver erro com a conexão do OBDII
   if (!(obd_error_flag)) {
      prompt = false;
      retries = 0;
      // Enquanto não estiver no prompt OBD e as tentativas forem menores do que o máximo
      while((!prompt) && (retries < OBD_CMD_RETRIES)) {
         // Envia o comando OBD
         blueToothSerial.print(obd_cmd);
         blueToothSerial.print("\r\n");
         // Espera até receber algum dado do ELM
         while (blueToothSerial.available() <= 0);
         // Enquanto há dados mas não está no prompt
         while ((blueToothSerial.available() > 0) && (!prompt)) {
            // Lê os dados do ELM
            recvChar = blueToothSerial.read();
            // Se o carácter for o '>', então prompt é TRUE
            if (recvChar == 62)
               prompt = true;
         }
         Serial.println(recvChar);
         // Incrementa as tentativas e espera 2s
         retries += 1;
         delay(2000);
      }
      // Se atingiu o máximo de tentativas, define como TRUE a flag de erro
      if (retries >= OBD_CMD_RETRIES)
         obd_error_flag = true;
   }
}

//----------------------------------------------------------//
//---------------Envia comando AT para o HC-05--------------//
//----------------------------------------------------------//
void sendATCommand(char *command) {
   char recvChar;
   char str[2];
   int i, retries;
   boolean OK_flag;
   Serial.println(command);
   // Se não houver erro com a conexão do Bluetooth
   if (!(bt_error_flag)) {
      retries = 0;
      OK_flag = false;
      // Enquanto não receber um OK e as tentativas forem menores do que o máximo
      while ((retries < BT_CMD_RETRIES) && (!(OK_flag))) {
         // Envia o comando AT para o HC-05
         blueToothSerial.print("AT");
         if (strlen(command) > 1) {
            // Concatena o '+' e o comando enviado como argumento
            blueToothSerial.print("+");
            blueToothSerial.print(command);
         }
         blueToothSerial.print("\r\n");
         // Espera até receber algum dado
         while (blueToothSerial.available() <= 0);
         i = 0;
         // Enquanto há dados
         while (blueToothSerial.available() > 0) {
            // Lê do HC-05
            recvChar = blueToothSerial.read();
            if (i < 2) {
               // Monta a string
               str[i] = recvChar;
               i += 1;
            }
         }
         // Incrementa as tentativas e espera 2s
         retries += 1;
         Serial.println(str);
         // Se a string for OK, seta a flag de OK para TRUE
         if ((str[0] == 'O') && (str[1] == 'K'))
            OK_flag = true;
         delay(1000);
      }
      // Se atingiu o máximo de tentativas, define como TRUE a flag de erro
      if (retries >= BT_CMD_RETRIES)
         bt_error_flag = true;
   }
}

//-----------------------------------------------------//
//---------------Calcula o valor do RPM----------------//
//-----------------------------------------------------//
void calcRPM(char *buf) {
   int i;
   // Zera as tentaivas e define a flag de erro como FALSE
   rpm_retries = 0;
   rpm_error_flag = false;

   // Para iniciar o cálculo do valor real do RPM
   // É necessário separar os dois bytes hexadecimais da resposta do OBD, por examplo A=0B e B=6C
   // A equação é ((A * 256) + B) / 4
   rpm = 0;
   // Entre os carácteres 10 e 14 do bufin está o valor do RPM
   for (i = 10; i < 14; i++) {
      // Se o carácter estiver entre 'A' e 'F'
      if ((buf[i] >= 'A') && (buf[i] <= 'F')) {
         // Subtraindo 55 temos o valor decimal do byte
         buf[i] -= 55;
      } 
      // Se o carácter estiver entre '0' e '9'
      if ((buf[i] >= '0') && (buf[i] <= '9')) {
         // Subtraindo 48 temos o valor decimal do byte
         buf[i] -= 48;
      }
      // Shifitando para a esquerda 4 bits e adicionando os 4 bits do novo carácter, temos rpm = (A * 256) + B
      rpm = (rpm << 4) | (buf[i] & 0xf);
   }
   // Shifitando para a direita 2 bits, temos rpm = rpm/4
   rpm = rpm >> 2;
}

//-----------------------------------------------------//
//-------------Calcula o valor do Throttle-------------//
//-----------------------------------------------------//
void calcThrottle(char *buf) {
   // Zera as tentaivas e define a flag de erro como FALSE
   throttle_retries = 0;
   throttle_error_flag = false;
   // Para iniciar o cálculo do valor real do THROTTLE
   // É necessário separar o byte hexadecimais da resposta do OBD, por exemplo A=0B
   // A equação é (A / 255) * 100
   // Transforma os bytes em String
   String loadHex(buf[9]);
   String loadHex2(buf[10]);
   // Converte-os pra decimal com a função hexToDec
   String loadHexTotal = loadHex + loadHex2;
   int DecimalDecode = hexToDec(loadHexTotal);
   // Faz o cálculo de throttle = (A / 255) * 100
   throttle = round((float(DecimalDecode) / 255) * 100);
}

//-----------------------------------------------------//
//-------------Calcula o valor do Velocity-------------//
//-----------------------------------------------------//
void calcVelocity(char *buf) {
   // Zera as tentaivas e define a flag de erro como FALSE
   velocity_retries = 0;
   velocity_error_flag = false;
   // Para iniciar o cálculo do valor real do VELOCITY
   // É necessário separar o byte hexadecimais da resposta do OBD, por exemplo A=0B
   // A equação é (A / 255) * 100
   // Transforma os bytes em String
   String loadHex(buf[9]);
   String loadHex2(buf[10]);
   // Converte-os pra decimal com a função hexToDec
   String loadHexTotal = loadHex + loadHex2;
   int DecimalDecode = hexToDec(loadHexTotal);
   // Faz o cálculo de velocity = (A / 255) * 100
   velocity = float(DecimalDecode);
}

//-----------------------------------------------------//
//----------Verifica qual a resposta do OBD------------//
//-----------------------------------------------------//
char checkResponse(char *buf) {
   // Retorna qual o comando está na serial
   if ((buf[6] == '4') && (buf[7] == '1') && (buf[8] == '0') && (buf[9] == 'C'))
      return 'r';
   else if ((buf[5] == '4') && (buf[6] == '1') && (buf[7] == '1') && (buf[8] == '1'))
      return 't';
   else if ((buf[6] == '4') && (buf[7] == '1') && (buf[8] == '0') && (buf[9] == 'D'))
      return 'v';
   else
      return 'e';
}

//-----------------------------------------------------//
//----------Recebe o valor do RPM do OBD---------------//
//-----------------------------------------------------//
void getRPM() {
   boolean prompt;  
   char recvChar, resp;
   char bufin[15];
   int i;
   // Se não houver erro com a conexão do OBDII
   if (!obd_error_flag) {
      prompt = false;
      // Envia o comando 010C para obter o RPM, o 1 final é para indicar que o ELM deve esperar apenas uma resposta
      blueToothSerial.print("010C1");
      blueToothSerial.print("\r");
      // Espera até receber algum dado do ELM
      while (blueToothSerial.available() <= 0);
      i = 0;
      // Enquanto não estiver no prompt OBD e as tentativas forem menores do que o máximo
      while ((blueToothSerial.available() > 0) && (!prompt)) {
         // Lê os dados do ELM
         recvChar = blueToothSerial.read();
         // A resposta normal para o comando é 010C1/r41 0C ?? ??>, então conta 15 carácteres e ignora o carácter 32 que é o espaço
         if ((i < 15) && (!(recvChar == 32))) {
            // Monta a string
            bufin[i] = recvChar;
            i += 1;
         }
         // Se o carácter 62 recebido for o '>', então está no prompt e a resposta do ELM está pronta  
         if (recvChar == 62)
            prompt = true;
      }
      resp = checkResponse(bufin);
      // Verifica se o comando retornou um valor de RPM válido
      if (resp == 'r')
         calcRPM(bufin);
      if (resp == 'v')
         calcVelocity(bufin);
      if (resp == 't')
         calcThrottle(bufin);
      if (resp == 'e') {
         // Seta a flag de erro como TRUE
         rpm_error_flag = true;
         // Incrementa uma tentativa
         rpm_retries += 1;
         // E zera o valor do RPM
         rpm = 0;
         // Se atingiu o máximo de tentativas, define como TRUE a flag de erro do OBD
         if (rpm_retries >= RPM_CMD_RETRIES) 
            obd_error_flag = true;
      }
      // No caso de haver erro na comunicação OBD, zera o valor do rpm e das tentativas
      if (obd_error_flag) {
         rpm = 0;
         rpm_retries = 0;
      }
   }
}

//-----------------------------------------------------//
//--------Recebe o valor do THROTTLE do OBD------------//
//-----------------------------------------------------//
void getThrottle() {
   boolean prompt;  
   char recvChar, resp;
   char bufin[15];
   int i;
   // Se não houver erro com a conexão do OBDII
   if (!obd_error_flag) {
      prompt = false;
      // Envia o comando 010C para obter o THROTTLE
      blueToothSerial.print("0111");
      blueToothSerial.print("\r");
      // Espera até receber algum dado do ELM
      while (blueToothSerial.available() <= 0);
      i = 0;
      // Enquanto não estiver no prompt OBD e as tentativas forem menores do que o máximo
      while ((blueToothSerial.available() > 0) && (!prompt)) {
         // Lê os dados do ELM
         recvChar = blueToothSerial.read();
         // A resposta normal para o comando é 0111/r41 11 ??>, então conta 12 carácteres e ignora o carácter 32 que é o espaço
         if ((i < 12) && (!(recvChar == 32))) {
            // Monta a string
            bufin[i] = recvChar;
            i += 1;
         }
         // Se o carácter 62 recebido for o '>', então está no prompt e a resposta do ELM está pronta  
         if (recvChar == 62)
            prompt = true;
      }

      resp = checkResponse(bufin);
      // Verifica se o comando retornou um valor de RPM válido
      if (resp == 't')
         calcThrottle(bufin);
      if (resp == 'v')
         calcVelocity(bufin);
      if (resp == 'r')      
         calcRPM(bufin);
      if (resp == 'e') {
         // Seta a flag de erro como TRUE
         throttle_error_flag = true;
         // Incrementa uma tentativa
         throttle_retries += 1;
         // E zera o valor do THROTTLE
         throttle = 0;
         // Se atingiu o máximo de tentativas, define como TRUE a flag de erro do OBD
         if (throttle_retries >= THROTTLE_CMD_RETRIES)
            obd_error_flag = true;
      }
      // No caso de haver erro na comunicação OBD, zera o valor do rpm e das tentativas
      if (obd_error_flag) {
         throttle = 0;
         throttle_retries = 0;
      }
   }
}

void getVelocity() {
   boolean prompt;  
   char recvChar, resp;
   char bufin[15];
   int i;
   // Se não houver erro com a conexão do OBDII
   if (!obd_error_flag) {
      prompt = false;
      // Envia o comando 010C para obter o VELOCITY
      blueToothSerial.print("010D1");
      blueToothSerial.print("\r");
      // Espera até receber algum dado do ELM
      while (blueToothSerial.available() <= 0);
      i = 0;
      // Enquanto não estiver no prompt OBD e as tentativas forem menores do que o máximo
      while ((blueToothSerial.available() > 0) && (!prompt)) {
         // Lê os dados do ELM
         recvChar = blueToothSerial.read();
         // A resposta normal para o comando é 0111/r41 11 ??>, então conta 12 carácteres e ignora o carácter 32 que é o espaço
         if ((i < 12) && (!(recvChar == 32))) {
            // Monta a string
            bufin[i] = recvChar;
            i += 1;
         }
         // Se o carácter 62 recebido for o '>', então está no prompt e a resposta do ELM está pronta  
         if (recvChar == 62)
            prompt = true;
      }

      resp = checkResponse(bufin);
      // Verifica se o comando retornou um valor de RPM válido
      if (resp == 'v')
         calcVelocity(bufin);
      if (resp == 't')
         calcVelocity(bufin);
      if (resp == 'r')      
         calcRPM(bufin);
      if (resp == 'e') {
         // Seta a flag de erro como TRUE
         velocity_error_flag = true;
         // Incrementa uma tentativa
         velocity_retries += 1;
         // E zera o valor do VELOCITY
         velocity = 0;
         // Se atingiu o máximo de tentativas, define como TRUE a flag de erro do OBD
         if (velocity_retries >= VELOCITY_CMD_RETRIES)
            obd_error_flag = true;
      }
      // No caso de haver erro na comunicação OBD, zera o valor do rpm e das tentativas
      if (obd_error_flag) {
         velocity = 0;
         velocity_retries = 0;
      }
   }
}

//-----------------------------------------------------//
//----Converte um valor hexadecimal para decimal-------//
//-----------------------------------------------------//
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


//-----------------------------------------------------//
//----------Define quando o PID será usado-------------//
//----------------de modo pervasivo--------------------//
//-----------------------------------------------------//
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
         // Espera 3 interações para ativar o uso do PID
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

//-----------------------------------------------------//
//-----Ajusta o valor da velocidade para um target-----//
//-----------------------------------------------------//
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
   if ((dt > 0) && (lastError != LASTERRORINI)) {
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
   if ((flagBreak) && (error != 0)) {
      // Define o sinal oposto a direção da curva para freá-la
      if (goingUp) 
         breaker = -1 * abs(delta / (error * error));
      else 
         breaker = abs(delta / (error * error));
   } else {
      breaker = 0;
   }

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



//-----------------------------------------------------//
void setup() {
	// Inicializa a serial com a mesma velocidade de comunicação do Bluetooth
   Serial.begin(38400);
   // Define a taxa de comunicação padrão do HC-05
   blueToothSerial.begin(38400);
   // Limpa qualquer dado que estiver nas serial
   Serial.flush();   
   // Limpa os dados que estiverem na serial do Bluetooth
   blueToothSerial.flush();
   myServo.attach(servoPin);
   myServo.write(1);

   while (1) {
      // Lê da Serial
      if (Serial.available()) {  
         delay(100);  
         while (Serial.available() > 0) {
            c = Serial.read();
            check += c;
            bufferRead = true;
         }

         if(bufferRead) {
            if (check == "sair") {
               pair = false;
               break;
            }
            if (check == "parear") {
               pair = true;
               break;
            }
         }
      }
   }
   if (pair) {

      // Inicia a conexão com o Bluetooth
      setupBlueToothConnection();
      // No caso de haver erro na conexão com o Bluetooth
      if (bt_error_flag) {
         Serial.print("Erro de conexão com o Bluetooth");
      }
      // Inicializa a conexão OBDII
      obd_init();
      // No caso de haver erro na conexão com o OBDII
      if (obd_error_flag) {
         Serial.print("Erro de conexão com o OBDII");
      }
      
   }
   
  
}

//----------------------------------------------------------//
void loop(){

   // Lê a entrada da Serial
   if (Serial.available() > 0) {
      delay(10);
      in = (Serial.read() - '0') * 100 + (Serial.read() - '0') * 10 + (Serial.read() - '0');

      if (in < 0)
         in = 0;

      if (in > 100) 
         in = 100;      
   }

   // Obtêm as informações do veículo
   getThrottle();
   delay(300);
   getRPM();
   delay(300);


   // Mapeia os valores de RPM de 0-6000 para a porcentagem de 0-100 
   velocity = map(rpm, 0, 6000, 0, 100);
   velocity++;

   // Chama função que verifica quando o sistema deve entrar em ação
   IA();
   // Chama o cáculo do PID
   pid = PID(); 
     
   // Verifica se o PID será usado
   if (flagLight) {
   // Se o PID estiver habilitado, soma o valor do ângulo atual do servo com a correção do PID    
      if (flagActivatePID)  
        servo += pid;      

      // Heurística de limitação da atuação do servo
      // Se a velocidade estiver a mais de 7 km/h do target, libera o servo para ir de 20 a 90
      if (abs(targetVelocity - velocity) > 7) {
         servoMin = 20; servoMax = 90;
      // Se a velocidade estiver entre 5 e 7 km/h do target, libera o servo para ir de 40 a 70   
      } else if (abs(targetVelocity - velocity) > 3) {
         servoMin = 40; servoMax = 70;
      // Se a velocidade estiver entre 3 e 5 km/h do target, libera o servo para ir de 45 a 60   
      } else if (abs(targetVelocity - velocity) > 2) {
         servoMin = 45; servoMax = 60;
      }

      // Define os limites de rotação do servo
      if (servo > servoMax)
         servo = servoMax;
      if (servo < servoMin)
         servo = servoMin;

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
      pwm = 20;
   }

   // Caso seja digitado 000 na Serial, solta o controle do servo, "panic button", do contrário, envia o PWM para o servo
   if (in == 0)
      myServo.write(1);
   else
      myServo.write(pwm);

   // Debugging...  
   Serial.print("t: ");
   Serial.print(throttle);
   Serial.print("\t tv: ");
   Serial.print(targetVelocity);
   Serial.print("\t rpm: ");
   Serial.print(rpm);      
   Serial.print("\t v: ");
   Serial.print(velocity);      
   Serial.print("\t vb: ");
   Serial.print(velocityBand);      
   Serial.print("\t pwm: ");
   Serial.print(pwm);
   Serial.print("\t sum: ");
   Serial.print(sum);
   Serial.println();
   
}
