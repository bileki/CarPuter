#include <SoftwareSerial.h>
#include <PWM.h>

#define RxD 7                    // Pino do Arduino conectado no Tx do HC-05
#define TxD 8                    // Pino do Arduino conectado no Rx do HC-05
#define servoPin 3               // Pino do Arduino conectado no sinal do servo
#define Reset 10                 // Pino do Arduino conectado no Reset do HC-05 (reset com LOW)
#define PIO11 A2                 // Pino do Arduino conectado no PI011 do HC-05 (entrar no AT Mode com HIGH)
#define BT_CMD_RETRIES 5         // Número de tentativas para cada comando AT do Bluetooth no caso de não responder OK
#define OBD_CMD_RETRIES 3        // Número de tentativas para cada comando OBD no caso de não responder com o carácter '>'
#define RPM_CMD_RETRIES 5        // Número de tentativas para o comando de obter RPM
#define THROTTLE_CMD_RETRIES 5   // Número de tentativas para o comando de obter o THROTTLE

boolean parear = false;
String check = "";
char c;
int flag = 0;
int32_t frequency = 150;
bool success = false;

boolean bt_error_flag;           // Váriavel para o erro de comunicação com o Bluetooth
boolean obd_error_flag;          // Váriavel para o erro de comunicação com o ELM-327

int rpm = 0;                     // Váriavel para o valor do RPM
boolean rpm_error_flag;          // Váriavel para erro do RPM
boolean rpm_retries = 0;         // Váriavel para o número de tentativas do comando de RPM

boolean throttle_error_flag;     // Váriavel para erro do THROTTLE
boolean throttle_retries = 0;    // Váriavel para o número de tentativas do comando do THROTTLE
int throttle = 0;                // Váriavel para o valor do THROTTLE

int taxaDelay = 5;               // Váriavel com o valor de ajuste de tempo de espera

SoftwareSerial blueToothSerial(RxD,TxD);     // Cria a entidade serial do Bluetooth



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
            if (recvChar == 62) prompt = true;
         }
         Serial.println(recvChar);
         // Incrementa as tentativas e espera 2s
         retries += 1;
         delay(2000);
      }
      // Se atingiu o máximo de tentativas, define como TRUE a flag de erro
      if (retries >= OBD_CMD_RETRIES) obd_error_flag = true;
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
         if ((str[0] == 'O') && (str[1] == 'K')) OK_flag = true;
         delay(1000);
      }
      // Se atingiu o máximo de tentativas, define como TRUE a flag de erro
      if (retries >= BT_CMD_RETRIES) bt_error_flag = true;
   }
}

//-----------------------------------------------------//
//----------Recebe o valor do RPM do OBD---------------//
//---------Converte para um valor decimal--------------//
//-----------------------------------------------------//
void rpm_calc() {
   boolean prompt, valid;  
   char recvChar;
   char bufin[15];
   int i;
   // Se não houver erro com a conexão do OBDII
   if (!(obd_error_flag)) {
      valid = false;
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
         if (recvChar == 62) prompt = true;
      }
      // Se os 4 primeiros caractéres após o comando forem 410C
      if ((bufin[6] == '4') && (bufin[7] == '1') && (bufin[8] == '0') && (bufin[9] == 'C')) {
         // Então a resposta do RPM é válida
         valid = true;
      } else {
         valid = false;
      }
      // No caso da resposta válida
      Serial.println("bufin rpm: ");
      Serial.println(bufin);
      Serial.println("valid rpm: ");
      Serial.println(valid);
      
      if (valid) {
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
            if ((bufin[i] >= 'A') && (bufin[i] <= 'F')) {
               // Subtraindo 55 temos o valor decimal do byte
               bufin[i] -= 55;
            } 
            // Se o carácter estiver entre '0' e '9'
            if ((bufin[i] >= '0') && (bufin[i] <= '9')) {
               // Subtraindo 48 temos o valor decimal do byte
               bufin[i] -= 48;
            }
            // Shifitando para a esquerda 4 bits e adicionando os 4 bits do novo carácter, temos rpm = (A * 256) + B
            rpm = (rpm << 4) | (bufin[i] & 0xf);
         }
         // Shifitando para a direita 2 bits, temos rpm = rpm/4
         rpm = rpm >> 2;
      }
   }
   // No caso de um valor inválido do RPM
   if (!valid) {
      // Seta a flag de erro como TRUE
      rpm_error_flag = true;
      // Incrementa uma tentativa
      rpm_retries += 1;
      // E zera o valor do RPM
      rpm = 0;
      // Se atingiu o máximo de tentativas, define como TRUE a flag de erro do OBD
      if (rpm_retries >= RPM_CMD_RETRIES) obd_error_flag = true;
   }
   // No caso de haver erro na comunicação OBD, zera o valor do rpm e das tentativas
   if ((obd_error_flag == true)) {
      rpm = 0;
      rpm_retries = 0;
   }
}

//-----------------------------------------------------//
//--------Recebe o valor do THROTTLE do OBD------------//
//---------Converte para um valor decimal--------------//
//-----------------------------------------------------//
void throttle_calc() {
   boolean prompt, valid;  
   char recvChar;
   char bufin[12];
   int i;
   // Se não houver erro com a conexão do OBDII
   if (!(obd_error_flag)) {
      valid = false;
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
         if (recvChar == 62) prompt = true;
      }

      // Se os 4 primeiros carácteres após o comando forem 4111
      if ((bufin[5] == '4') && (bufin[6] == '1') && (bufin[7] == '1') && (bufin[8] == '1')) {
         // Então a resposta do THROTTLE é válida
         valid=true;
      } else {
         valid=false;
      }
      Serial.println("bufin tr: ");
      Serial.println(bufin);
      Serial.println("valid tr: ");
      Serial.println(valid);
      // No caso da resposta válida
      if (valid) {
         // Zera as tentaivas e define a flag de erro como FALSE
         throttle_retries = 0;
         throttle_error_flag = false;
         // Para iniciar o cálculo do valor real do THROTTLE
         // É necessário separar o byte hexadecimais da resposta do OBD, por exemplo A=0B
         // A equação é (A / 255) * 100
         // Transforma os bytes em String
         String loadHex(bufin[9]); //11?
         String loadHex2(bufin[10]); //12?
         // Converte-os pra decimal com a função hexToDec
         String loadHexTotal = loadHex + loadHex2;
         int DecimalDecode = hexToDec(loadHexTotal);
         // Faz o cálculo de throttle = (A / 255) * 100
         throttle = round((float(DecimalDecode) / 255) * 100);
      }
      // No caso de um valor inválido do THROTTLE
      if (!valid) {
         // Seta a flag de erro como TRUE
         throttle_error_flag = true;
         // Incrementa uma tentativa
         throttle_retries += 1;
         // E zera o valor do THROTTLE
         throttle = 0;
         // Se atingiu o máximo de tentativas, define como TRUE a flag de erro do OBD
         if (throttle_retries >= THROTTLE_CMD_RETRIES) obd_error_flag = true;
      }
      // No caso de haver erro na comunicação OBD, zera o valor do rpm e das tentativas
      if ((obd_error_flag == true)) {
         throttle = 0;
         throttle_retries = 0;
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
void setup() {
   InitTimersSafe();
   success = SetPinFrequencySafe(servoPin, frequency);
   // Inicializa a serial com a mesma velocidade de comunicação do Bluetooth
   Serial.begin(38400);
   // Define a taxa de comunicação padrão do HC-05
   blueToothSerial.begin(38400);
   // Limpa qualquer dado que estiver nas serial
   Serial.flush();   
   // Limpa os dados que estiverem na serial do Bluetooth
   blueToothSerial.flush();

   while (1) {

      if (Serial.available()) {  
         delay(100);  
         while (Serial.available() > 0) {
            c = Serial.read();
            check += c;
            flag = 1;
         }

         if(flag==1) {
            if (check == "sair") {
               parear = false;
               break;
            }
            if (check == "parear") {
               parear = true;
               break;
            }
         }
      }
   }
   if (parear) {

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
   
   // Funções para testar com o carro
   throttle_calc();
   delay(300);

   if ((throttle < 90) && (throttle > 30) && success) {
      pwmWrite(servoPin, throttle);
   }
   Serial.print(throttle);
   Serial.print("\t");
   Serial.print(rpm);
   Serial.println();

}
