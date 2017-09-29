  #include <PWM.h> 
  #include <SoftwareSerial.h>

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
  #define INTERACTIONS 200
  // Define o número de interações para resposta do motorista
  #define RESPONSETIME 100
  // Define a tolerância do valor da velocidade em relação ao target
  #define TOL 1
  // Define a faixa para manter a velocidade constante
  #define FAIXA 5
  // Define a tolerância da velocidade em relação ao target para o acúmulo da soma da componente integrativa do PID
  #define TOLKI 15

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
  int velocityBand = 100, velocity = 0, velocityMedia = 0;
  // Variáveis para leitura da posição do cabo do acelerador
  int throttle = 0, throttleMedia = 0;
  // Contadores de interações 
  int countInteractions = 0, count = 0;
  
  // Flags de controle
  boolean goingUp = true; // Flag para definir se a velocidade está subindo ou descendo
  boolean flagLight = false; // Flag para acionar o LED
  boolean flagBreak = true; // Flag para acionar o uso do freio
  boolean flagThrottleDown = false; // Flag para acionar o cálculo do PID  
  boolean flagActivatePID = false; // Flag para ativar o uso do PID
  int32_t frequency = 50;
  bool success = false;
  
  void setup() {
    // Define o LED como saída
    pinMode(LED, OUTPUT);  
    InitTimersSafe();      
    // Inicializa serial
    Serial.begin(38400);    
    // Limpa qualquer dado que estiver na serial
    Serial.flush();
    // Define o pino 9 para o PWM do servo
    success = SetPinFrequencySafe(9, frequency); 
    // Inicializa a váriavel de tempo
    lastProcess = millis(); 
  
  }
  
  void loop() {  
    // Lê o valor do potenciômetro usando a média das leituras para evitar ruídos
    velocityMedia = throttleMedia = 0;

    for (int j = 0; j < 100; j++) {
        // Lê os valores dos potenciômetros
        throttle = analogRead(A1);
        velocity = analogRead(A0);
        // Mapeia os valores dos potenciômetros de 0-1023 para os ângulos de 0-179 
        velocity = map(velocity, 240, 1000, 35, 179);
        throttle = map(throttle, 0, 1023, 0, 179);
        // Acumula os valores para tirar a média e assim diminuir ruídos
        velocityMedia += velocity;
        throttleMedia += throttle;
    }
    // Tira a média
    velocity = velocityMedia / 100;
    throttle = throttleMedia / 100;

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

    // Define a posição do ângulo do servo
    pwmWrite(9, pwm);

    delay(1);  

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
    Serial.print(flagThrottleDown);
    Serial.print("\t");
    Serial.print(flagActivatePID);
    Serial.println();
  }


  void IA() {
    // Caso o cálculo do PID não esteja habillitado
    if (!flagThrottleDown) {
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
        if (count > 5) { 
          flagActivatePID = true;
        } else {
          // Caso contrário, zera a soma do fator integrativo, pois o erro acumulado pode ser muito grande e atrapalhar a primeira correção do PID
          sum = 0;
        }
        // Espera 100 interações para o motorista tomar algum ação
        if (count < RESPONSETIME) {
          // Caso solte o pedal, ativa o cálculo do PID
          if (throttle < 20) {
            flagThrottleDown = true;
            //servo = 40; ****no caso do carro, testar se vale a pena zerar o servo****
          }            
        } else {
          // Caso contrário, desliga o LED e zera os contadores
          digitalWrite(LED, LOW);
          flagLight = false;
          countInteractions = 0;
          count = 0;
        }
        // Incrementa a contagem 
        count++;
      }
    } else {
      // Caso o cáculo do PID esteja habilitado e o motorista ultrapassar o target da velocidade
      if (throttle > targetVelocity) {
        // Desativa o cálculo e o uso do PID, apaga o LED, zera os contadores e redefine a faixa de velocidae
        flagThrottleDown = false;
        flagActivatePID = false;
        digitalWrite(LED, LOW);
        flagLight = false;
        countInteractions = 0;
        count = 0;
        velocityBand = velocity;
      }
    }
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
