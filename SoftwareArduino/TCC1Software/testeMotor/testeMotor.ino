#define R 2
#define PWM 3
#define L 4 
#define MIN 40
#define SETTOL 10
//minimo de 70 para o motor comecar a rodar

int target = 500;

float soma = 0;
float lastError = 0;
float ontem = millis();
float Fpot = 0;
int i, j, pot = 0;
int tolerancia = 0;
int toleranciaki = 20;
float kc = 0.2;
float ki = 0.05; //kc/ti;
float kd = 0;//kc*td;

  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(R, OUTPUT);
  pinMode(L, OUTPUT);
  pinMode(PWM, OUTPUT); 

  
}

void loop() {
  // put your main code here, to run repeatedly:

  //media dos valores obtidos pra diminuir o ruido
  Fpot = analogRead(A0);
  for (i = 0; i < 50; i++) {    
    for (j = 0; j < 10; j++) {
      Fpot += analogRead(A0);
    }
    Fpot = Fpot/11;
  }
  pot = (int)(Fpot);
  


  if (Serial.available() > 0) {
    delay(10);
    target = (Serial.read() - '0') * 100 + (Serial.read() - '0') * 10 + (Serial.read() - '0');
    if (target < 020) target = 020;
    if (target > 980) target = 980;
  }
  

  int vel = (int)PID(kc, ki, kd, target, pot);
  
  //delay(10);
  //Serial.print("vel ");
  //Serial.print(vel);
  
  Serial.print(" Pot ");
  Serial.print(pot);
  Serial.print(" Int ");
  Serial.print(soma);

  Serial.print(" Tar ");
  Serial.print(target);

  Serial.print(" Tol ");
  Serial.print(tolerancia);

  if ((pot > (target+toleranciaki)) || (pot < (target-toleranciaki))) {
    soma = 0;
  }

  if (soma > 10) soma = 10;
  if (soma < -10) soma = -10;
  
  if (pot < target-tolerancia) {
    //somando o valor do vel com o 70 para ele rodar, caso seja menor q 70
    vel=vel+MIN;
    Serial.print(" R ");
    Serial.print(vel);
    
    digitalWrite(L, HIGH);
    digitalWrite(R, LOW);
    analogWrite(PWM, vel);
    
  } else if (pot > target+tolerancia) {
    //como o vel fica negativo, subtraimos o 70 pra aaumentar o valor negativo
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
  
  if (pot == target)
      tolerancia = SETTOL; 
         
  if ((pot > target +tolerancia) || (pot < target - tolerancia))
      tolerancia = 0;
      
  Serial.println();
}

float PID(float kc, float ki, float kd, float target, float sense) {
  float error, integral, proportional, derivative, dt;
  
  dt = (millis() - ontem)/1000;
  ontem  = millis();

  error = target-sense;
  soma = soma + (error * dt);
  
  proportional = error * kc;  
  integral = ki * soma;
  derivative = kd * ((error - lastError)/dt);
  
  lastError = error;

  return proportional + integral + derivative;
}

