#include <PID_v1.h>
#define R 2
#define PWM 3
#define L 4 

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=0.1, aggKi=0.2, aggKd=1;
double consKp=0.1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input = analogRead(A0);
  Setpoint = 500;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = analogRead(A0);

  double gap = abs(Setpoint-Input); //distance away from setpoint
  if(gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();

  if (Input < 500) {
    Serial.print(" R ");
    Serial.println(Output);
    
    digitalWrite(L, HIGH);
    digitalWrite(R, LOW);
    analogWrite(PWM, Output);
  } else {
    Serial.print(" L ");
    Serial.println(Output);
    
    digitalWrite(R, HIGH);
    digitalWrite(L, LOW);
    analogWrite(PWM, Output);
  }
}
