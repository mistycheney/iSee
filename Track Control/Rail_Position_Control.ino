#include "Arduino.h"
#include <Servo.h>
#include <superSerial.h>
#include <digitalWriteFast.h>  

// library for high performance reads and writes by jrraines
// see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
// and http://code.google.com/p/digitalwritefast/

#define DIR1 13
#define PWM1 11

// Output Bounds
#define OUT_MAX 35
#define OUT_MIN -35

// PID Variables
double Setpoint = 0;
double Input = 0; 
double Output = 0; 
double sampleTime = 0; 
double Last = 0;

double pos = 0;
double P, I, D;
double Error, vError, Integral;
double timeChange, runTime;
double Kp=3.3;
double Ki=0.015;
double Kd=0;
double linear_gain = 83.157;

//servo motor;
Servo myservo1;
double angle = 0;
double i = 0;

// Timer Variables
unsigned long startTime, stopTime, now, lastTime, duration;

// Setpoint achivement variables
#define P_ERR_MAX 5
#define V_ERR_MAX 2
int n, nReq = 30;

// Encoder
#define EncoderInterruptA 0
#define EncoderInterruptB 1
#define EncoderPinA 2
#define EncoderPinB 3

//volatile bool _EncoderBSet;
long EncoderTicks = 0;    //up to +- 2147483647
bool Dir = 0; //forward

void setup()
{
  Serial.begin(115200);
  pinMode(DIR1,OUTPUT);

  //Encoder
  pinMode(EncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(EncoderPinA, LOW);  // turn on pullup resistors
  pinMode(EncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(EncoderPinB, LOW);  // turn on pullup resistors

  digitalWrite(DIR1,0);      //stop motor
  analogWrite(PWM1,0);

  attachInterrupt(EncoderInterruptA, InterruptA, CHANGE);  //interrupts for encoder
  attachInterrupt(EncoderInterruptB, InterruptB, CHANGE);

  duration = 10000; // How long to PID for in milli-seconds

  myservo1.attach(10); //servo to pin 10
  myservo1.write(5); 
  angle = 0;

}

void loop()
{
  Serial.println(" ");
  Serial.println(" ");

  Serial.print("Current Position: ");
  Serial.print(pos ,DEC);
  
  Serial.print("  Current Angle: ");
  Serial.println(angle,DEC);
  
  Serial.println();
  
  Serial.println("Please specify the desired linear position in [cm]");
  pos = sSerial.readFloat();
  Serial.println(pos);
  Setpoint = pos * linear_gain;

  Serial.print("Please specify the desired angle in degrees");
  angle = NewVal();
  Serial.println(angle);
  
  // tell servo to go to position in variable 'angle' in 5 steps
  myservo1.write(angle+5);   
  delay(5000);
  //myservo1.write(2*(angle+5)/5);
  //delay(5000);
  //myservo1.write(3*(angle+5)/5);  
  //delay(5000);
  //myservo1.write(4*(angle+5)/5);
  //delay(5000);
  //myservo1.write(5*(angle+5)/5);
  //delay(5000);

  run();
}

int  NewVal() {
  int val;
  val=sSerial.readInt();
  Serial.println(val);
  return val;
}

void run(){
  // Start of the PID code
  startTime = millis();
  lastTime = millis();
  stopTime = startTime;
  timeChange = 0;
  Integral = 0;
  n = 0;
  Last = EncoderTicks;
  while ( n < nReq && (stopTime-startTime) < duration) {
    Input = EncoderTicks;
    Output = ComputePID();

    if (Output > 0) {
      digitalWrite(DIR1,HIGH);
    }
    else {
      digitalWrite(DIR1,LOW);
    }

    analogWrite(PWM1,abs(Output));
    Serial.print("position: ");
    Serial.print(Input/linear_gain,3);
    Serial.print("    Drive: ");
    Serial.print(Output,3);
    Serial.print("    dt: ");
    Serial.print((timeChange)/1000,3); 
    Serial.println();

    // Check the ending Criterion
    if ( abs(Error) < P_ERR_MAX && abs(vError) < V_ERR_MAX) { 
      n++; 
    }

    stopTime = millis();
  }

  runTime = (stopTime - startTime);
  runTime = runTime/1000;

  if ( n == nReq ) {
    Serial.print ("Time (sec): ");
    Serial.println (runTime,3);
  }
  else {
    Serial.print ("Time (sec): ");
    Serial.println (runTime,3);
  }

  analogWrite(PWM1,0);
}

double ComputePID() {
  now = micros();

  timeChange = now-lastTime;
  Error = Setpoint-Input;
  vError = Error-Last;
  Integral = Integral + Error;

  P = Error * Kp;
  I = Integral * Ki * (timeChange/1000000);
  D = vError * Kd / (timeChange/1000000);

  Output = P + I + D;

  if (Output > OUT_MAX) Output = OUT_MAX;
  else if (Output < OUT_MIN) Output = OUT_MIN;

  lastTime = now;
  Last = Error;

  return Output;
}

/*
             _____
 Channel A         |_____    1 1 0 0
 ______
 Channel B   __|      |___   0 1 1 0
 
 */

void InterruptA(){
  if (digitalReadFast(EncoderPinB)!=digitalReadFast(EncoderPinA)){
    EncoderTicks++;
  }
  else {
    EncoderTicks--;
  } 
}

void InterruptB(){
  if (digitalReadFast(EncoderPinB)!=digitalReadFast(EncoderPinA)){
    EncoderTicks--;
  }
  else {
    EncoderTicks++;
  } 
}



