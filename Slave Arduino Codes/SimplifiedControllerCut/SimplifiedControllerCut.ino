#include <MatrixMath.h>
#include <superSerial.h>
#include <Wire.h>
#include <digitalWriteFast.h>
#include <MemoryFree.h>

#define NO_PORTB_PINCHANGES //to indicate that port b will not be used for pin change interrupts
#include <PinChangeInt.h>   //using version 1.6beta

MatrixMath math;

//Define pin
#define PINA1 A0
#define PINA2 A1
#define DIRA 13
#define PWMA 11

#define PINB1 A2
#define PINB2 A3
#define DIRB 12
#define PWMB 10

#define PINC1 2
#define PINC2 3
#define DIRC 8
#define PWMC 9

#define PIND1 4
#define PIND2 5
#define DIRD 7
#define PWMD 6


////LQR controller////

//Global variable
float Rotation[4][4];


float state[6]={0,0,0,0,0,0};//should I write [6][1]????
float Kstate[4];
float reference[6]={0.91,0.30,0,0,0,0};//Coming from RECEIVE DATA

float Input[4];

//LQR control vector
int torquePWM[4];
int Direction[4];



unsigned long startTime=0,lastTime=0;

//Encoder Positions
double countA = 0;
double countB = 0;
double countC = 0;
double countD = 0;

//Reference Velocity
//int Vref=1; //m/s

//Wheels speed


//Conditions
int NewPath=1;
int Step=0;

//coming from MatLab LQR toolbox, precalculated we can even have various K for different type of navigation i.e: with different weight on Q and R//

/*float K[4][6]={{0.0, 0.7071, 0.2236, 0.0, 0.8468, 0.2493},
  {-0.7071, 0.0, 0.2236, -0.8468, 0.0, 0.2493},
  {0.0, -0.7071, 0.2236, 0.0, -0.8468, 0.2493},
  {0.7071, 0.0, 0.2236, 0.8468, 0.0, 0.2493}};*/
  
float K[4][6]={{0.0, 0.7071, 0.3873, 0.0, 0.8468, 0.1613},
              {-0.7071, 0.0, 0.3873, -0.8468, 0.0, 0.1613},
              {0.0, -0.7071, 0.3873, 0.0, -0.8468, 0.1613},
              {0.7071, 0.0, 0.3873, 0.8468, 0.0, 0.1613}};

float qTOv[3][4]={{     0,   -0.5000,         0,    0.5000},
                  {0.5000,   -0.0000,   -0.5000,    0.0000},
                  {1.6129,    1.6129,    1.6129,    1.6129}};
float vTOV[3][3]={{1,0,0},{0,1,0},{0,0,1}};

//PIN A                     
void funcA1(){
  PCintPort::pinState!=digitalReadFast(PINA2)?countA++:countA--;
}
void funcA2(){
  PCintPort::pinState!=digitalReadFast(PINA1)?countA--:countA++;
}
//PIN B
void funcB1(){
  PCintPort::pinState!=digitalReadFast(PINB2)?countB++:countB--;
}
void funcB2(){
  PCintPort::pinState!=digitalReadFast(PINB1)?countB--:countB++;
}
//PIN C
void funcC1(){
  PCintPort::pinState!=digitalReadFast(PINC2)?countC++:countC--;
}
void funcC2(){
  PCintPort::pinState!=digitalReadFast(PINC1)?countC--:countC++;
}
//PIN D
void funcD1(){
  PCintPort::pinState!=digitalReadFast(PIND2)?countD++:countD--;
}
void funcD2(){
  PCintPort::pinState!=digitalReadFast(PIND1)?countD--:countD++;
}

                     
void setup(){
Serial.begin(115200);
/*WHAT DOES HIGH LOW INPUT MEAN??*/
pinMode(PINA1, INPUT); digitalWrite(PINA1, HIGH);
PCintPort::attachInterrupt(PINA1, &funcA1, CHANGE);  
pinMode(PINA2, INPUT); digitalWrite(PINA2, HIGH);
PCintPort::attachInterrupt(PINA2, &funcA2, CHANGE);
pinMode(DIRA,OUTPUT); digitalWrite(DIRA,LOW);
pinMode(PWMA,OUTPUT); digitalWrite(PWMA,LOW);
  
pinMode(PINB1, INPUT); digitalWrite(PINB1, HIGH);
PCintPort::attachInterrupt(PINB1, &funcB1, CHANGE);  
pinMode(PINB2, INPUT); digitalWrite(PINB2, HIGH);
PCintPort::attachInterrupt(PINB2, &funcB2, CHANGE);
pinMode(DIRB,OUTPUT); digitalWrite(DIRB,LOW);
pinMode(PWMB,OUTPUT); digitalWrite(PWMB,LOW);
  
pinMode(PINC1, INPUT); digitalWrite(PINC1, HIGH);
PCintPort::attachInterrupt(PINC1, &funcC1, CHANGE);  
pinMode(PINC2, INPUT); digitalWrite(PINC2, HIGH);
PCintPort::attachInterrupt(PINC2, &funcC2, CHANGE);
pinMode(DIRC,OUTPUT); digitalWrite(DIRC,LOW);
pinMode(PWMC,OUTPUT); digitalWrite(PWMC,LOW);
  
pinMode(PIND1, INPUT); digitalWrite(PIND1, HIGH);
PCintPort::attachInterrupt(PIND1, &funcD1, CHANGE);  
pinMode(PIND2, INPUT); digitalWrite(PIND2, HIGH);
PCintPort::attachInterrupt(PIND2, &funcD2, CHANGE);
pinMode(DIRD,OUTPUT); digitalWrite(DIRD,LOW);
pinMode(PWMD,OUTPUT); digitalWrite(PWMD,LOW);
}
int buga;
void loop(){
  float Kreference[4]={0,0,0,0};
  float u[4]={0,0,0,0};
  float Kstate[4]={0,0,0,0};
  if (NewPath==1){
    Serial.println("Start");
    buga = sSerial.readFloat();
    NewPath=0;}
  if(abs(state[0]-reference[0])<0.02 && abs(state[1]-reference[1])<0.5){
      analogWrite(PWMA, 0); 
      analogWrite(PWMB, 0); 
      analogWrite(PWMC, 0); 
      analogWrite(PWMD, 0);
      Serial.println("Done");
      while(NewPath=1){}}
  rotationmatrix(state[2]);
  math.MatrixMult((float*)K,(float*)state, 4, 6, 1,(float*)Kstate);//why was not declared?? It's a function!
  math.MatrixMult((float*)K,(float*)reference, 4, 6, 1,(float*) Kreference);
  Serial.println("Kr[:]"); 
  Serial.println(Kreference[0]);
  Serial.println(Kreference[1]);
  Serial.println(Kreference[2]);
  Serial.println(Kreference[3]);
   Serial.println("Kstate[:]"); 
  Serial.println(Kstate[0]);
  Serial.println(Kstate[1]);
  Serial.println(Kstate[2]);
  Serial.println(Kstate[3]);
  u[0]=(Kreference[0]-Kstate[0]);
  u[1]=(Kreference[1]-Kstate[1]);
  u[2]=(Kreference[2]-Kstate[2]);
  u[3]=(Kreference[3]-Kstate[3]);
  Serial.println("u[:]"); 
  Serial.println(u[0]);
  Serial.println(u[1]);
  Serial.println(u[2]);
  Serial.println(u[3]);
  math.MatrixMult((float*)Rotation,(float*)u, 4, 4, 1,(float*)Input);
  Serial.println("Input[:]"); 
  Serial.println(Input[0]);
  Serial.println(Input[1]);
  Serial.println(Input[2]);
  Serial.println(Input[3]);
  PwmDirection();
  Serial.println("Torque[:]"); 
  Serial.println(torquePWM[0]);
  Serial.println(torquePWM[1]);
  Serial.println(torquePWM[2]);
  Serial.println(torquePWM[3]);
  /*Serial.println("Direction[:]"); 
  Serial.println(Direction[0]);
  Serial.println(Direction[1]);
  Serial.println(Direction[2]);
  Serial.println(Direction[3]);*/
  analogWrite(PWMA, torquePWM[0]); 
  digitalWrite(DIRA, Direction[0]);
  analogWrite(PWMB, torquePWM[1]);
  digitalWrite(DIRB, Direction[1]);
  Serial.println(torquePWM[1]);
  Serial.println(Direction[1]);
  analogWrite(PWMC, torquePWM[2]);
  digitalWrite(DIRC, Direction[2]);
  analogWrite(PWMD, torquePWM[3]);
  digitalWrite(DIRD, Direction[3]);
  Serial.println(torquePWM[3]);
  Serial.println(Direction[3]);
  Speed();}

void rotationmatrix(float angle){ //is it needed to put float??//
double coth,sith;
float bubba;
coth=cos(angle);
sith=sin(angle);
Rotation[0][0]=(pow(coth,2)/2 + coth/2 + pow(sith,2)/2)/(pow(coth,2) + pow(sith,2));
Rotation[0][1]=sith/(2*(pow(coth,2) + pow(sith,2)));
Rotation[0][2]=(pow(coth,2)/2 - coth/2 + pow(sith,2)/2)/(pow(coth,2) + pow(sith,2));
Rotation[0][3]=-sith/(2*(pow(coth,2) + pow(sith,2)));
Rotation[1][0]=Rotation[0][3];
Rotation[1][1]=Rotation[0][0];
Rotation[1][2]=Rotation[0][1];
Rotation[1][3]=Rotation[0][2];
Rotation[2][0]=Rotation[0][2];
Rotation[2][1]=Rotation[0][3];
Rotation[2][2]=Rotation[0][0];
Rotation[2][3]=Rotation[0][1];
Rotation[3][0]=Rotation[0][1];
Rotation[3][1]=Rotation[0][2];
Rotation[3][2]=Rotation[0][3];
Rotation[3][3]=Rotation[0][0];}

void PwmDirection(){
  float InputLimit=1.3;
  int i;
  for(i=0;i<=3;i=i++){
     if(Input[i]>0){
     Direction[i]=HIGH;}
     else{
     Direction[i]=LOW;}
     if(abs(Input[i])>InputLimit){
     Input[i]=InputLimit;}
     if(abs(Input[i])<0.03){
     Input[i]=0.0;}
     if(abs(Input[i])<0.35 && abs(Input[i])>0){
     Input[i]=0.35;}
  torquePWM[i]=abs((Input[i]/InputLimit)*243);
}}

void Speed(){
float deltaT=0;
float vlocal[3]={0,0,0};
float Vglobal[3]={0,0,0};
float dX=0, dY=0, dth=0;
float speedwheel[4]={0,0,0,0};
startTime=millis();
Serial.println("startTime"); 
Serial.println(startTime);
/*Serial.println("lastTime"); 
Serial.println(lastTime);*/
deltaT=(startTime-lastTime);//SHOULD BE CONVERTED IN SEC//
speedwheel[0]=countA/(deltaT*9.5);//m/s in theory
speedwheel[1]=countB/(deltaT*9.5);//m/s in theory
speedwheel[2]=countC/(deltaT*9.5);//m/s in theory
speedwheel[3]=countD/(deltaT*9.5);//m/s in theory
/*Serial.println("speedwheel[:]"); 
Serial.println(speedwheel[0]);
Serial.println(speedwheel[1]);
Serial.println(speedwheel[2]); 
Serial.println(speedwheel[3]);*/
countA = 0;
countB = 0;
countC = 0;
countD = 0;
lastTime=millis();
math.MatrixMult((float*)qTOv,(float*)speedwheel, 3, 4, 1,(float*)vlocal);
/*Serial.println("U"); 
Serial.println(vlocal[0]);
Serial.println("V"); 
Serial.println(vlocal[1]);
Serial.println("TH-dot"); 
Serial.println(vlocal[2]);*/
RotationMatrix();
math.MatrixMult((float*)vTOV,(float*)vlocal, 3, 3, 1,(float*)Vglobal); //consider using a 2x2 matrix to save computational time
dX=((Vglobal[0]+state[3])/2)*deltaT/1000; //RK2 integration method//
dY=((Vglobal[1]+state[4])/2)*deltaT/1000; //RK2 integration method//
dth=((Vglobal[2]+state[5])/2)*deltaT/1000;//RK2 integration method//
/*Serial.println("Vx"); 
Serial.println(Vglobal[0]);
Serial.println("vy"); 
Serial.println(Vglobal[1]);
Serial.println("Vth"); 
Serial.println(Vglobal[2]);
Serial.println("dX"); 
Serial.println(dX);
Serial.println("dY"); 
Serial.println(dY);
Serial.println("dth"); 
Serial.println(dth);*/
state[0]=state[0]+dX;
state[1]=state[1]+dY;
state[2]=state[2]+dth;
state[3]=Vglobal[0];
state[4]=Vglobal[1];
state[5]=Vglobal[2];
Serial.println("state[:]"); 
Serial.println(state[0]);
Serial.println(state[1]);
Serial.println(state[2]);
Serial.println(state[3]);
Serial.println(state[4]);
Serial.println(state[5]);
}

void RotationMatrix(){
vTOV[0][0]=cos(state[2]);
vTOV[0][1]=-sin(state[2]);
vTOV[1][0]=-vTOV[0][1];
vTOV[1][1]=vTOV[0][0];}



