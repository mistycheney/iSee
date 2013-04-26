/*********************************************
/ This library aims to simplify serial input /
/ on the arduino platform                    /
*********************************************/

#include <superSerial.h>

/***********************************************
/ To include the library in your code, it must /
/ be placed in the libraries folder within the /
/ arduino directory                            /
***********************************************/

int input_I;
float input_F;

void setup(){
  Serial.begin(9600);   //set up serial baud rate
   
}

void loop(){
  Serial.println("Input an Integer...");
  input_I = sSerial.readInt();
  Serial.print("Input: ");
  Serial.println(input_I,DEC);
  Serial.print("input*2: ");
  Serial.println(2*input_I,DEC);
  
  Serial.println("Input a Float...");
  input_F = sSerial.readFloat();
  Serial.print("Input: ");
  Serial.println(input_F,DEC);
  Serial.print("input*2: ");
  Serial.println(2*input_F,DEC);
}
