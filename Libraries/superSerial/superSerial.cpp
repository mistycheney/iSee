/*
  Name: Arduino Super Serial Library
  Copyright: UCSD MAE156A 
  Author: David Adams & Yoshio Tsuruta
  Date: 09/02/11 
  Description: Aims to simplify numerical inputs to 
  arduino platforms
*/

#include <stdlib.h>
#include <math.h>
#include <superSerial.h>
#include <Arduino.h>

superSerial::superSerial()
{
}

float superSerial::readFloat(){ 
  char serial_input[9]; 
  serialbuffer(1, serial_input);
  return atof(serial_input);
}
  
      
int superSerial::readInt(){  //allows for all intigers inluding negatives 
  char serial_input[9];   
  serialbuffer(0, serial_input);
  return atoi(serial_input);    
}

void superSerial::serialbuffer(int dot, char *serial_input){
     Serial.flush();
     int i;
     char c;
     int serAva;
     
     while (Serial.available()==0) {}
     delayMicroseconds(10000);
     serAva = Serial.available();
     for (i=0;i<=serAva;i++){
        c=Serial.read();
        if ((c>='0'&& c<='9') || c=='-'){
           serial_input[i] = c;
         }
         else if (dot==1 && c =='.' ){
           serial_input[i] = c;
         }
         else {
            break;
         }
      }
}

superSerial sSerial = superSerial();
