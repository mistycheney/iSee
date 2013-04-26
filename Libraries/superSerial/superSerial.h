#ifndef superSerial_h
#define superSerial_h

#include <stdlib.h>
#include <math.h>
#include <Arduino.h>


class superSerial 
{
   public:
       superSerial();
       
       float readFloat();
       int readInt();  
   private:
       void serialbuffer(int,char[]);
};

extern superSerial sSerial;

#endif
