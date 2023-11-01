#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"
#define SERVO 1 //0 ou 1
#define ANGLE 90 //angle entre 0 et 180 degré
//mettez vos fonctions dans functions.h et functions.h
//j'ai créer la fonction exemple pour vous guider

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  BoardInit();
   SERVO_Enable(SERVO);
   SERVO_SetAngle(SERVO,30);
   while(ROBUS_IsBumper(3)==false){};
}

void loop() {
  SERVO_SetAngle(SERVO,130);
  
  
 
 
}