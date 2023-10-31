#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"

//mettez vos fonctions dans functions.h et functions.h
//j'ai créer la fonction exemple pour vous guider

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  BoardInit();
  //MOTOR_SetSpeed(MOTORGAUCHE,0.2);
  //MOTOR_SetSpeed(MOTORDROITE,0.2);
}

void loop() {
 suiveur_ligne(0.2);
 //Serial.println(detecteur_ligne());
 //delay(500);
 //suiveur_ligne(0.2);
 //Serial.println(analogRead(A8));
// Serial.println(extreme_gauche());
}