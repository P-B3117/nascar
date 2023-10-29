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
}

void loop() {
detecteur_ligne();
delay(500);
}