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
  detection_distance_droite ();
  detection_distance_gauche ();
}

void loop() {
 detection_distance_droite();
 detection_distance_gauche();
 Serial.print("droite:");
 Serial.println(detection_distance_droite());
Serial.print("     gauche:");
 Serial.println(detection_distance_gauche());

 delay(500);

}