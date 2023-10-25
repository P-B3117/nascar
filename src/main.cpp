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
  detection_distance_bas ();
  detection_distance_haut ();
}

void loop() {
 detection_distance_bas();
 detection_distance_haut();
 Serial.print("bas:");
 Serial.println(detection_distance_bas());
Serial.print("     haut:");
 Serial.println(detection_distance_haut());

 delay(500);

}