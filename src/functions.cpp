#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#define GAUCHE 2
#define DROITE 3

//définir les fonctions ici

void detection_distance (void){
    float distance_droite=ROBUS_ReadIR(DROITE)/1024.0;
    float distance_gauche=ROBUS_ReadIR(GAUCHE)/1024.0;
    Serial.print("      Droite:");
    Serial.print(distance_droite);
    Serial.print("      Gauche:");
    Serial.println(distance_gauche);

}
