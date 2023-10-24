#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#define GAUCHE 2
#define DROITE 3

//définir les fonctions ici

float detection_distance (void){
    float distance=ROBUS_ReadIR(DROITE);
    Serial.println(distance);
    return distance;

}
