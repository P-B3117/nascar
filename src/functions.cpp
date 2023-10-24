#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#define GAUCHE 2
#define DROITE 3



//définir les fonctions ici

float detection_distance_droite (void){
    float tension_droite=ROBUS_ReadIR(DROITE);
    float inverse_droite=1/tension_droite;
    float distance_droite=inverse_droite*6839.3-6.3;
    return distance_droite;
}
float detection_distance_gauche (void){
    float tension_gauche=ROBUS_ReadIR(GAUCHE);
    float inverse_gauche=1/tension_gauche;
    float distance_gauche=inverse_gauche*6839.3-6.3;
    return distance_gauche;
}