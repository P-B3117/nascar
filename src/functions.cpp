#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#define BAS 3
#define HAUT 2



//définir les fonctions ici

float detection_distance_bas (void){
    float tension_bas=ROBUS_ReadIR(BAS);
    float inverse_bas=1/tension_bas;
    float distance_bas=inverse_bas*6839.3-6.3;
    return distance_bas;
}
float detection_distance_haut (void){
    float tension_haut=ROBUS_ReadIR(HAUT);
    float inverse_haut=1/tension_haut;
    float distance_haut=inverse_haut*6839.3-6.3;
    return distance_haut;
}