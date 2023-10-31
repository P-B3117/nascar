#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#define BAS 0
#define HAUT 3   



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


void suiveur_mur(float power){
    if (detection_distance_haut()<=5.5){//tres proche
        MOTOR_SetSpeed(RIGHT,power+(power/1.5));
        MOTOR_SetSpeed(LEFT,power/3);
    }
    else if (detection_distance_haut()>5.5 && detection_distance_haut()<7){//centre
        MOTOR_SetSpeed(RIGHT,power);
        MOTOR_SetSpeed(LEFT,power);

    }
    else if (detection_distance_haut()>=7 && detection_distance_haut()<40){//loin
        MOTOR_SetSpeed(RIGHT,power);
        MOTOR_SetSpeed(LEFT,power+power/3);
    }
    
    /*else if (detection_distance_haut()>=11 && detection_distance_haut()<40){//tres loin
        MOTOR_SetSpeed(RIGHT,power);
        MOTOR_SetSpeed(LEFT,power+power/3);
    }*/
    else if(detection_distance_haut()>120){
        MOTOR_SetSpeed(RIGHT,0);
        MOTOR_SetSpeed(LEFT,power);
    }



}