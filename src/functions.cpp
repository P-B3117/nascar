#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"

//définir les fonctions ici

int suiveur_ligne()
{
    int option;
   float tension_suiveur=analogRead(A5);
    if(tension_suiveur<0.3){
        int option=AUCUN;
        printf("aucun   ");
        Serial.print(option);
    }
    else if(tension_suiveur > 2.3 && tension_suiveur<3){
        int option=GAUCHE;
        printf("gauche   ");
        Serial.print(option);
    }
    else if(tension_suiveur>1 && tension_suiveur<1.5){
         int option=CENTRE;
        printf("centre   ");
        Serial.print(option);
    }
    else if (tension_suiveur>0.3 && tension_suiveur<1){
         int option=DROITE;
        printf("droite   ");
        Serial.print(option);
    }
    else if(tension_suiveur>3.5 && tension_suiveur<4.3){
         int option=GAUCHEETCENTRE;
        printf("gauche et centre   ");
        Serial.print(option);
    }
    else if (tension_suiveur>3 && tension_suiveur<3.5){
         int option=GAUCHEETDROITE;
        printf("gauche et droite   ");
        Serial.print(option);
    }
    else if (tension_suiveur>1.5 && tension_suiveur<2.3){
         int option=CENTREETDROITE;
        printf("centre et droite  ");
        Serial.print(option);
    }
    else if(tension_suiveur>4.3){
         int option=TOUT;
        printf("tout   ");
        Serial.print(option);
    }
    return option;
}