#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"

//définir les fonctions ici

int suiveur_ligne()
{
    int option;
   double tension_suiveur=analogRead(A5)*5.0/1024.0;
   Serial.println(tension_suiveur);
    if(tension_suiveur<0.3){
        int option=AUCUN;
        Serial.println("aucun   ");
        Serial.println(option);
        return option;

    }
    else if(tension_suiveur > 2.3 && tension_suiveur<3){
        int option=GAUCHE;
        Serial.println("gauche   ");
        Serial.println(option);
        return option;
    }
    else if(tension_suiveur>1 && tension_suiveur<1.5){
         int option=CENTRE;
        Serial.println("centre   ");
        Serial.println(option);
        return option;
    }
    else if (tension_suiveur>0.3 && tension_suiveur<1){
         int option=DROITE;
        Serial.println("droite   ");
        Serial.println(option);
        return option;
    }
    else if(tension_suiveur>3.5 && tension_suiveur<4.3){
         int option=GAUCHEETCENTRE;
        Serial.println("gauche et centre   ");
        Serial.println(option);
        return option;
    }
    else if (tension_suiveur>3 && tension_suiveur<3.5){
         int option=GAUCHEETDROITE;
        Serial.println("gauche et droite   ");
        Serial.println(option);
        return option;
    }
    else if (tension_suiveur>1.5 && tension_suiveur<2.3){
         int option=CENTREETDROITE;
        Serial.println("centre et droite  ");
        Serial.println(option);
        return option;
    }
    else if(tension_suiveur>4.3){
         int option=TOUT;
        Serial.println("tout   ");
        Serial.println(option);
        return option;
    }
}