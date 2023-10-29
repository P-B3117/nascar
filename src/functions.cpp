#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"

//définir les fonctions ici

int detecteur_ligne()
{
    int option;
   float tension_suiveur=analogRead(A5)*5.0/1023.0;
   Serial.println(tension_suiveur);
    if(tension_suiveur<0.3){
        option=AUCUN;
        Serial.println("aucun   ");
        Serial.println(option);
        return option;

    }
    else if(tension_suiveur > 2.3 && tension_suiveur<3){
        option=GAUCHE;
        Serial.println("gauche   ");
        Serial.println(option);
    }
    else if(tension_suiveur>1 && tension_suiveur<1.5){
         int option=CENTRE;
        Serial.println("centre   ");
        Serial.println(option);
    }
    else if (tension_suiveur>0.3 && tension_suiveur<1){
        option=DROITE;
        Serial.println("droite   ");
        Serial.println(option);
    }
    else if(tension_suiveur>3.5 && tension_suiveur<4.3){
        option=GAUCHEETCENTRE;
        Serial.println("gauche et centre   ");
        Serial.println(option);
    }
    else if (tension_suiveur>3 && tension_suiveur<3.5){
        option=GAUCHEETDROITE;
        Serial.println("gauche et droite   ");
        Serial.println(option);
    }
    else if (tension_suiveur>1.5 && tension_suiveur<2.3){
        option=CENTREETDROITE;
        Serial.println("centre et droite  ");
        Serial.println(option);
    }
    else if(tension_suiveur>4.3){
        option=TOUT;
        Serial.println("tout   ");
        Serial.println(option);
        
    }
    return option;
}

void suiveur_ligne(float power){
    MOTOR_SetSpeed(MOTORGAUCHE,power);
    MOTOR_SetSpeed(MOTORDROITE,power);
    /*if(detecteur_ligne()==GAUCHEETDROITE){
        MOTOR_SetSpeed(MOTORGAUCHE,power);
        MOTOR_SetSpeed(MOTORDROITE,power);
        Serial.println("tout dorit");
    }*/
    if(detecteur_ligne()==GAUCHEETCENTRE){
        MOTOR_SetSpeed(MOTORGAUCHE,power);
        MOTOR_SetSpeed(MOTORDROITE,power/2);
        while (detecteur_ligne()!=GAUCHEETDROITE){
        Serial.println("vers la droite");
        }
    }
    else if(detecteur_ligne()==CENTREETDROITE){
        MOTOR_SetSpeed(MOTORGAUCHE,power/2);
        MOTOR_SetSpeed(MOTORDROITE,power);
        while(detecteur_ligne()!=GAUCHEETDROITE){
        Serial.println("vers la gauche");
        }
    }
    /*else if(detecteur_ligne()==TOUT){
        MOTOR_SetSpeed(MOTORGAUCHE,power);
        MOTOR_SetSpeed(MOTORDROITE,0);
        while(detecteur_ligne()==TOUT){
            Serial.println("tourne droite");
        }
       
    }*/
    

}