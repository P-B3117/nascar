#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
  
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
    /*else if(tension_suiveur > 2.3 && tension_suiveur<3){
        option=GAUCHE;
        Serial.println("gauche   ");
        Serial.println(option);
    }*/
    /*else if(tension_suiveur>1 && tension_suiveur<1.5){
         int option=CENTRE;
        Serial.println("centre   ");
        Serial.println(option);
    }
    /*else if (tension_suiveur>0.3 && tension_suiveur<1){
        option=DROITE;
        Serial.println("droite   ");
        Serial.println(option);
    }*/
    else if(tension_suiveur>3.8 && tension_suiveur<4.8){
        option=DROITE_SUIVEUR;
        Serial.println("gauche et centre   ");
        Serial.println(option);
    }
    else if (tension_suiveur>3 && tension_suiveur<3.8){
        option=CENTRE;
        Serial.println("gauche et droite   ");
        Serial.println(option);
    }
    else if (tension_suiveur>1.5 && tension_suiveur<2.3){
        option=GAUCHE_SUIVEUR;
        Serial.println("centre et droite  ");
        Serial.println(option);
    }
    /*else if(tension_suiveur>4.8){
        option=TOUT;
        Serial.println("tout   ");
        Serial.println(option);
        
    }*/
    return option;
}
bool extreme_droite(){
    float tension_extreme_gauche=analogRead(A8);
    //Serial.println(tension_extreme_gauche);
    if(tension_extreme_gauche>490){
        return true;
    }
        else {
            return false;
        }
    
}
bool extreme_gauche(){
    float tension_extreme_droite=analogRead(A2);
    //Serial.println(tension_extreme_droite);
    if(tension_extreme_droite>200){
        return true;
    }
    else{
        return false;
    }


}



void suiveur_ligne(float power){
    MOTOR_SetSpeed(RIGHT,power);
    MOTOR_SetSpeed(LEFT,power);

   
    if(detecteur_ligne()==CENTRE){
        MOTOR_SetSpeed(MOTORGAUCHE,power);
        MOTOR_SetSpeed(MOTORDROITE,power);
        Serial.println("tout dorit");
    }
    else if(detecteur_ligne()==DROITE_SUIVEUR){
        MOTOR_SetSpeed(MOTORGAUCHE,power+0.1);
        MOTOR_SetSpeed(MOTORDROITE,0);
        while (detecteur_ligne()==DROITE_SUIVEUR){
        Serial.println("vers la droite");
        }
    }
    else if(detecteur_ligne()==GAUCHE_SUIVEUR){
        MOTOR_SetSpeed(MOTORGAUCHE,0);
        MOTOR_SetSpeed(MOTORDROITE,power+0.1);
        while(detecteur_ligne()==GAUCHE_SUIVEUR){
        Serial.println("vers la gauche");
        }
    }  
    else if (extreme_droite()==true)  {
        MOTOR_SetSpeed(MOTORGAUCHE,power/2);
        MOTOR_SetSpeed(MOTORDROITE,-power/2);
        while( extreme_droite()==true){}
    } 
     else if (extreme_gauche()==true)  {
        MOTOR_SetSpeed(MOTORDROITE,-power/2);
        MOTOR_SetSpeed(MOTORGAUCHE,power/2);
        while( extreme_gauche()==true){}
    } 
}

/*void suiveur_ligne_blanc(float power){
    if (detecteur_ligne()==GAUCHEETCENTRE){
        MOTOR_SetSpeed(LEFT,power);
        MOTOR_SetSpeed(RIGHT,power*1.5);
        while (detecteur_ligne()==DROITE) {}
    }
    else if (detecteur_ligne()==CENTREETDROITE){
        MOTOR_SetSpeed(LEFT,power*1.5);
        MOTOR_SetSpeed(RIGHT,power);   
        while (detecteur_ligne()==CENTREETDROITE){}
    }
    else{
        MOTOR_SetSpeed(LEFT,power);
        MOTOR_SetSpeed(RIGHT,power);
    }
    
}*/