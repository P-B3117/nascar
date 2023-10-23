#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"

#define Gauche 0
#define Droite 1

//define pour tourner a gauche
#define VITESSE_TOURNEGAUCHE_GAUCHE
#define VITESSE_TOURNEGAUCHE_DROITE

//define pour tourner a droite
#define VITESSE_TOURNEDROITE_GAUCHE
#define VITESSE_TOURNEDROITE_DROITE

//define pour avancer
#define VITESSE_AVANCE_GAUCHE
#define VITESSE_AVANCE_DROITE




//définir les fonctions ici


void tournegauche( float valeurGauche, float valeurDroite ){
 
    MOTOR_SetSpeed(Gauche,valeurGauche);
    MOTOR_SetSpeed(Droite,valeurDroite);
  
  if (ENCODER_Read(Gauche)<-1942 || ENCODER_Read(Droite)>1942 )
  {
    if (ENCODER_Read(Gauche)<-1942){
        MOTOR_SetSpeed(Gauche,0);
      if (ENCODER_Read(Droite)>1942){
        MOTOR_SetSpeed(Droite,0);
        return;
       }     }
    else if (ENCODER_Read(Droite)>1942){
        MOTOR_SetSpeed(Droite,0);

        if (ENCODER_Read(Gauche)<-1942){
        MOTOR_SetSpeed(Gauche,0);
        return;
       } 
  }


}
}
void tournedroit(float valeurGauche,float valeurDroite){
 
    MOTOR_SetSpeed(Gauche,valeurGauche);
    MOTOR_SetSpeed(Droite,valeurDroite);
  
  if (ENCODER_Read(Gauche)>1942 || ENCODER_Read(Droite)<-1942 )
  {
    if (ENCODER_Read(Gauche)>1942){
        MOTOR_SetSpeed(Gauche,0);
      if (ENCODER_Read(Droite)<-1942){
        MOTOR_SetSpeed(Droite,0);
        return;
       } 
    }
    else if (ENCODER_Read(Droite)<-1942){
        MOTOR_SetSpeed(Droite,0);

        if (ENCODER_Read(Gauche)>1942){
        MOTOR_SetSpeed(Gauche,0);
        return;
       } 
  }


}
}

void avance(float valeurGauche,float valeurDroite)
{
  MOTOR_SetSpeed(Gauche,valeurGauche);
  MOTOR_SetSpeed(Droite,valeurDroite);
  if ( ENCODER_Read(Gauche)< ENCODER_Read(Droite))
  {
    MOTOR_SetSpeed(Gauche,valeurGauche+0.05);
  }

  else if ( ENCODER_Read(Gauche)> ENCODER_Read(Droite))
  {
    MOTOR_SetSpeed(Gauche,valeurGauche-0.05);
  }
  else
  {
    MOTOR_SetSpeed(Gauche,valeurGauche);
    MOTOR_SetSpeed(Droite,valeurDroite);
  }
  if (ENCODER_Read(Gauche)>20000 || ENCODER_Read(Droite)>20000 )
  {
    if (ENCODER_Read(Gauche)>20000){
        MOTOR_SetSpeed(Gauche,0);
      if (ENCODER_Read(Droite)>20000){
        MOTOR_SetSpeed(Droite,0);
        return;
       } 
    }
    else if (ENCODER_Read(Droite)>20000){
        MOTOR_SetSpeed(Droite,0);

        if (ENCODER_Read(Gauche)>20000){
        MOTOR_SetSpeed(Gauche,0);
        return;
       } 
  }

}
}
