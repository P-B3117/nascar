#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"

//PID
//define
#define pLeft  0.0004
#define iLeft  0.001
#define dLeft  0

#define pRight 0.0004
#define iRight 0.001
#define dRight 0

ArduPID myPIDLeft;
ArduPID myPIDRight;

double   KpLeft = pLeft, KiLeft = iLeft, KdLeft = dLeft;
double   KpRight = pRight, KiRight = iRight, KdRight = dRight;

double SetpointLeft, InputLeft, OutputLeft;
double SetpointRight, InputRight, OutputRight;




//définir les fonctions ici

void create()
{
    myPIDLeft.begin(&InputLeft, &OutputLeft, &SetpointLeft, KpLeft, KiLeft, KdLeft);
    myPIDRight.begin(&InputRight, &OutputRight, &SetpointRight, KpRight, KiRight, KdRight);

    myPIDLeft.setOutputLimits(0,0.1);
    myPIDRight.setOutputLimits(0,0.1);
}

void tournegauche( float valeurGauche, float valeurDroite ){
 
    MOTOR_SetSpeed(Gauche,valeurGauche);
    MOTOR_SetSpeed(Droite,valeurDroite);

  int encodeurGauche = ENCODER_Read(Gauche);
  int encodeurDroite = ENCODER_Read(Droite); 
  
  if (encodeurGauche<-1942 || encodeurDroite>1942 )
  {
    if (encodeurGauche<-1942){
        MOTOR_SetSpeed(Gauche,0);
      if (encodeurDroite>1942){
        MOTOR_SetSpeed(Droite,0);
        return;
       }     }
    else if (encodeurDroite>1942){
        MOTOR_SetSpeed(Droite,0);

        if (encodeurGauche<-1942){
        MOTOR_SetSpeed(Gauche,0);
        return;
       } 
  }


}
}


void tournedroit(float valeurGauche,float valeurDroite){
 
    MOTOR_SetSpeed(Gauche,valeurGauche);
    MOTOR_SetSpeed(Droite,valeurDroite);

  int encodeurGauche = ENCODER_Read(Gauche);
  int encodeurDroite = ENCODER_Read(Droite);
  
  if (encodeurGauche>1942 || encodeurDroite<-1942 )
  {
    if (encodeurGauche>1942){
        MOTOR_SetSpeed(Gauche,0);
      if (encodeurDroite<-1942){
        MOTOR_SetSpeed(Droite,0);
        return;
       } 
    }
    else if (encodeurDroite<-1942){
        MOTOR_SetSpeed(Droite,0);

        if (encodeurGauche>1942){
        MOTOR_SetSpeed(Gauche,0);
        return;
       } 
  }


 }
}

void avance(float valeurGauche = VITESSE_AVANCE_GAUCHE,float valeurDroite = VITESSE_AVANCE_DROITE)
{

  ENCODER_Reset(Gauche);
  ENCODER_Reset(Droite);
  
  MOTOR_SetSpeed(Gauche,valeurGauche);
  MOTOR_SetSpeed(Droite,valeurDroite);

  int encodeurGauche = ENCODER_Read(Gauche);
  int encodeurDroite = ENCODER_Read(Droite);

while (encodeurGauche != TARGET_POSITION && encodeurDroite != TARGET_POSITION)
{

  int encodeurGauche = ENCODER_Read(Gauche);
  int encodeurDroite = ENCODER_Read(Droite);

  if ( encodeurGauche< encodeurDroite)
  {
    OutputRight = 0;
    InputLeft = encodeurGauche;
    SetpointLeft = encodeurDroite;
    myPIDLeft.compute();
    MOTOR_SetSpeed(Gauche, valeurGauche + OutputLeft);
  }

  else if ( encodeurGauche> encodeurDroite)
  {
    OutputLeft = 0;
    InputRight = encodeurDroite;
    SetpointRight = encodeurGauche;
    myPIDRight.compute();
    MOTOR_SetSpeed(Droite, valeurDroite + OutputRight);
  }
  else
  {
    MOTOR_SetSpeed(Gauche,valeurGauche);
    MOTOR_SetSpeed(Droite,valeurDroite);
  }
  if (encodeurGauche> TARGET_POSITION || encodeurDroite> TARGET_POSITION )
  {
    if (encodeurGauche> TARGET_POSITION){
        MOTOR_SetSpeed(Gauche,0);
      if (encodeurDroite> TARGET_POSITION){
        MOTOR_SetSpeed(Droite,0);
        return;
       } 
    }
    else if (encodeurDroite> TARGET_POSITION){
        MOTOR_SetSpeed(Droite,0);

        if (encodeurGauche> TARGET_POSITION){
        MOTOR_SetSpeed(Gauche,0);
        return;
       } 
  }

 }

 Serial.print(encodeurGauche);
 Serial.print("     ");
 Serial.print(OutputLeft);
 Serial.print("     ");
 Serial.print(encodeurDroite);
 Serial.print("     ");
 Serial.print(OutputRight);
 Serial.print("     ");
 Serial.println();
}
}
