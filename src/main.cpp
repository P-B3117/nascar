#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"
#include <stdio.h>

int lecture_gauche = 0;
int lecture_droite=1;
float vitesse_avance_gauche = 0.44;
float vitesse_avance_droite = 0.44;
float vitesse_tournegauche_gauche = -0.1;
float vitesse_tournegauche_droite = 0.1;
float vitesse_tournedroite_gauche = 0.1;
float vitesse_tournedroite_droite = -0.1;

void tournegauche(int gauche, int droite, float vitesse_tournegauche_gauche, float vitesse_tournegauche_droite ){
 
    MOTOR_SetSpeed(gauche,vitesse_tournegauche_gauche);
    MOTOR_SetSpeed(droite,vitesse_tournegauche_droite);
  
  if (ENCODER_Read(lecture_gauche)<-1942 || ENCODER_Read(lecture_droite)>1942 )
  {
    if (ENCODER_Read(lecture_gauche)<-1942){
        MOTOR_SetSpeed(gauche,0);
      if (ENCODER_Read(lecture_droite)>1942){
        MOTOR_SetSpeed(droite,0);
        return;
       } 
    }
    else if (ENCODER_Read(lecture_droite)>1942){
        MOTOR_SetSpeed(droite,0);

        if (ENCODER_Read(lecture_gauche)<-1942){
        MOTOR_SetSpeed(gauche,0);
        return;
       } 
  }


}
}
void tournedroit(int gauche, int droite, float vitesse_tournedroite_gauche,float vitesse_tournedroite_droite){
 
    MOTOR_SetSpeed(gauche,vitesse_tournedroite_gauche);
    MOTOR_SetSpeed(droite,vitesse_tournedroite_droite);
  
  if (ENCODER_Read(lecture_gauche)>1942 || ENCODER_Read(lecture_droite)<-1942 )
  {
    if (ENCODER_Read(lecture_gauche)>1942){
        MOTOR_SetSpeed(gauche,0);
      if (ENCODER_Read(lecture_droite)<-1942){
        MOTOR_SetSpeed(droite,0);
        return;
       } 
    }
    else if (ENCODER_Read(lecture_droite)<-1942){
        MOTOR_SetSpeed(droite,0);

        if (ENCODER_Read(lecture_gauche)>1942){
        MOTOR_SetSpeed(gauche,0);
        return;
       } 
  }


}
}

void avance(int gauche, int droite, float vitesse_avance_gauche,float vitesse_avance_droite)
{
  MOTOR_SetSpeed(gauche,vitesse_avance_gauche);
  MOTOR_SetSpeed(droite,vitesse_avance_droite);
  if ( ENCODER_Read(lecture_gauche)< ENCODER_Read(lecture_droite))
  {
    MOTOR_SetSpeed(gauche,vitesse_avance_gauche+0.05);
  }

  else if ( ENCODER_Read(lecture_gauche)> ENCODER_Read(lecture_droite))
  {
    MOTOR_SetSpeed(gauche,vitesse_avance_gauche-0.05);
  }
  else
  {
    MOTOR_SetSpeed(gauche,vitesse_avance_gauche);
    MOTOR_SetSpeed(droite,vitesse_avance_droite);
  }
  if (ENCODER_Read(lecture_gauche)>20000 || ENCODER_Read(lecture_droite)>20000 )
  {
    if (ENCODER_Read(lecture_gauche)>20000){
        MOTOR_SetSpeed(gauche,0);
      if (ENCODER_Read(lecture_droite)>20000){
        MOTOR_SetSpeed(droite,0);
        return;
       } 
    }
    else if (ENCODER_Read(lecture_droite)>20000){
        MOTOR_SetSpeed(droite,0);

        if (ENCODER_Read(lecture_gauche)>20000){
        MOTOR_SetSpeed(gauche,0);
        return;
       } 
  }

}
}

void setup()
{
  
  BoardInit();
  
}
void loop()
{
  Serial.print(ENCODER_Read(1));
  Serial.print("    ");
  Serial.println(ENCODER_Read(0));
  avance(lecture_gauche, lecture_droite, vitesse_avance_gauche,vitesse_avance_droite);
  //tournegauche(lecture_gauche, lecture_droite, vitesse_tournegauche_gauche,vitesse_tournegauche_droite);
  //tournedroit(lecture_gauche, lecture_droite, vitesse_tournedroite_gauche, vitesse_tournedroite_droite);

}   

