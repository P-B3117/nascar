#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"
#include <stdio.h>

float vitesse_tournegauche_gauche = -0.1;
float vitesse_tournegauche_droite = 0.1;
float vitesse_tournedroite_gauche = 0.1;
float vitesse_tournedroite_droite = -0.1;


//PID



void setup()
{
  BoardInit();
  Serial.begin(9600);
  create();


}

  int i = 0;

void loop()
{
  Serial.print(ENCODER_Read(1));
  Serial.print("    ");
  Serial.println(ENCODER_Read(0));

  if ( i == 0 ) {  i++; avance(); }
  //tournegauche(lecture_gauche, lecture_droite, vitesse_tournegauche_gauche,vitesse_tournegauche_droite);
  //tournedroit(lecture_gauche, lecture_droite, vitesse_tournedroite_gauche, vitesse_tournedroite_droite);

}