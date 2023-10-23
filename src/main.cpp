#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"
#include <stdio.h>

float vitesse_avance_gauche = 0.44;
float vitesse_avance_droite = 0.44;
float vitesse_tournegauche_gauche = -0.1;
float vitesse_tournegauche_droite = 0.1;
float vitesse_tournedroite_gauche = 0.1;
float vitesse_tournedroite_droite = -0.1;

void setup()
{
  
  BoardInit();
  
}
void loop()
{
  Serial.print(ENCODER_Read(1));
  Serial.print("    ");
  Serial.println(ENCODER_Read(0));
  avance(vitesse_avance_gauche,vitesse_avance_droite);
  //tournegauche(lecture_gauche, lecture_droite, vitesse_tournegauche_gauche,vitesse_tournegauche_droite);
  //tournedroit(lecture_gauche, lecture_droite, vitesse_tournedroite_gauche, vitesse_tournedroite_droite);

}