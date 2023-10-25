#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"
#include <stdio.h>

//variable globale couleur
int couleurInitiale = 0;

void setup()
{
  BoardInit();
  Serial.begin(9600);
  create();
  couleurINIT();
 

  while (!detectionSifflet()== true){
  }
    couleurInitiale = 2;

}


void loop()
{
  /*Serial.print(ENCODER_Read(1));
  Serial.print("    ");
  Serial.println(ENCODER_Read(0));
  */
  switch(couleurInitiale){
    case JAUNE:
    avance();
    break;



    case VERT: //Parcour a effectuer couleur initiale jaune
    
    break;
  }


  //tournegauche(lecture_gauche, lecture_droite, vitesse_tournegauche_gauche,vitesse_tournegauche_droite);
  //tournedroit(lecture_gauche, lecture_droite, vitesse_tournedroite_gauche, vitesse_tournedroite_droite);

}