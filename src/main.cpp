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
 

  while (!detectionSifflet()==true ){
    couleurInitiale = getCouleur();
    
  }
  
}


void loop()
{
  /*Serial.print(ENCODER_Read(1));
  Serial.print("    ");
  Serial.println(ENCODER_Read(0));
  */
  //Serial.println(getCouleur());
 //Serial.println(detectionSifflet());
// Serial.println(detection_distance_bas());
  switch(couleurInitiale){
    case VERT:
   
    avance();
    if (detection_distance_haut()>120){
      Serial.println("****************************************************************************************************************");
      slowDown();
    }
   
    break;
  

    case JAUNE: //Parcour a effectuer couleur initiale jaune
    
    break;
  }


  //tournegauche(lecture_gauche, lecture_droite, vitesse_tournegauche_gauche,vitesse_tournegauche_droite);
  //tournedroit(lecture_gauche, lecture_droite, vitesse_tournedroite_gauche, vitesse_tournedroite_droite);

}