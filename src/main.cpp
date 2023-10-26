#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"

//variable globale couleur
int couleurInitiale = 0;
int beginMillis;

void setup()
{
  BoardInit();
  Serial.begin(9600);
  create();
  couleurINIT();
 

  while (!ROBUS_IsBumper(2)){
    couleurInitiale = VERT;
  }
  beginMillis = millis();
}


void loop()
{
  /*
  if (millis() < beginMillis + 2000)
  {
    tourne();
  }
  else
  {
    slowDown();
  }
  */
  
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
   
    break;
  

    case JAUNE: //Parcour a effectuer couleur initiale jaune
    
    break;
  }

  
  //tournegauche(lecture_gauche, lecture_droite, vitesse_tournegauche_gauche,vitesse_tournegauche_droite);
  //tournedroit(lecture_gauche, lecture_droite, vitesse_tournedroite_gauche, vitesse_tournedroite_droite);

}