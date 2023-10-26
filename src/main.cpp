#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"
#include <stdio.h>

//variable globale couleur
int couleurInitiale = 0;
int beginMillis;

void setup()
{
  BoardInit();
  Serial.begin(9600);
  create();
  couleurINIT();
 

<<<<<<< Updated upstream
  while (!detectionSifflet()==true ){
    couleurInitiale = getCouleur();
    
=======
  while (!detectionSifflet() && !ROBUS_IsBumper(2)){
    couleurInitiale = 2;
>>>>>>> Stashed changes
  }
  beginMillis = millis();
}


void loop()
{
  if (millis() < beginMillis + 2000)
  {
    tourne();
  }
  else
  {
    slowDown();
  }
  
  /*Serial.print(ENCODER_Read(1));
  Serial.print("    ");
  Serial.println(ENCODER_Read(0));
<<<<<<< Updated upstream
  */
  //Serial.println(getCouleur());
 //Serial.println(detectionSifflet());
// Serial.println(detection_distance_bas());
=======
  
>>>>>>> Stashed changes
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

  */
  //tournegauche(lecture_gauche, lecture_droite, vitesse_tournegauche_gauche,vitesse_tournegauche_droite);
  //tournedroit(lecture_gauche, lecture_droite, vitesse_tournedroite_gauche, vitesse_tournedroite_droite);

}