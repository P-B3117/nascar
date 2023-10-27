#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"
#include <stdio.h>

//variable globale couleur
int couleurInitiale = 0;


//variable algo
int etape = 3;//1;
void setup()
{
  BoardInit();
  Serial.begin(9600);
  create();
  couleurINIT();
  SERVO_Enable(1);
// SERVO_SetAngle(1,180);
  

  while (!detectionSifflet()==true ){
    couleurInitiale = getCouleur();
    Serial.println(couleurInitiale);
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
 //Serial.println(detection_distance_haut());
  switch(couleurInitiale){
    case JAUNE:

      switch(etape){      
      case 1: //ligne depart avance jusqua fin mur
        
        computePIDLigneDroite(3200,3200,0.3,0.3);
        Serial.println("j'avance");
        
        if(detection_distance_haut() > 120){
          etape ++;
          }
        break;
        
       
      case 2://on toune de jaune a jaune
       computePID(10000,12800,0.2,0.256);//trouver bonne donner 
       Serial.println("je tourne");
        etape++;
        break;

      
      
        
      case 3:// avance a coter poutre jusqua verre
        computePIDLigneDroite(3200,3200,0.3,0.305);//enttre ligne droite dans le futur
        Serial.println("j'avance");
        if (detection_distance_haut()<20){
          etape ++;
        }
        break;
      
      case 4:
      Serial.println("servo!!!!!");
      SERVO_SetAngle(1,40);
      delay(20);
      computePIDLigneDroite(3200,3200,0.3,0.305);
      if (getCouleur()==BLANC){
        etape ++;
      }
      break;

      case 5:
      computePID(0,0,0,0);
      break;
     }
    case VERT: //Parcour a effectuer couleur initiale jaune  
    
    break;
   
  }
}