#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"
#include <stdio.h>

//variable globale couleur
int couleurInitiale = 0;

//variable algo
int etape = 1;
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

      switch(etape){
      
      case 1: //ligne depart avance jusqua fin mur
        avance();
        if (detection_distance_haut()>120){
          etape ++; 
        }
        break;
        
      case 2://on tourne
        slowDown();// pour tourne droite
        if (detection_distance_haut()<120){
          etape ++; 
        }
        break;
       
      case 3:// avance tapis
        avance();
        if (detection_distance_haut()>120){
          etape ++; 
        }
        break;
      case 4://on tourne encore
        slowDown();// pour tourne droite
        etape ++;
        break;      

    }
      break;
    

    case JAUNE: //Parcour a effectuer couleur initiale jaune  
    break;
    }
    }