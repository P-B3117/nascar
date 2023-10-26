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
    case BLEU:

      switch(etape){      
      case 1: //ligne depart avance jusqua fin mur
        if(detection_distance_haut() < 120){
        avance();
        Serial.println("j'avance");
        }
        else{
          etape ++;
          }
        break;
        
       
      case 2://on tourne 90 sur jaune
        slowDown();
        Serial.println("j'arrete");
        etape++;
        break;

      case 3:
        tournedroit(0.15,-0.15);
        Serial.println("jai fini");
        if (ENCODER_Read(Gauche)>1942 && ENCODER_Read(Droite)<-1942){
        etape++;
        }
        break;
      }
       /* 
      case 3:// avance tapis
        avance(0.05);
        if (detection_distance_haut()>120){
          etape ++; 
        }
        break;
      case 4://avance jusqua jaune
        avance(0.05);
        if (getCouleur()==JAUNE){
          etape ++;
        }
        break;
      
      case 5:  //avance quand voit poutre 
        avance(0.05);
        if (detection_distance_bas()<70){
          etape ++;
        }
        break;
      case 6:   //arrete quand pu poutre
        avance(0.05);
        if (detection_distance_bas()>120){
              avance(0);
              etape ++;
            }       
        break;
      case 7://on tourne 90
        slowDown();// pour tourne droite 90
        etape ++;
        break;      
*/
 
    

    case JAUNE: //Parcour a effectuer couleur initiale jaune  
    
    break;
    }
  }