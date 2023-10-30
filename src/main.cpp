#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"
#include <stdio.h>
#define SPEED 0.3
//variable globale couleur
int couleurInitiale = 0;


//variable algo
int etape = 1;

int position; 

void setup()
{
  BoardInit();
  Serial.begin(9600);
  create();
  couleurINIT();
  SERVO_Enable(1);
 SERVO_SetAngle(1,160);
  

  while (ROBUS_IsBumper(3)==false){//!detectionSifflet()==true  ){
    couleurInitiale = getCouleur();
    Serial.println(couleurInitiale);
  }
  
}


void loop()
{

  Serial.print(ENCODER_Read(1));
  Serial.print("    ");
  
  //Serial.println(getCouleur());
 //Serial.println(detectionSifflet());
 //Serial.println(detection_distance_haut());
  switch(couleurInitiale){
    case JAUNE:

      switch(etape){      
      case 1: //ligne depart avance jusqua fin mur
        
        computePIDLigneDroite(3200,3200,SPEED,SPEED);
        Serial.println("j'avance");
        Serial.println(ENCODER_Read(0));
  
        if(detection_distance_haut() > 120){
          Serial.println("IF");
          etape ++;
          }
        break;
        
       
      case 2://on toune de jaune a jaune
       while(ENCODER_Read(LEFT)< 12800){//temporaire encodeur gauche ne marche pus
       computePID(10000,12800,0.2,0.256);//trouver bonne donner
       } 
       Serial.println("je tourne");
        etape++;
        break;

      
      
        
      case 3:// avance a coter poutre jusquau verre et arrete d'avancer sur
        computePIDLigneDroite(3200,3200,SPEED,SPEED);//enttre ligne droite dans le futur
        Serial.println("j'avance");
        if (detection_distance_haut()<20){
          
          Serial.println("servo!!!!!");
          SERVO_SetAngle(1,0);
          delay(20); 
          computePIDLigneDroite(3200,3200,SPEED,SPEED);
          if (getCouleur()==BLANC){
          etape ++;
          }
        }
        else if (getCouleur()==BLANC){
          etape++;
        }
        
        break;
      
      /*case 4:
      
     
      if (getCouleur()==BLANC){
        etape ++;
      }
      break;
*/
      case 4:
      Serial.print("case 4");
      computePID(0,0,0,0);
      break;
      }
    case VERT: //Parcour a effectuer couleur initiale jaune  
    
    break;
      
  }
  }
