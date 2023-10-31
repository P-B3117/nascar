#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"
#include <stdio.h>
#define SPEED 0.3
//variable globale couleur
int couleurInitiale;


//variable algo
int etape =1;// 1;

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
  //computePIDSuiveurMur(3200,3200,0.3,0.3,detection_distance_haut(),80)
  //computePID(3200,3200,0.3,0.3);
  Serial.print(ENCODER_Read(1));
  Serial.print("    ");
  Serial.println(ENCODER_Read(0));
  
 //Serial.println(getCouleur());
 //Serial.println(detectionSifflet());
 
 /*Serial.print("bas:");
 Serial.println(detection_distance_haut());*/
  switch(couleurInitiale){
    case JAUNE:

      switch(etape){      
      case 1: //ligne depart avance jusqua fin mur
        
        computePIDLigneDroite(3200,3200,SPEED,SPEED);
        Serial.println("j'avance");

        if(detection_distance_haut() > 120){
          Serial.println("etape++");
          etape ++;
          ENCODER_ReadReset(1);
          ENCODER_ReadReset(0);
          }
        break;
        
       
      case 2://on tourne de jaune a jaune
       //while(ENCODER_Read(LEFT)< 12800)
       //temporaire encodeur gauche ne marche plus
      if(ENCODER_Read(1<14488)){
       computePID(14488,18582, 0.200, 0.257);
      }
       
       
       Serial.println("je tourne");
       if (ENCODER_Read(1)>=14488){
        ENCODER_ReadReset(0);
        ENCODER_Read(1);
        etape++;
        Serial.println("fin tourner");
        }

        break;

      
      case 3:
      computePIDLigneDroite(3200,3200,SPEED,SPEED);
      if (ENCODER_ReadReset(0)<8148){
        etape++;
        ENCODER_ReadReset(0);
        ENCODER_ReadReset(1);
      }

        break;
      
      case 4:
       if(ENCODER_Read(1<14488)){
       computePID(14488,18582, 0.200, 0.257);
      Serial.println("je tourne");
      }
       
       
      
       if (ENCODER_Read(1)>=14488){
        ENCODER_ReadReset(0);
        ENCODER_ReadReset(1);
        delay(20);
        etape++;
        Serial.println("fin tourner");
        }
        break;
      case 5:
      Serial.println("case 5");
      computePIDLigneDroite(3200,3200,SPEED,SPEED);
        if (detection_distance_haut()<20){
          Serial.println("servo!!!!!");
          position=ENCODER_Read(1);
          SERVO_SetAngle(1,0);
          delay(20); 
          }
        if (getCouleur()==BLANC){
            SERVO_SetAngle(1,160);
            etape ++;
          }
          break;      
      
      case 6:
      computePID(0,0,0,0);
      break;
      }
    /*case VERT: //Parcour a effectuer couleur initiale jaune  
    
    break;*/
   
  }
}
