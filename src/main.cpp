#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"
#include <stdio.h>
#define SPEED 0.3
//variable globale couleur
int couleurInitiale;


//variable algo
int etape =1;// 1;à
int beginmillis;

int position; 

void setup()
{
  BoardInit();
  Serial.begin(9600);
  create();
  couleurINIT();
  SERVO_Enable(0);
  SERVO_Enable(1);
  SERVO_SetAngle(0,90);
  SERVO_SetAngle(1,60);


  while (ROBUS_IsBumper(3)==false){//!detectionSifflet()==true  ){
    couleurInitiale = getCouleur();
    Serial.println(couleurInitiale);
  }
  
}


void loop()
{
  //if (couleurInitiale == JAUNE) couleurInitiale = ROUGE;
  //else if (couleurInitiale == VERT) couleurInitiale = BLEU;
  //if(detection(PROXDEVANT)==PROXTOUT){
  //SERVO_SetAngle(1,100);}
//Serial.println(analogRead(A12)*5.0/1023.0);
//suiveur_ligne(0.2);
//Serial.println(extreme_droite());
//delay(500);
//suiveur_mur_droit(0.2);
  //computePIDSuiveurMur(3200,3200,0.3,0.3,detection_distance_haut(),80)
  //computePID(3200,3200,0.3,0.3);
  //Serial.print(ENCODER_Read(1));
  //Serial.print("    ");
 // Serial.println(ENCODER_Read(0));
  
 //Serial.println(getCouleur());
 //Serial.println(detectionSifflet());
 
 /*Serial.print("bas:");
 Serial.println(detection_distance_haut());*/
  switch(couleurInitiale){
    case ROUGE:

      switch(etape){      
      case 1: //ligne depart avance jusqua fin mur
        
        computePIDLigneDroite(3200,3200,SPEED,SPEED);
        Serial.println("j'avance");

        if(detection_distance_droite() > 120){
          Serial.println("etape++");
          etape ++;
          ENCODER_ReadReset(1);
          ENCODER_ReadReset(0);
          }
        break;
        
       
      case 2://on tourne de jaune a jaune
       //while(ENCODER_Read(LEFT)< 12800)
       //temporaire encodeur gauche ne marche plus
      if(ENCODER_Read(RIGHT)<14488){
       computePID(14488,18582, 0.200, 0.257);
      }
       
       
       Serial.println("je tourne");
       if (ENCODER_Read(RIGHT)>=14488 && ENCODER_Read(LEFT)>=18582){
        ENCODER_Reset(RIGHT);
        ENCODER_Reset(LEFT);
        etape++;
        Serial.println("fin tourner");
        }

        break;

      
      case 3:
      computePIDLigneDroite(3200,3200,SPEED,SPEED);
      Serial.println("avance");
      if (ENCODER_Read(LEFT)>8148){
        
        etape++;
        ENCODER_Reset(0);
        ENCODER_Reset(1);
      }

        break;
      
      case 4:
       computePID(14488,18582, 0.200, 0.257);
      Serial.println("je tourne");
      
       
       
      
       if (ENCODER_Read(RIGHT)>=14488 && ENCODER_Read(LEFT)>=18582){
        ENCODER_ReadReset(RIGHT);
        ENCODER_ReadReset(LEFT);
        etape++;
        Serial.println("fin tourner");
        }
        break;
      case 5:
      Serial.println("case 5");
      suiveur_mur_droit(0.2);
       //rajouter detectio cup
        if (detection(PROXDROITE)==PROXTOUT ||detection(PROXDROITE)==PROXROUGE || detection(PROXDROITE)==PROXVERT ){
          SERVO_SetAngle(0,170);
          beginmillis = millis();
        }

        if (millis() >= 2000 + beginmillis) SERVO_SetAngle(0,90);
        
        if (detection_distance_droite()>120){
            SERVO_SetAngle(0,90);
            ENCODER_ReadReset(RIGHT);
            ENCODER_ReadReset(LEFT);

            etape ++;
          }
          break;      
      
      
      case 6:
      Serial.println("case 6");
      computePID(2000,2000,0.2,0.2);
      if (ENCODER_Read(RIGHT)>=2000){
        ENCODER_Reset(RIGHT);
        ENCODER_Reset(LEFT);
        etape++;
      }
      break;

    case 7: 
    Serial.println("case 7");
    computePID(0,3884,0,0.2);
    if (ENCODER_Read(LEFT)>=3884){
      ENCODER_ReadReset(RIGHT);
      ENCODER_ReadReset(LEFT);
      etape++;
    }
    break;
    case 8:
    Serial.println("case 8");
    computePID(3500,3500,0.2,0.2);
    if (ENCODER_Read(LEFT)>=3500){
      ENCODER_ReadReset(RIGHT);
      ENCODER_ReadReset(LEFT);
      etape++;
    }

    break;

    case 9:
    Serial.println("case 9");
    computePID(1942,-1942,0.2,-0.2);
    if (ENCODER_Read(RIGHT)>=1942){
      ENCODER_ReadReset(RIGHT);
      ENCODER_ReadReset(LEFT);
      etape++;
    }
    break;
    case 10:
    Serial.println("suiveur ligne");
    suiveur_ligne(0.2);
    if(detection(PROXDEVANT) == PROXTOUT or detection(PROXDEVANT) == PROXVERT or detection(PROXDEVANT) == PROXROUGE){
      etape++;
    }
    if(detecteur_ligne()==TOUT && extreme_droite()==0 && extreme_gauche()){
      etape+=3;
    }
    break;
    case 11:
    computePID(0,0,0,0);
    SERVO_SetAngle(1,100);
    delay(500);
    etape++;
    break;

  case 12:
  suiveur_ligne(0.2);
  if(detecteur_ligne()==TOUT && extreme_droite()==1){
  Serial.println("AR'JEIN CALISSE!!!");
  if (millis() >= 100 + beginmillis) etape++;
  }
  else beginmillis = millis();
  break;
  
  case 13:
  Serial.println("case 13");
  computePID(0,2100,0,0.2);
  if(ENCODER_Read(LEFT)<=2100){
    ENCODER_Reset(RIGHT);
    ENCODER_Reset(LEFT);
    etape++;
  }
  break;
  case 14:
  computePIDLigneDroite(3200,3200,0.2,0.2);
  if (detection_distance_droite()<20){
   ENCODER_Reset(RIGHT);
   ENCODER_Reset(LEFT);
  etape ++;
  }

  break;    
    }
    break;
    case (BLEU): //Parcour a effectuer couleur initiale vert
    switch(etape){
      case 1:
      computePIDLigneDroite(3200,3200,0.2,0.2);
      if (detection_distance_droite()>120){
        ENCODER_Reset(LEFT);
        ENCODER_Reset(RIGHT);
        etape++;
      }
      break;
      case 2:
      computePID(6929,11023,0.2,0.32);
      Serial.println("tourne vert");
      if(ENCODER_Read(LEFT)>=11023){//bonne vakeur
      ENCODER_Reset(LEFT);
      ENCODER_Reset(RIGHT);
      etape ++;

    }
    break;

    case 3:
      computePIDLigneDroite(3200,3200,SPEED,SPEED);
      Serial.println("avance");
      if (ENCODER_Read(RIGHT)>8148){
        etape++;
        ENCODER_ReadReset(0);
        ENCODER_ReadReset(1);
      }
      break;


      case 4:
      computePID(6929,11023,0.2,0.32);//ajouter bonne valeur
      if(ENCODER_Read(LEFT)>=11023){//bonne vakeur
      ENCODER_Reset(LEFT);
      ENCODER_Reset(RIGHT);
      etape ++;

    }
    break;
    case 5:
    int beginmillis;

    suiveur_mur_gauche(0.2);
    if (detection(PROXGAUCHE)==PROXTOUT ||detection(PROXDROITE)==PROXROUGE || detection(PROXDROITE)==PROXVERT ){
          SERVO_SetAngle(0,10);
          beginmillis = millis();
        }

        if (millis() >= 2000 + beginmillis) SERVO_SetAngle(0,90);
        
        if (detection_distance_droite()>120){
            SERVO_SetAngle(0,90);
            ENCODER_ReadReset(RIGHT);
            ENCODER_ReadReset(LEFT);
            etape++;
          }
    break;

    case 6:
    computePIDLigneDroite(3200,3200,0.2,0.2);
    if (ENCODER_Read(RIGHT)>2200){
      ENCODER_Reset(RIGHT);
      ENCODER_Reset(LEFT);
      etape ++;
    }
    break;

    case 7:
    suiveur_ligne(0.2);
    if(detection(PROXDEVANT)==PROXTOUT){
      etape++;
    }
    break;
    case 8:
    computePID(0,0,0,0);
    SERVO_SetAngle(1,100);
    break;
    }
    break;

    
    
    



    break;
   
  }
}

