#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"
#include <stdio.h>
#define SPEED 0.3
int couleurInitiale;


//variable algo
int etape =1;// 1;à
unsigned long long beginmillis;

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


  while (!ROBUS_IsBumper(3) && analogRead(A13) < 600){
    Serial.print("in loop");
    Serial.print("          ");
    Serial.println(analogRead(A13));
    couleurInitiale = getCouleur();
    Serial.println(couleurInitiale);
  }
  
}


void loop()
{
  if (couleurInitiale == JAUNE) couleurInitiale = ROUGE;
  else if (couleurInitiale == VERT) couleurInitiale = BLEU;
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
        /*
       tourne(couleurInitiale);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
       etape++;
       */
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
        /*
       
       tourne(couleurInitiale);
        ENCODER_Reset(0);
        ENCODER_Reset(1);
       etape++;
       */
        break;
      case 5:
      
      computePIDLigneDroite(3200,3200,SPEED,SPEED);
      Serial.println("avance");
      if (detection(PROXDROITE)==PROXTOUT ||detection(PROXDROITE)==PROXROUGE || detection(PROXDROITE)==PROXVERT ){
          SERVO_SetAngle(0,170);
          beginmillis = millis();
        }

        if (millis() >= 1000 + beginmillis) SERVO_SetAngle(0,90);
      if (ENCODER_Read(LEFT)>4600){
        
        etape++;
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        break;
      }

      case 6:
      Serial.println("case 6");
      suiveur_mur_droit(0.2);
       //rajouter detectio cup
        if (detection(PROXDROITE)==PROXTOUT ||detection(PROXDROITE)==PROXROUGE || detection(PROXDROITE)==PROXVERT ){
          SERVO_SetAngle(0,170);
          beginmillis = millis();
        }

        if (millis() >= 1000 + beginmillis) SERVO_SetAngle(0,90);
        
        if (detection_distance_droite()>120){
            SERVO_SetAngle(0,90);
            ENCODER_ReadReset(RIGHT);
            ENCODER_ReadReset(LEFT);
            etape ++;
          }
          break;      
      
      
      case 7:
      Serial.println("case 7");
      computePID(2000,2000,0.2,0.2);
      
        if (detection(PROXDROITE)==PROXTOUT ||detection(PROXDROITE)==PROXROUGE || detection(PROXDROITE)==PROXVERT ){
          SERVO_SetAngle(0,170);
          beginmillis = millis();
        }

        if (millis() >= 1000 + beginmillis) SERVO_SetAngle(0,90);
        
      if (ENCODER_Read(RIGHT)>=2000){
        SERVO_SetAngle(0,90);
        ENCODER_Reset(RIGHT);
        ENCODER_Reset(LEFT);
        etape++;
      }
      break;

    case 8: 
    Serial.println("case 8");
    computePID(0,3884,0,0.2);
    if (ENCODER_Read(LEFT)>=3884){
      ENCODER_ReadReset(RIGHT);
      ENCODER_ReadReset(LEFT);
      etape++;
    }
    break;
    case 9:
    Serial.println("case 9");
    computePID(3500,3500,0.2,0.2);
    if (ENCODER_Read(LEFT)>=3500){
      ENCODER_ReadReset(RIGHT);
      ENCODER_ReadReset(LEFT);
      etape++;
    }

    break;

    case 10:
    Serial.println("case 10");
    computePID(1942,-1942,0.2,-0.2);
    if (ENCODER_Read(RIGHT)>=1942){
      ENCODER_ReadReset(RIGHT);
      ENCODER_ReadReset(LEFT);
      etape++;
    }
    break;
    case 11:
    Serial.println("suiveur ligne");
    
    if(detection(PROXDEVANT) == PROXTOUT or detection(PROXDEVANT) == PROXVERT or detection(PROXDEVANT) == PROXROUGE){
      etape++;
    }
    if(suiveur_ligne(0.15) > 28000){
      etape+=3;
    }
    break;

    case 12:
    computePID(0,0,0,0);
    delay(200);
    SERVO_SetAngle(1,100);
    delay(500);
    etape++;
    break;

  case 13:
  if(suiveur_ligne(0.15) > 28000){
  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
  etape++;
  }
  break;
  
  case 14:
  Serial.println("case 14");
  computePID(0,2000,0,0.2);
  if(ENCODER_Read(LEFT)>=2000){
    ENCODER_Reset(RIGHT);
    ENCODER_Reset(LEFT);
    etape++;
  }
  break;
  
  case 15:
  Serial.println("case 15");
  computePIDLigneDroite(3200,3200,0.2,0.2);
  if (detection_distance_droite()<15){
   ENCODER_Reset(RIGHT);
   ENCODER_Reset(LEFT);
  etape ++;
  }

  break;    

  case 16:
  suiveur_mur_droit(0.2);
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
      computePID(6929,10980,0.2,0.32);
      Serial.println("tourne vert");
      if(ENCODER_Read(LEFT)>=10980){
      ENCODER_Reset(LEFT);
      ENCODER_Reset(RIGHT);
      etape ++;

    }
    break;

    case 3:
      computePIDLigneDroite(3200,3200,SPEED,SPEED);
      Serial.println("avance");
      if (ENCODER_Read(LEFT)>8400){
        etape++;
        ENCODER_ReadReset(0);
        ENCODER_ReadReset(1);
      }
      break;


      case 4:
      computePID(6929,10980,0.2,0.32);//ajouter bonne valeur
      if(ENCODER_Read(LEFT)>=10980){//bonne vakeur
      ENCODER_Reset(LEFT);
      ENCODER_Reset(RIGHT);
      etape ++;

    }
    break;

    case 5:
      
      computePIDLigneDroite(500,500,SPEED,SPEED);
      Serial.println("avance");
      if (detection(PROXGAUCHE)==PROXTOUT ||detection(PROXGAUCHE)==PROXROUGE || detection(PROXGAUCHE)==PROXVERT ){
          SERVO_SetAngle(0,20);
          beginmillis = millis();
        }

        if (millis() >= 1000 + beginmillis) SERVO_SetAngle(0,90);
      if (ENCODER_Read(LEFT)>500){
        
        etape++;
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        break;
      }

    case 6:
    int beginmillis;

    suiveur_mur_gauche(0.2);
    
      if (detection(PROXGAUCHE)==PROXTOUT ||detection(PROXGAUCHE)==PROXROUGE || detection(PROXGAUCHE)==PROXVERT ){
          SERVO_SetAngle(0,20);
          beginmillis = millis();
        }

        if (millis() >= 1000 + beginmillis) SERVO_SetAngle(0,90);
        
        if (detection_distance_gauche()>100){
            SERVO_SetAngle(0,90);
            ENCODER_ReadReset(RIGHT);
            ENCODER_ReadReset(LEFT);
            etape++;
          }
    break;

    case 7:
    computePIDLigneDroite(3200,3200,0.2,0.2);
    if (ENCODER_Read(RIGHT)>1900){
      ENCODER_Reset(RIGHT);
      ENCODER_Reset(LEFT);
      etape ++;
    }
    break;

    case 8:
    Serial.println("suiveur ligne");
    if(detection(PROXDEVANT) == PROXTOUT or detection(PROXDEVANT) == PROXVERT or detection(PROXDEVANT) == PROXROUGE){
      etape++;
    }
    if(suiveur_ligne(0.15) > 28700){
      etape+=3;
    }
    break;

    case 9:
    computePID(0,0,0,0);
    delay(200);
    SERVO_SetAngle(1,100);
    delay(500);
    etape++;
    break;

  case 10:
  if(suiveur_ligne(0.15) > 28700){
  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
  etape++;
  }
  break;
  
  case 11:
  Serial.println("case 10");
  computePID(0,2000,0,0.2);
  if(ENCODER_Read(LEFT)>=2000){
    ENCODER_Reset(RIGHT);
    ENCODER_Reset(LEFT);
    etape++;
    beginmillis = millis();
  }
  break;
  
  case 12:
  Serial.println("case 11");

  
  if (millis() >= beginmillis && millis() <= 5000 + beginmillis){
  computePIDLigneDroite(3200,3200,0.2,0.2);
  }
  else if (millis() >= 5000 + beginmillis && millis() <= 5350 + beginmillis){
  computePID(2000,2000,0,0.2);
  }

  if (detection_distance_droite()<15){
   ENCODER_Reset(RIGHT);
   ENCODER_Reset(LEFT);
  etape ++;
  }

  break;    

  case 13:
  suiveur_mur_droit(0.2);
  break;
   
  }
}

}