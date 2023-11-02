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
int lastPosLeft = 0;
int lastPosRight = 0;

int position; 

void setup()
{
  delay(200);
  BoardInit();
  delay(100);
  Serial.begin(9600);
  delay(100);
  create();
  delay(100);
  couleurINIT();
  delay(100);
  SERVO_Enable(0);
  SERVO_Enable(1);
  SERVO_SetAngle(0,90);
  SERVO_SetAngle(1,60);

  while (!ROBUS_IsBumper(3) && analogRead(A13) < 600){
    Serial.print("in loop");
    Serial.print("          ");
    Serial.println(detection_distance_gauche());
    couleurInitiale = getCouleur();
    Serial.println(couleurInitiale);
    delay(30);
  }
  
}


void loop()
{
  if (couleurInitiale == JAUNE) couleurInitiale = ROUGE;
  else if (couleurInitiale == VERT) couleurInitiale = BLEU;
    switch (etape)
    {
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

      case 2: //avance pour clearer le mur
       computePIDLigneDroite(2000,2000,SPEED,SPEED);
        Serial.println("j'avance");

        if(ENCODER_Read(0) >= 2000){
          Serial.println("etape++");
          etape ++;
          ENCODER_ReadReset(1);
          ENCODER_ReadReset(0);
          }
        break;

      case 3: //tourne 90
        computePID(-1942,1942,-0.2,0.2);
        if(ENCODER_Read(0) >= 2000){
          Serial.println("etape++");
          if (couleurInitiale == BLEU) etape ++;
          else etape += 2;
          ENCODER_ReadReset(1);
          ENCODER_ReadReset(0);
          }
      break;

      case 4: //avance vert
       computePIDLigneDroite(3200,320000,SPEED,SPEED);
        Serial.println("j'avance");

        if(ENCODER_Read(0) >= 7018){
          Serial.println("etape++");
          etape += 2;
          ENCODER_ReadReset(1);
          ENCODER_ReadReset(0);
          }
        break;
        
      case 5: //avance jaune
       computePIDLigneDroite(3200,320000,SPEED,SPEED);
        Serial.println("j'avance");

        if(ENCODER_Read(0) >= 11830){
          Serial.println("etape++");
          etape ++;
          ENCODER_ReadReset(1);
          ENCODER_ReadReset(0);
          }
        break;

        case 6: //french le mur comme si y'avait pas de lendemain
        suiveur_mur_droit(0.2);
        break;
    }
}