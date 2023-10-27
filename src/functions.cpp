#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"



//détecteur de distance
//define
#define BAS 2
#define HAUT 3

//PID
//define


//Variable global Sifflet
int mic ; //valeur de lecture du micro
int amplitude;
bool siffletActif = false;
unsigned long previousMillis = 0;
unsigned long interval = 1000;

//detecteur de couleur
 tcs34725 rgb_sensor;


float detection_distance_bas (void){
    float tension_bas=ROBUS_ReadIR(BAS);
    float inverse_bas=1/tension_bas;
    float distance_bas=inverse_bas*6839.3-6.3;
    return distance_bas;
}
float detection_distance_haut (void){
    float tension_haut=ROBUS_ReadIR(HAUT);
    float inverse_haut=1/tension_haut;
    float distance_haut=inverse_haut*6839.3-6.3;
    return distance_haut;
}

void couleurINIT() { //

  rgb_sensor.begin();
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW); // @gremlins Bright light, bright light!

}

int getCouleur() {
    return rgb_sensor.getData();
}




void tournegauche( float valeurGauche, float valeurDroite ){
 
    MOTOR_SetSpeed(Gauche,valeurGauche);
    MOTOR_SetSpeed(Droite,valeurDroite);

  int encodeurGauche = ENCODER_Read(Gauche);
  int encodeurDroite = ENCODER_Read(Droite); 
  
  if (encodeurGauche<-1942 || encodeurDroite>1942 )
  {
    if (encodeurGauche<-1942){
        MOTOR_SetSpeed(Gauche,0);
      if (encodeurDroite>1942){
        MOTOR_SetSpeed(Droite,0);
        return;
       }     }
    else if (encodeurDroite>1942){
        MOTOR_SetSpeed(Droite,0);

        if (encodeurGauche<-1942){
        MOTOR_SetSpeed(Gauche,0);
        return;
       } 
  }


}
}


void tournedroit(float valeurGauche,float valeurDroite){
 
    MOTOR_SetSpeed(Gauche,valeurGauche);
    MOTOR_SetSpeed(Droite,valeurDroite);

  int encodeurGauche = ENCODER_Read(Gauche);
  int encodeurDroite = ENCODER_Read(Droite);
  
  if (encodeurGauche>1942 || encodeurDroite<-1942 )
  {
    if (encodeurGauche>1942){
        MOTOR_SetSpeed(Gauche,0);
      if (encodeurDroite<-1942){
        MOTOR_SetSpeed(Droite,0);
        return;
       } 
    }
    else if (encodeurDroite<-1942){
        MOTOR_SetSpeed(Droite,0);

        if (encodeurGauche>1942){
        MOTOR_SetSpeed(Gauche,0);
        return;
       } 
  }


 }
}


/*
void avance(float valeurGauche = VITESSE_AVANCE_GAUCHE, float valeurDroite = VITESSE_AVANCE_DROITE, int target = TARGET_POSITION)
{
  bool isDecelerating = 0;

  ENCODER_Reset(Gauche);
  ENCODER_Reset(Droite);
  
  MOTOR_SetSpeed(Gauche,valeurGauche + 0.01);
  MOTOR_SetSpeed(Droite,valeurDroite);

  long encodeurGauche = ENCODER_Read(Gauche);
  long encodeurDroite = ENCODER_Read(Droite);

  float beginMillis = millis();

while (encodeurGauche != target && encodeurDroite != target)
{
  MOTOR_SetSpeed(Gauche,valeurGauche);
  MOTOR_SetSpeed(Droite,valeurDroite);

  int encodeurGauche = ENCODER_Read(Gauche);
  int encodeurDroite = ENCODER_Read(Droite);

  if (valeurGauche < 0.8 && !isDecelerating) //acceleration
  {
    valeurGauche += (millis() - beginMillis)/10000.0;
    valeurDroite += (millis() - beginMillis)/10000.0;
  }

  //ajustement
  if ( encodeurGauche< encodeurDroite)
  {
    OutputRight = 0;
    InputLeft = encodeurGauche;
    SetpointLeft = encodeurDroite;
    myPIDLeft.compute();
    MOTOR_SetSpeed(Gauche, valeurGauche + OutputLeft);
  }

  else if ( encodeurGauche> encodeurDroite)
  {
    OutputLeft = 0;
    InputRight = encodeurDroite;
    SetpointRight = encodeurGauche;
    myPIDRight.compute();
    MOTOR_SetSpeed(Droite, valeurDroite + OutputRight);
  }
  else
  {
    MOTOR_SetSpeed(Gauche,valeurGauche);
    MOTOR_SetSpeed(Droite,valeurDroite);
  }
  if (encodeurGauche> target || encodeurDroite> target )
  {
    if (encodeurGauche> target){
        MOTOR_SetSpeed(Gauche,0);
      if (encodeurDroite> target){
        MOTOR_SetSpeed(Droite,0);
        return;
       } 
    }
    else if (encodeurDroite> target){
        MOTOR_SetSpeed(Droite,0);

        if (encodeurGauche> target){
        MOTOR_SetSpeed(Gauche,0);
        return;
       } 
  }

 }

 Serial.print(encodeurGauche);
 Serial.print("     ");
 Serial.print(OutputLeft);
 Serial.print("     ");
 Serial.print(encodeurDroite);
 Serial.print("     ");
 Serial.print(OutputRight);
 Serial.print("     ");
 Serial.print(valeurGauche);
 Serial.print("     ");
 Serial.print((encodeurGauche/10 + encodeurDroite/10) / 2.0);
 Serial.println();
}
}

void slowDown(int target = TARGET_SLOW, float valeurGauche = VITESSE_AVANCE_GAUCHE, float valeurDroite = VITESSE_AVANCE_DROITE)
 {

  long encodeurGauche = ENCODER_Read(Gauche);
  long encodeurDroite = ENCODER_Read(Droite);

  if ( target/10 - ( (encodeurGauche/10 + encodeurDroite/10) / 2.0)  <= TARGET_SLOW/10  && valeurGauche > VITESSE_AVANCE_GAUCHE)
  {
    valeurGauche -= 0.1;
    valeurDroite -= 0.1;
    //valeurGauche -= (target - ( (encodeurGauche/10 + encodeurDroite/10) / 2.0)) / 2000.0;
    //valeurDroite -= (target - ( (encodeurGauche/10 + encodeurDroite/10) / 2.0)) / 2000.0;
  }
 }
*/

bool detectionSifflet (){

  unsigned long currentMillis = millis();
  mic = analogRead(micpin);
  amplitude = abs(moyenneSifflet - mic); 
  //Serial.println(amplitude);

  if(amplitude>700){
    if(currentMillis-previousMillis > interval){
      siffletActif = true;
      }
    }

  else{
    previousMillis = currentMillis;
    siffletActif = false;
  }
  Serial.println(siffletActif);

  return siffletActif;

}

