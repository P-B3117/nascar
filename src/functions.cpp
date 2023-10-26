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

#define pLeft  0.0006
#define iLeft  0.001
#define dLeft  0

#define pRight 0.0006
#define iRight 0.001
#define dRight 0

ArduPID myPIDLeft;
ArduPID myPIDRight;

double   KpLeft = pLeft, KiLeft = iLeft, KdLeft = dLeft;
double   KpRight = pRight, KiRight = iRight, KdRight = dRight;

double SetpointLeft, InputLeft, OutputLeft;
double SetpointRight, InputRight, OutputRight;


//Variable global Sifflet
int mic ; //valeur de lecture du micro
int amplitude;
bool siffletActif = false;
unsigned long previousMillis = 0;
unsigned long interval = 1000;

//detecteur de couleur
 tcs34725 rgb_sensor;

 //structure pour avancer


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


void create()
{
    myPIDLeft.begin(&InputLeft, &OutputLeft, &SetpointLeft, KpLeft, KiLeft, KdLeft);
    myPIDRight.begin(&InputRight, &OutputRight, &SetpointRight, KpRight, KiRight, KdRight);

    myPIDLeft.setOutputLimits(0,0.1);
    myPIDRight.setOutputLimits(0,0.1);
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

void avance(float speedLimit = SPEEDLIMIT)
{
  if (avancer.isMoving == false)
  {
  avancer.isMoving = true;    
  avancer.beginMillis = millis();
  }

  avancerUpdate();

  avancer.encodeurGauche = ENCODER_Read(Gauche);
  avancer.encodeurDroite = ENCODER_Read(Droite);

  if (avancer.valeurGauche < speedLimit && !avancer.hasAccelerated) //acceleration
  {
    avancer.valeurGauche += (millis() - avancer.beginMillis)/10000.0;
    avancer.valeurDroite += (millis() - avancer.beginMillis)/10000.0;
  }
  else if (avancer.valeurGauche >= speedLimit) {
    avancer.hasAccelerated = 1;
  }

  //ajustement
  if ( avancer.encodeurGauche < avancer.encodeurDroite)
  {
    OutputRight = 0;
    InputLeft = avancer.encodeurGauche;
    SetpointLeft = avancer.encodeurDroite;
    myPIDLeft.compute();
    MOTOR_SetSpeed(Gauche, avancer.valeurGauche + OutputLeft);
  }

  else if ( avancer.encodeurGauche > avancer.encodeurDroite)
  {
    OutputLeft = 0;
    InputRight = avancer.encodeurDroite;
    SetpointRight = avancer.encodeurGauche;
    myPIDRight.compute();
    MOTOR_SetSpeed(Droite, avancer.valeurDroite + OutputRight);
  }
  else
  {
    MOTOR_SetSpeed(Gauche, avancer.valeurGauche);
    MOTOR_SetSpeed(Droite, avancer.valeurDroite);
  }

/*
 Serial.print(avancer.encodeurGauche);
 Serial.print("     ");
 Serial.print(OutputLeft);
 Serial.print("     ");
 Serial.print(avancer.encodeurDroite);
 Serial.print("     ");
 Serial.print(OutputRight);
 Serial.print("     ");
 Serial.print(avancer.valeurGauche);
 Serial.print("     ");
 Serial.print((avancer.encodeurGauche/10 + avancer.encodeurDroite/10) / 2.0);
 Serial.println();
 */
}

void slowDown(int target = TARGET_SLOW)
 {

  while (avancer.encodeurDroite < target &&  avancer.encodeurGauche < target)
  {
  avancerUpdate();

  if ( target/10 - ( (avancer.encodeurGauche/10 + avancer.encodeurDroite/10) / 2.0)  <= TARGET_SLOW/10  && avancer.valeurGauche > VITESSE_AVANCE_GAUCHE)
  {
    avancer.valeurGauche -= 0.1;
    avancer.valeurDroite -= 0.1;
    //valeurGauche -= (target - ( (encodeurGauche/10 + encodeurDroite/10) / 2.0)) / 2000.0;
    //valeurDroite -= (target - ( (encodeurGauche/10 + encodeurDroite/10) / 2.0)) / 2000.0;
  }

   if ( avancer.encodeurGauche < avancer.encodeurDroite)
  {
    OutputRight = 0;
    InputLeft = avancer.encodeurGauche;
    SetpointLeft = avancer.encodeurDroite;
    myPIDLeft.compute();
    MOTOR_SetSpeed(Gauche, avancer.valeurGauche + OutputLeft);
  }

  else if ( avancer.encodeurGauche > avancer.encodeurDroite)
  {
    OutputLeft = 0;
    InputRight = avancer.encodeurDroite;
    SetpointRight = avancer.encodeurGauche;
    myPIDRight.compute();
    MOTOR_SetSpeed(Droite, avancer.valeurDroite + OutputRight);
  }
  else
  {
    MOTOR_SetSpeed(Gauche, avancer.valeurGauche);
    MOTOR_SetSpeed(Droite, avancer.valeurDroite);
  }
   if ( avancer.encodeurGauche > target || avancer.encodeurDroite> target )
  {
    if (avancer.encodeurGauche> target){
        MOTOR_SetSpeed(Gauche,0);
      if (avancer.encodeurDroite> target){
        MOTOR_SetSpeed(Droite,0);
        return;
       } 
    }
    else if (avancer.encodeurDroite> target){
        MOTOR_SetSpeed(Droite,0);

        if (avancer.encodeurGauche> target){
        MOTOR_SetSpeed(Gauche,0);
        return;
      } 
    }
  }
  }
  avancerReset();
}


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

void avancerUpdate()
{
  avancer.encodeurGauche = ENCODER_Read(Gauche);
  avancer.encodeurDroite = ENCODER_Read(Droite);

  if (avancer.encodeurGauche >= 30000 or avancer.encodeurDroite >= 30000)
  {
    avancer.posOverflow += ( avancer.encodeurGauche + avancer.encodeurDroite )/2;
    avancer.encodeurGauche = ENCODER_ReadReset(Gauche);
    avancer.encodeurDroite = ENCODER_ReadReset(Droite);
  }

  avancer.posInTurn = ( avancer.encodeurGauche + avancer.encodeurDroite ) /2 + avancer.posOverflow;
}

void avancerReset()
{
  avancer.isMoving = false;
  avancer.hasAccelerated = false;
  avancer.beginMillis = 0;
  avancer.valeurGauche = VITESSE_AVANCE_GAUCHE;
  avancer.valeurDroite = VITESSE_AVANCE_DROITE;
}