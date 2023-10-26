#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"



//détecteur de distance
//define
#define BAS 3
#define HAUT 2

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
 nice avancer;

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
    ENCODER_Reset(0);
    ENCODER_Reset(1);


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

    ENCODER_Reset(0);
    ENCODER_Reset(1);
 
    MOTOR_SetSpeed(Gauche,valeurGauche);
    MOTOR_SetSpeed(Droite,valeurDroite);

  int encodeurGauche = ENCODER_Read(Gauche);
  int encodeurDroite = ENCODER_Read(Droite);
  Serial.println(ENCODER_Read(Gauche));
  Serial.println(ENCODER_Read(Droite));
  
  if (encodeurGauche>1942 || encodeurDroite<-1942 )
  {
    if (encodeurGauche>1942){
        MOTOR_SetSpeed(Gauche,0);
        Serial.println("je tourne1");
      if (encodeurDroite<-1942){
        MOTOR_SetSpeed(Droite,0);
        Serial.println("je tourne2");
        return;
       } 
    }
    else if (encodeurDroite<-1942){
        MOTOR_SetSpeed(Droite,0);
        Serial.println("je tourne3");

        if (encodeurGauche>1942){
        MOTOR_SetSpeed(Gauche,0);
        Serial.println("je tourne4");
        return;
       } 
  }


 }
  Serial.println("je tourne5");
}

/*
struct nice{
  bool isMoving = false;
  bool hasAccelerated = false;
  long encodeurGauche = 0;
  long encodeurDroite = 0;
  float beginMillis = 0;
  int posInTurn = 0;
  int posOverflow = 0;
  int valeurGauche = VITESSE_AVANCE_GAUCHE;
  int valeurDroite = VITESSE_AVANCE_DROITE;
} avancer;
*/

void avance(float speedLimit = SPEEDLIMIT)
{
  
 int beginMillis = millis();

  if (avancer.isMoving == false)
  {
  avancer.isMoving = true;
  avancer.hasAccelerated = false;   
  avancer.beginMillis = millis();
  }

  avancerUpdate();

  if (avancer.valeurGauche < speedLimit && !avancer.hasAccelerated) //acceleration
  {
    avancer.valeurGauche += (millis() - avancer.beginMillis)/10000.0;
    avancer.valeurDroite += (millis() - avancer.beginMillis)/10000.0;
  }
  else if (avancer.valeurGauche >= speedLimit) {
    avancer.hasAccelerated = 1;
  }

  MOTOR_SetSpeed(Gauche, avancer.valeurGauche);
  MOTOR_SetSpeed(Droite, avancer.valeurDroite);

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

while ( millis() < beginMillis + 100 ) {}
 
}


bool tourne()
{

  float speedLimit = 0.6;
  float ratioTour = 1.59;
  int beginMillis = millis();
  bool hasTurned = false;
/*
  switch (getCouleur())
  {
  
  case VERT:
    ratioTour = 1.59;
    break;
  
  case JAUNE:
    ratioTour = 1.28;
    break;
  
  }*/

   avancer.encodeurGauche = ENCODER_ReadReset(Gauche);
   avancer.encodeurDroite = ENCODER_ReadReset(Droite);
   avancer.posOverflow += ( avancer.encodeurGauche + avancer.encodeurDroite )/2;
   avancer.encodeurGauche = ENCODER_Read(Gauche);
   avancer.encodeurDroite = ENCODER_Read(Droite);

  while (!hasTurned)
  {

  if (avancer.isMoving == false)
  {
  avancer.isMoving = true;    
  avancer.beginMillis = millis();
  }

  avancerUpdate();

  avancer.encodeurGauche = ENCODER_Read(Gauche);
  avancer.encodeurDroite = ENCODER_Read(Droite);

  if (avancer.valeurGauche * ratioTour < speedLimit && !avancer.hasAccelerated) //acceleration
  {
    avancer.valeurGauche += (millis() - avancer.beginMillis)/10000.0;
    avancer.valeurDroite += (millis() - avancer.beginMillis)/10000.0;
  }
  else if (avancer.valeurGauche >= speedLimit) {
    avancer.hasAccelerated = 1;
  }

  
  MOTOR_SetSpeed(Gauche, avancer.valeurGauche * ratioTour);
  MOTOR_SetSpeed(Droite, avancer.valeurDroite);

  //ajustement
  if ( avancer.encodeurGauche / ratioTour < avancer.encodeurDroite)
  {
    OutputRight = 0;
    InputLeft = avancer.encodeurGauche;
    SetpointLeft = avancer.encodeurDroite;
    myPIDLeft.compute();
    MOTOR_SetSpeed(Gauche, avancer.valeurGauche * ratioTour + OutputLeft );
  }

  else if ( avancer.encodeurGauche / ratioTour > avancer.encodeurDroite)
  {
    OutputLeft = 0;
    InputRight = avancer.encodeurDroite;
    SetpointRight = avancer.encodeurGauche;
    myPIDRight.compute();
    MOTOR_SetSpeed(Droite, avancer.valeurDroite + OutputRight);
  }
  else
  {
    MOTOR_SetSpeed(Gauche, avancer.valeurGauche * ratioTour);
    MOTOR_SetSpeed(Droite, avancer.valeurDroite);
  }

  while ( millis() < beginMillis + 100 ) {}


 Serial.print(avancer.encodeurGauche);
 Serial.print("     ");
 Serial.print(OutputLeft);
 Serial.print("     ");
 Serial.print(avancer.encodeurDroite);
 Serial.print("     ");
 Serial.print(OutputRight);
 Serial.print("     ");
 Serial.print(avancer.valeurGauche * ratioTour);
 Serial.print("     ");
 Serial.print(avancer.valeurDroite);
 Serial.print("     ");
 Serial.print((avancer.encodeurGauche/10 + avancer.encodeurDroite/10) / 2.0);
 Serial.println();

if (true) hasTurned = true;

}

 avancerReset();

 return hasTurned;
}

void slowDown(int target = TARGET_SLOW)
 {

  int beginMillis = millis();

  myPIDLeft.reset();
  myPIDRight.reset();

   avancer.encodeurGauche = ENCODER_ReadReset(Gauche);
   avancer.encodeurDroite = ENCODER_ReadReset(Droite);
   avancer.posOverflow += ( avancer.encodeurGauche + avancer.encodeurDroite )/2;
   avancer.encodeurGauche = ENCODER_Read(Gauche);
   avancer.encodeurDroite = ENCODER_Read(Droite);

  while (avancer.encodeurDroite < target or  avancer.encodeurGauche < target)
  {

    
   avancer.encodeurGauche = ENCODER_Read(Gauche);
   avancer.encodeurDroite = ENCODER_Read(Droite);

   
  if ( target/10 - ( (avancer.encodeurGauche/10 + avancer.encodeurDroite/10) / 2.0)  <= TARGET_SLOW/10  && avancer.valeurGauche > VITESSE_AVANCE_GAUCHE)
  {
    float decel = ( ( -( (long)millis() - beginMillis ) ) / 10000.0 ) + 0.1;
    avancer.valeurGauche -= decel;
    avancer.valeurDroite -= decel;
  }
/*
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
  */

  MOTOR_SetSpeed(Gauche, avancer.valeurGauche);
  MOTOR_SetSpeed(Droite, avancer.valeurDroite);

  if (avancer.encodeurGauche > target) { MOTOR_SetSpeed(Gauche,0); }
  if (avancer.encodeurDroite > target) { MOTOR_SetSpeed(Droite,0); }
  
  
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
 Serial.print(avancer.valeurDroite);
 Serial.print("     ");
 Serial.print((avancer.encodeurGauche/10 + avancer.encodeurDroite/10) / 2.0);
 Serial.println();
 delay(100);
  }

  avancerUpdate();
  avancerReset();
}


bool detectionSifflet (){

  unsigned long currentMillis = millis();
  mic = analogRead(micpin);
  amplitude = abs(moyenneSifflet - mic); 
  //Serial.println(amplitude);

  if(amplitude>300){
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
    avancer.encodeurGauche = ENCODER_ReadReset(Gauche);
    avancer.encodeurDroite = ENCODER_ReadReset(Droite);
    avancer.posOverflow += ( avancer.encodeurGauche + avancer.encodeurDroite )/2;
    avancer.encodeurGauche = ENCODER_Read(Gauche);
    avancer.encodeurDroite = ENCODER_Read(Droite);
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
  myPIDLeft.reset();
  myPIDRight.reset();
}
bool bumper(int x){
  if (ROBUS_IsBumper(x)==true){
    return true;
  }
  else{
    return false;
  }
}