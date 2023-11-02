#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"



//détecteur de distance
//define
#define distance_droite 3
#define distance_gauche 0
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

//variables globale PID
long prevT = 0;
float eprevDroite = 0;
float eintegralDroite = 0;
float eprevGauche = 0;
float eintegralGauche = 0;

//targets
float target_f[] = {0.0,0.0};
long target[] = {0,0};

float pwrLimit = 0.5;
//Variable global Sifflet
int mic ; //valeur de lecture du micro
int amplitude;
bool siffletActif = false;
unsigned long previousMillis = 0;
unsigned long interval = 1000;

//detecteur de couleur
 tcs34725 rgb_sensor;
 nice avancer;


// valeur suiveur ligne
#define AUCUN 0
#define GAUCHE 1
//#define CENTRE 2
#define DROITE 3
#define DROITE_SUIVEUR 4
#define CENTRE 5
#define GAUCHE_SUIVEUR 6
#define TOUT 7
#define MOTORGAUCHE 0
#define MOTORDROITE 1
 //structure pour avancer
 


float detection_distance_gauche (void){
    float tension_gauche=ROBUS_ReadIR(distance_gauche);
    float inverse_gauche=1/tension_gauche;
    float distance_left=inverse_gauche*6839.3-6.3;
    return distance_left;
}
float detection_distance_droite (void){
    float tension_droite=ROBUS_ReadIR(distance_droite);
    float inverse_droite=1/tension_droite;
    float distance_right=inverse_droite*6839.3-6.3;
    return distance_right;
}
float detecteur_distance_gauche (void){
    float tension_gauche=ROBUS_ReadIR(distance_gauche);
    float inverse_gauche=1/tension_gauche;
    float distance_left=inverse_gauche*6839.3-6.3;
    return distance_left;
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


void tourne()
{

  float speedLimit = 0.6;
  float ratioTour = 1.59;
  int beginMillis = millis();
  bool hasTurnedLeft = false;
  bool hasTurnedRight = false;
  int targetGauche = 12800;
  int targetDroite = 6400;
/*
  switch (getCouleur())
  {
  
  case VERT:
    ratioTour = 1.59;
    targetGauche = 12800;
    targetDroite = 6400;
    break;
  
  case JAUNE:
    ratioTour = 1.28;
    targetGauche = 19200;
    targetDroite = 12800
    break;
  
  }*/

   avancer.encodeurGauche = ENCODER_ReadReset(Gauche);
   avancer.encodeurDroite = ENCODER_ReadReset(Droite);
   avancer.posOverflow += ( avancer.encodeurGauche + avancer.encodeurDroite )/2;
   avancer.encodeurGauche = ENCODER_Read(Gauche);
   avancer.encodeurDroite = ENCODER_Read(Droite);

  while (!hasTurnedRight or !hasTurnedLeft)
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

if (avancer.encodeurGauche > targetGauche) { MOTOR_SetSpeed(Gauche,0); hasTurnedLeft = true; }
if (avancer.encodeurDroite > targetDroite) { MOTOR_SetSpeed(Droite,0); hasTurnedRight = true; }

}

 avancerReset();
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
  amplitude = abs(moyenneSifflet - mic); 
  //Serial.println(amplitude);

  if(analogRead(A13)>600){
    
      siffletActif = true;
      
    }

  else{
    siffletActif = false;
  }
  //Serial.println(siffletActif);

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


void computePID(int targetDroit, int targetGauche, float pwrLimitDroit, float pwrLimitGauche){
    
  // PID constants Droite
  float kpDroite = 0.000999;
  float kdDroite = 0.000000001;
  float kiDroite = 0.0;

  float kpGauche = 0.000999;
  float kdGauche = 0.000000001;
  float kiGauche = 0.0;

  //time difference
  long currT = micros();

  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT; 

  // set target position
  setTarget(currT/1.0e6,deltaT, targetDroit, targetGauche);

  int posDroite = ENCODER_Read(Droite);
  int posGauche = ENCODER_Read(Gauche);

  // erreur
  int eDroite = target[0] - posDroite;
  int eGauche = target[1] - posGauche;

  //derive
  float dedtDroite = (eDroite-eprevDroite)/(deltaT);
  float dedtGauche = (eGauche-eprevGauche)/(deltaT);

  //integral
  eintegralDroite = eintegralDroite + eDroite*deltaT;
  eintegralGauche = eintegralGauche + eGauche*deltaT;

  //control signal
  float uDroite = kpDroite*eDroite + kdDroite*dedtDroite + kiDroite*eintegralDroite;
  float uGauche = kpGauche*eGauche + kdGauche*dedtGauche + kiGauche*eintegralGauche;

  // moteur power
  float pwrDroite = fabs(uDroite);
  float pwrGauche = fabs(uGauche);
  if(pwrDroite>pwrLimitDroit){

    pwrDroite=pwrLimitDroit;
    if(targetDroit<0){
      pwrDroite = pwrDroite*-1;
    }
  }
  if(pwrGauche>pwrLimitGauche){

    pwrGauche=pwrLimitGauche;
    if(targetGauche<0){
      pwrGauche = pwrGauche*-1;
    }
  }

  //Signal to motor
  //il faut ajouter une -Valeur a la condition pour arriver au valeurs pile que nous voulons
  if(posDroite>=targetDroit){
    MOTOR_SetSpeed(Droite,0);
  }
  else{
    MOTOR_SetSpeed(Droite,pwrDroite);
  }
  if(posGauche>=targetGauche){
    MOTOR_SetSpeed(Gauche,0);
  }
  else{
    MOTOR_SetSpeed(Gauche,pwrGauche);
  }
  //store previous error
  eprevDroite = eDroite;
  eprevGauche = eGauche;
}


void computePIDLigneDroite(int targetDroit, int targetGauche, float pwrLimitDroit, float pwrLimitGauche){

  
  // PID constants Droite
  float kpDroite = 0.000999;
  float kdDroite = 0.000000001;
  float kiDroite = 0.0;

  float kpGauche = 0.000999;
  float kdGauche = 0.000000001;
  float kiGauche = 0.0;

  //time difference
  long currT = micros();

  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT; 

  // set target position
  setTarget(currT/1.0e6,deltaT, targetDroit, targetGauche);

  int posDroiteCourante = ENCODER_Read(Droite);
  int posGaucheCourante = ENCODER_Read(Gauche);
  int posDroiteAncienne;
  int posGaucheAncienne;

  // erreur
  int eDroite = target[0] - posDroiteCourante;
  int eGauche = target[1] - posGaucheCourante;

  //derive
  float dedtDroite = (eDroite-eprevDroite)/(deltaT);
  float dedtGauche = (eGauche-eprevGauche)/(deltaT);

  //integral
  eintegralDroite = eintegralDroite + eDroite*deltaT;
  eintegralGauche = eintegralGauche + eGauche*deltaT;

  //control signal
  float uDroite = kpDroite*eDroite + kdDroite*dedtDroite + kiDroite*eintegralDroite;
  float uGauche = kpGauche*eGauche + kdGauche*dedtGauche + kiGauche*eintegralGauche;

  // moteur power
  float pwrDroite = fabs(uDroite);
  float pwrGauche = fabs(uGauche);
  if(pwrDroite>pwrLimitDroit){

    pwrDroite=pwrLimitDroit;
    if(targetDroit<0){
      pwrDroite = pwrDroite*-1;
    }
  }
  if(pwrGauche>pwrLimitGauche){

    pwrGauche=pwrLimitGauche;
    if(targetGauche<0){
      pwrGauche = pwrGauche*-1;
    }
  }

  //Signal to motor
  //il faut ajouter une -Valeur a la condition pour arriver au valeurs pile que nous voulons
  if(posDroiteCourante>=targetDroit){
    posDroiteAncienne = posDroiteCourante-targetDroit;
    MOTOR_SetSpeed(Droite,pwrDroite);
  }
  else{
    MOTOR_SetSpeed(Droite,pwrDroite);
    posGaucheAncienne = posGaucheCourante-targetGauche;
  }
  if(posGaucheCourante>=targetGauche){
    posGaucheAncienne = posGaucheCourante-targetGauche;
    MOTOR_SetSpeed(Gauche,pwrGauche);
  }
  else{
    MOTOR_SetSpeed(Gauche,pwrGauche);
  }
  //store previous error
  eprevDroite = eDroite;
  eprevGauche = eGauche;
}
void computePIDTourneDroite(int targetDroit, int targetGauche, float pwrLimitDroit, float pwrLimitGauche, int pointFinDroite, int pointFinGauche){

  
  // PID constants Droite
  float kpDroite = 0.000999;
  float kdDroite = 0.000000001;
  float kiDroite = 0.0;

  float kpGauche = 0.000999;
  float kdGauche = 0.000000001;
  float kiGauche = 0.0;

  //time difference
  long currT = micros();

  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT; 

  // set target position
  setTarget(currT/1.0e6,deltaT, targetDroit, targetGauche);

  int posDroiteCourante = ENCODER_Read(Droite);
  int posGaucheCourante = ENCODER_Read(Gauche);
  int posDroiteAncienne;
  int posGaucheAncienne;

  // erreur
  int eDroite = target[0] - posDroiteCourante;
  int eGauche = target[1] - posGaucheCourante;

  //derive
  float dedtDroite = (eDroite-eprevDroite)/(deltaT);
  float dedtGauche = (eGauche-eprevGauche)/(deltaT);

  //integral
  eintegralDroite = eintegralDroite + eDroite*deltaT;
  eintegralGauche = eintegralGauche + eGauche*deltaT;

  //control signal
  float uDroite = kpDroite*eDroite + kdDroite*dedtDroite + kiDroite*eintegralDroite;
  float uGauche = kpGauche*eGauche + kdGauche*dedtGauche + kiGauche*eintegralGauche;

  // moteur power
  float pwrDroite = fabs(uDroite);
  float pwrGauche = fabs(uGauche);
  if(pwrDroite>pwrLimitDroit){

    pwrDroite=pwrLimitDroit;
    if(targetDroit<0){
      pwrDroite = pwrDroite*-1;
    }
  }
  if(pwrGauche>pwrLimitGauche){

    pwrGauche=pwrLimitGauche;
    if(targetGauche<0){
      pwrGauche = pwrGauche*-1;
    }
  }

  //Signal to motor
  //il faut ajouter une -Valeur a la condition pour arriver au valeurs pile que nous voulons
  if(posDroiteCourante>=targetDroit){
    posDroiteAncienne = posDroiteCourante-targetDroit;
    MOTOR_SetSpeed(Droite,pwrDroite);
  }
  else{
    MOTOR_SetSpeed(Droite,pwrDroite);
    posGaucheAncienne = posGaucheCourante-targetGauche;
  }
  if(posGaucheCourante>=targetGauche){
    posGaucheAncienne = posGaucheCourante-targetGauche;
    MOTOR_SetSpeed(Gauche,pwrGauche);
  }
  else{
    MOTOR_SetSpeed(Gauche,pwrGauche);
  }

  if(posDroiteCourante>=pointFinDroite){

    MOTOR_SetSpeed(Droite, 0);
  }

  if(posGaucheCourante>=pointFinGauche){

    MOTOR_SetSpeed(Gauche, 0);
  }
  //store previous error
  eprevDroite = eDroite;
  eprevGauche = eGauche;
}


void computePIDSuiveurMur(int targetDroit, int targetGauche, float pwrLimitDroit, float pwrLimitGauche, float distanceMur, float distanceCible){

  
  // PID constants Droite
  float kpDroite = 0.000999;
  float kdDroite = 0.000000001;
  float kiDroite = 0.0;

  float kpGauche = 0.000999;
  float kdGauche = 0.000000001;
  float kiGauche = 0.0;

  //time difference
  long currT = micros();

  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT; 

  // set target position
  setTarget(currT/1.0e6,deltaT, targetDroit, targetGauche);

  int posDroite = ENCODER_Read(Droite);
  int posGauche = ENCODER_Read(Gauche);

  // erreur
  int eDroite = target[0] - posDroite;
  int eGauche = target[1] - posGauche;

  //derive
  float dedtDroite = (eDroite-eprevDroite)/(deltaT);
  float dedtGauche = (eGauche-eprevGauche)/(deltaT);

  //integral
  eintegralDroite = eintegralDroite + eDroite*deltaT;
  eintegralGauche = eintegralGauche + eGauche*deltaT;

  //control signal
  float uDroite = kpDroite*eDroite + kdDroite*dedtDroite + kiDroite*eintegralDroite;
  float uGauche = kpGauche*eGauche + kdGauche*dedtGauche + kiGauche*eintegralGauche;

  // moteur power
  float pwrDroite = fabs(uDroite);
  float pwrGauche = fabs(uGauche);
  if(pwrDroite>pwrLimitDroit){

    pwrDroite=pwrLimitDroit;
    if(targetDroit<0){
      pwrDroite = pwrDroite*-1;
    }
  }
  if(pwrGauche>pwrLimitGauche){

    pwrGauche=pwrLimitGauche;
    if(targetGauche<0){
      pwrGauche = pwrGauche*-1;
    }
  }

  //Signal to motor
  //il faut ajouter une -Valeur a la condition pour arriver au valeurs pile que nous voulons
  if(posDroite>=targetDroit){
    MOTOR_SetSpeed(Droite,pwrDroite);
  }
  else{
    MOTOR_SetSpeed(Droite,pwrDroite);
  }
  if(posGauche>=targetGauche){
    MOTOR_SetSpeed(Gauche,pwrDroite);
  }
  else{
    MOTOR_SetSpeed(Gauche,pwrGauche);
  }
  //store previous error
  eprevDroite = eDroite;
  eprevGauche = eGauche;
}


void setTarget(float t, float deltat, float x, float y){

  float positionChange[2] = {x,y};// ici on lui donne ces targets
  for(int k = 0; k<2; k++){

    target_f[k] = target_f[k]+positionChange[k];
  }
  target[0] = (long) target_f[0];
  target[1] = (long) target_f[1];
}



int detecteur_ligne()
{
    int option;
    float tension_suiveur=analogRead(A12)*5.0/1023.0;
    Serial.println(tension_suiveur);
    if(tension_suiveur<0.3){
        option=AUCUN;
        Serial.println("aucun   ");
        Serial.println(option);

    }
    /*else if(tension_suiveur > 2.3 && tension_suiveur<3){
        option=GAUCHE;
        Serial.println("gauche   ");
        Serial.println(option);
    }*/
    /*else if(tension_suiveur>1 && tension_suiveur<1.5){
         int option=CENTRE;
        Serial.println("centre   ");
        Serial.println(option);
    }
    /*else if (tension_suiveur>0.3 && tension_suiveur<1){
        option=DROITE;
        Serial.println("droite   ");
        Serial.println(option);
    }*/
    else if(tension_suiveur>3.8 && tension_suiveur<4.8){
        option=DROITE_SUIVEUR;
        Serial.println("gauche et centre   ");
        Serial.println(option);
    }
    else if (tension_suiveur>3 && tension_suiveur<3.8){
        option=CENTRE;
        Serial.println("gauche et droite   ");
        Serial.println(option);
    }
    else if (tension_suiveur>1.5 && tension_suiveur<2.3){
        option=GAUCHE_SUIVEUR;
        Serial.println("centre et droite  ");
        Serial.println(option);
    }
    else if(tension_suiveur>4.8){
        option=TOUT;
        Serial.println("tout   ");
        Serial.println(option);
        
    }
    return option;
}
bool extreme_gauche(){
    float tension_extreme_gauche=analogRead(A10);
    //Serial.println(tension_extreme_gauche);
    if(tension_extreme_gauche>400){
        return true;
    }
        else {
            return false;
        }
    
}
bool extreme_droite(){
    float tension_extreme_droite=analogRead(A11);
    Serial.println(tension_extreme_droite);
    if(tension_extreme_droite >70){
        return true;
    }
    else{
        return false;
    }
}

void suiveur_ligne(float power){
    MOTOR_SetSpeed(RIGHT,power);
    MOTOR_SetSpeed(LEFT,power);

    if (extreme_droite()==true)  {
        MOTOR_SetSpeed(MOTORGAUCHE,power/3);
        MOTOR_SetSpeed(MOTORDROITE,-power/3);
        while( extreme_droite()==true){
          Serial.println("extreme droite");
        }
    } 
     else if (extreme_gauche()==true)  {
        MOTOR_SetSpeed(MOTORGAUCHE,-power/3);
        MOTOR_SetSpeed(MOTORDROITE,power/3);
        while( extreme_gauche()==true){
          Serial.println("extreme gauche");
        }
    } 
    else if(detecteur_ligne()==DROITE_SUIVEUR){
        MOTOR_SetSpeed(MOTORGAUCHE,power);
        MOTOR_SetSpeed(MOTORDROITE,0);
        while (detecteur_ligne()==DROITE_SUIVEUR){
        Serial.println("vers la droite");
        }
    }
    else if(detecteur_ligne()==GAUCHE_SUIVEUR){
        MOTOR_SetSpeed(MOTORGAUCHE,0);
        MOTOR_SetSpeed(MOTORDROITE,power);
        while(detecteur_ligne()==GAUCHE_SUIVEUR){
        Serial.println("vers la gauche");
        }
    }  
    else if(detecteur_ligne()==CENTRE){
        MOTOR_SetSpeed(MOTORGAUCHE,power);
        MOTOR_SetSpeed(MOTORDROITE,power);
        Serial.println("tout dorit");
    }
   
}

void suiveur_mur_droit(float power){
    if (detection_distance_droite()<=5.5){//tres proche
        MOTOR_SetSpeed(RIGHT,power+(power/1.5));
        MOTOR_SetSpeed(LEFT,power/3);
    }
    else if (detection_distance_droite()>5.5 && detection_distance_droite()<7){//centre
        MOTOR_SetSpeed(RIGHT,power);
        MOTOR_SetSpeed(LEFT,power);

    }
    else if (detection_distance_droite()>=7 && detection_distance_droite()<11){//loin
        MOTOR_SetSpeed(RIGHT,power);
        MOTOR_SetSpeed(LEFT,power+power/3);
    }
    
    else if (detection_distance_droite()>=11 && detection_distance_droite()<40){//tres loin
        MOTOR_SetSpeed(RIGHT,power);
        MOTOR_SetSpeed(LEFT,power+power/1.5);
    }
    else if(detection_distance_droite()>120)
    {
       
        MOTOR_SetSpeed(RIGHT,0);
        MOTOR_SetSpeed(LEFT,power);        
    }
}


void suiveur_mur_gauche(float power){
    if (detection_distance_gauche()<=5.5){//tres proche
        MOTOR_SetSpeed(LEFT,power+(power/1.5));
        MOTOR_SetSpeed(RIGHT,power/3);
    }
    else if (detecteur_distance_gauche()>5.5 && detecteur_distance_gauche()<7){//centre
        MOTOR_SetSpeed(LEFT,power);
        MOTOR_SetSpeed(RIGHT,power);

    }
    else if (detecteur_distance_gauche()>=7 && detecteur_distance_gauche()<11){//loin
        MOTOR_SetSpeed(LEFT,power);
        MOTOR_SetSpeed(RIGHT,power+power/3);
    }
    
    else if (detecteur_distance_gauche()>=11 && detecteur_distance_gauche()<40){//tres loin
        MOTOR_SetSpeed(LEFT,power);
        MOTOR_SetSpeed(RIGHT,power+power/1.5);
    }
    else if(detecteur_distance_gauche()>120)
    {
       
        MOTOR_SetSpeed(LEFT,0);
        MOTOR_SetSpeed(RIGHT,power);        
    }
}





//fonction.cpp

int detection(int orientation)
{
    bool valueR = 0;
    bool valueV = 0;
    
    if (orientation == 0) {
    valueR = !digitalRead(38);//valeur du voltage de la DEL rouge
    valueV = !digitalRead(39);//valeur du voltage de la DEL vert
    }
    else if (orientation == 1) {
    valueR = !digitalRead(40);//valeur du voltage de la DEL rouge
    valueV = !digitalRead(41);//valeur du voltage de la DEL vert
    }
    else if (orientation == 2) {
    valueR = !digitalRead(42);//valeur du voltage de la DEL rouge
    valueV = !digitalRead(43);//valeur du voltage de la DEL vert
    }

  if (valueR && valueV)
  {
    return PROXTOUT;
  }
  else if (valueV) {
    return PROXVERT;
  }
  else if (valueR) {
    return PROXROUGE;
  }
  else {
    return PROXRIEN;
  }
}

//gauche rouge: 38 vert : 39
//droite rouge: 40 vert : 41
//devant rouge: 42 vert : 43