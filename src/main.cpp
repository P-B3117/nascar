#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"
#include <stdio.h>

//variable globale couleur
int couleurInitiale = 0;


//variable globale encodeur

long prevT = 0;
float eprevDroite = 0;
float eintegralDroite = 0;
float eprevGauche = 0;
float eintegralGauche = 0;

float pwrLimit = 0.5;

//targets
float target_f[] = {0.0,0.0};
long target[] = {0,0};


//etat
int etat = 1;

void setup(){
  BoardInit();
  Serial.begin(9600);
  couleurINIT();
}


void loop(){
  
  switch(couleurInitiale){

    case VERT:
      switch(etat){
        
        case 1:
        computePID(20000,20000,pwrLimit,pwrLimit);//position (droite,gauche,pwrLimitDroit,pwrLimitGauche)
  
        Serial.print(ENCODER_Read(Droite));
        Serial.print("      ");
        Serial.println(ENCODER_Read(Gauche));
        
        
        break;

      }
    break;


    case JAUNE:

    break;
    

  }




}

void setTarget(float t, float deltat, float x, float y){

  float positionChange[2] = {x,y};// ici on lui donne ces targets
  for(int k = 0; k<2; k++){

    target_f[k] = target_f[k]+positionChange[k];
  }
  target[0] = (long) target_f[0];
  target[1] = (long) target_f[1];
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
