#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"
#include "functions.h"
#include <stdio.h>

//variable globale couleur
int couleurInitiale = 0;


//variable globale encodeur
int posDroite=0;
int posGauche=0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;


void setup(){
  BoardInit();
  Serial.begin(9600);
  couleurINIT();
}


void loop(){
  // set target position
  int target = 3200;
  
  // PID constants Droite
  float kpDroite = 0.01;
  float kdDroite = 0;
  float kiDroite = 0;

  //time difference
  long currT = micros();

  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT; 

  // erreur
  int e = target - ENCODER_Read(Droite);

  //derive
  float dedt = (e-eprev)/(deltaT);

  //integral
  eintegral = eintegral + e*deltaT ;

  //control signal
  float u = kpDroite*e + kdDroite*dedt + kiDroite*eintegral;

  // moteur power
  float pwr = fabs(u);
  if(pwr>1){

    pwr=1;
  }

  //Signal to motor
  MOTOR_SetSpeed(Droite,u);

  //store previous error
  eprev = e;

  //Serial.println(target);
  Serial.println(posDroite);

}
