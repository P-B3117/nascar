#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"

//définir les fonctions ici
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
int detecteur_ligne()
{
    int option;
    float tension_suiveur=analogRead(A5)*5.0/1023.0;
    Serial.println(tension_suiveur);
    if(tension_suiveur<0.3){
        option=AUCUN;
        Serial.println("aucun   ");
        Serial.println(option);
        return option;

    }
    else if(tension_suiveur > 2.3 && tension_suiveur<3){
        option=GAUCHE;
        Serial.println("gauche   ");
        Serial.println(option);
    }
    else if(tension_suiveur>1 && tension_suiveur<1.5){
         int option=CENTRE;
        Serial.println("centre   ");
        Serial.println(option);
    }
    else if (tension_suiveur>0.3 && tension_suiveur<1){
        option=DROITE;
        Serial.println("droite   ");
        Serial.println(option);
    }
    else if(tension_suiveur>3.5 && tension_suiveur<4.3){
        option=GAUCHEETCENTRE;
        Serial.println("gauche et centre   ");
        Serial.println(option);
    }
    else if (tension_suiveur>3 && tension_suiveur<3.5){
        option=GAUCHEETDROITE;
        Serial.println("gauche et droite   ");
        Serial.println(option);
    }
    else if (tension_suiveur>1.5 && tension_suiveur<2.3){
        option=CENTREETDROITE;
        Serial.println("centre et droite  ");
        Serial.println(option);
    }
    else if(tension_suiveur>4.3){
        option=TOUT;
        Serial.println("tout   ");
        Serial.println(option);
        
    }
    return option;
}
bool extreme_droite(){
    float tension_extreme_gauche=analogRead(A2)*5.0/1023.2;
    Serial.println(tension_extreme_gauche);
    if(tension_extreme_gauche>2.0){
        return false;
    }
        else {
            return true;
        }
    
}
bool extreme_gauche(){
    float tension_extreme_droite=analogRead(A8)*5.0/1023.0;
    Serial.println(tension_extreme_droite);
    if(tension_extreme_droite>2.0){
        return false;
    }
    else{
        return true;
    }


}



void suiveur_ligne(float power){
    MOTOR_SetSpeed(RIGHT,power);
    MOTOR_SetSpeed(LEFT,power);

   
    if(detecteur_ligne()==GAUCHEETDROITE){
        MOTOR_SetSpeed(MOTORGAUCHE,power);
        MOTOR_SetSpeed(MOTORDROITE,power);
        Serial.println("tout dorit");
    }
    else if(detecteur_ligne()==GAUCHEETCENTRE){
        MOTOR_SetSpeed(MOTORGAUCHE,power/2);
        MOTOR_SetSpeed(MOTORDROITE,power/5);
        while (detecteur_ligne()!=GAUCHEETDROITE && extreme_gauche()!=false){
        Serial.println("vers la droite");
        }
    }
    else if(detecteur_ligne()==CENTREETDROITE){
        MOTOR_SetSpeed(MOTORGAUCHE,power/5);
        MOTOR_SetSpeed(MOTORDROITE,power/2);
        while(detecteur_ligne()!=GAUCHEETDROITE && extreme_droite()!=false){
        Serial.println("vers la gauche");
        }
    }  
    else if (extreme_droite()==false)  {
        MOTOR_SetSpeed(MOTORGAUCHE,power/2);
        MOTOR_SetSpeed(MOTORDROITE,0);
        while( detecteur_ligne()!=GAUCHEETDROITE){}
    } 
     else if (extreme_gauche()==false)  {
        MOTOR_SetSpeed(MOTORGAUCHE,0);
        MOTOR_SetSpeed(MOTORDROITE,power/2);
        while( detecteur_ligne()!=GAUCHEETDROITE){}
    } 
}

void suiveur_ligne_blanc(float power){
    if (detecteur_ligne()==GAUCHEETCENTRE){
        MOTOR_SetSpeed(LEFT,power);
        MOTOR_SetSpeed(RIGHT,power*1.5);
        while (detecteur_ligne()==DROITE) {}
    }
    else if (detecteur_ligne()==CENTREETDROITE){
        MOTOR_SetSpeed(LEFT,power*1.5);
        MOTOR_SetSpeed(RIGHT,power);   
        while (detecteur_ligne()==CENTREETDROITE){}
    }
    else{
        MOTOR_SetSpeed(LEFT,power);
        MOTOR_SetSpeed(RIGHT,power);
    }
    
}