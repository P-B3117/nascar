#pragma once
//déclarer les fonctions ici

//define pour les couleurs
#include "sensor_couleur.h"


#define Gauche 0
#define Droite 1

//define pour tourner a gauche
#define VITESSE_TOURNEGAUCHE_GAUCHE -0.1
#define VITESSE_TOURNEGAUCHE_DROITE 0.1

//define pour tourner a droite
#define VITESSE_TOURNEDROITE_GAUCHE 0.1
#define VITESSE_TOURNEDROITE_DROITE -0.1

//define pour avancer
#define VITESSE_AVANCE_GAUCHE 0.3
#define VITESSE_AVANCE_DROITE 0.3
#define SPEEDLIMIT 0.7
#define SPEEDLIMITLENT 0.4

//define pour slowDown
#define TARGET_SLOW 6000

//define pour sifflet
#define micpin A7
#define moyenneSifflet 700

//Global pour avancer
 struct nice{
  bool isMoving = false;
  bool hasAccelerated = false;
  long encodeurGauche = 0;
  long encodeurDroite = 0;
  float beginMillis = 0;
  int posInTurn = 0;
  int posOverflow = 0;
  float valeurGauche = VITESSE_AVANCE_GAUCHE;
  float valeurDroite = VITESSE_AVANCE_DROITE;
};

float detection_distance_haut (void); //retourne la distance du capteur du haut

float detection_distance_bas (void); //retourne la distance du capteur du bas

void tournegauche(float valeurGauche, float valeurDroite ); //legacy, pas touche tourne de 90deg

void tournedroit(float valeurGauche,float valeurDroite); //legacy, pas touche tourne de 90deg

void tourne();

void avance(float speedLimit = SPEEDLIMIT); //avance continuellement

void slowDown(int target = TARGET_SLOW);  //ralenti

void avancerUpdate();

void avancerReset();

void create();

void couleurINIT();

int getCouleur();

bool detectionSifflet ();

bool bumper(int x);

void setTarget(float t, float deltat, float x, float y);

void computePIDLigneDroite(int targetDroit, int targetGauche, float pwrLimitDroit, float pwrLimitGauche);

void computePID(int targetDroit, int targetGauche, float pwrLimitDroit, float pwrLimitGauche);

void computePIDSuiveurMur(int targetDroit, int targetGauche, float pwrLimitDroit, float pwrLimitGauche);

void computePIDTourneDroite(int targetDroit, int targetGauche, float pwrLimitDroit, float pwrLimitGauche, int pointFinDroite, int pointFinGauche);

int suiveur_ligne();