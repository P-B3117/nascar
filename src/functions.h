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
#define SPEEDLIMIT 0.8
#define SPEEDLIMITLENT 0.4

//define pour slowDown
#define TARGET_SLOW 4000

//define pour sifflet
#define micpin A7
#define moyenneSifflet 300

//Global pour avancer
 struct {
  bool isMoving = 0;
  bool hasAccelerated = 0;
  long encodeurGauche = 0;
  long encodeurDroite = 0;
  float beginMillis = 0;
  int posInTurn = 0;
  int posOverflow = 0;
  int valeurGauche = 0;
  int valeurDroite = 0;
} avancer;

float detection_distance_haut (void); //retourne la distance du capteur du haut

float detection_distance_bas (void); //retourne la distance du capteur du bas

void tournegauche(float valeurGauche, float valeurDroite ); //legacy, pas touche tourne de 90deg

void tournedroit(float valeurGauche,float valeurDroite); //legacy, pas touche tourne de 90deg

void avance(float valeurGauche = VITESSE_AVANCE_GAUCHE,float valeurDroite = VITESSE_AVANCE_DROITE, float speedLimit); //avance continuellement

void slowDown(int target = TARGET_SLOW);  //ralenti

void avancerUpdate();

void avancerReset();

void create();

void couleurINIT();

int getCouleur();

bool detectionSifflet ();