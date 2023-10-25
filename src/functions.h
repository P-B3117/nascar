#pragma once
//déclarer les fonctions ici

#define Gauche 0
#define Droite 1

//define pour tourner a gauche
#define VITESSE_TOURNEGAUCHE_GAUCHE -0.1
#define VITESSE_TOURNEGAUCHE_DROITE 0.1

//define pour tourner a droite
#define VITESSE_TOURNEDROITE_GAUCHE 0.1
#define VITESSE_TOURNEDROITE_DROITE -0.1

//define pour avancer
#define TARGET_POSITION 30000
#define VITESSE_AVANCE_GAUCHE 0.3
#define VITESSE_AVANCE_DROITE 0.3

//define pour sifflet
#define micpin A7
#define moyenneSifflet 300



void tournegauche(float valeurGauche, float valeurDroite );

void tournedroit(float valeurGauche,float valeurDroite);

void avance(float valeurGauche = VITESSE_AVANCE_GAUCHE,float valeurDroite = VITESSE_AVANCE_DROITE);

void create();

void couleurINIT();

int getCouleur();

bool detectionSifflet ();