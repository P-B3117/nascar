#pragma once
#define AUCUN 0
#define GAUCHE 1
#define CENTRE 2
#define DROITE 3
#define GAUCHEETCENTRE 4
#define GAUCHEETDROITE 5
#define CENTREETDROITE 6
#define TOUT 7
#define MOTORGAUCHE 0
#define MOTORDROITE 1
int detecteur_ligne();
void suiveur_ligne(float power);
void suiveur_ligne_blanc(float power);
bool extreme_droite();
bool extreme_gauche();
