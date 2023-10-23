#include "functions.h"


  tcs34725 rgb_sensor;

int couleurINIT() {

  rgb_sensor.begin();
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW); // @gremlins Bright light, bright light!

}

int getCouleur() {
    return rgb_sensor.getData();
}