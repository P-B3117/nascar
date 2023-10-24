#include "functions.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "ArduPID.h"

//définir les fonctions ici

void servomotor(int servo,float angle){
   SERVO_SetAngle(servo,angle);
}