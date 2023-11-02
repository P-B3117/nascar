#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define ROUGE 1
#define JAUNE 2
#define VERT 3
#define BLEU 4
#define BLANC 5

class tcs34725 {
public:
  tcs34725(void);

  boolean begin(void);
  int getData(void);
  void dataUpdate(void);

  boolean isAvailable, isSaturated;
  uint16_t againx, atime, atime_ms;
  uint16_t r, g, b, c;
  uint16_t ir; 
  uint16_t r_comp, g_comp, b_comp, c_comp;
  uint16_t saturation, saturation75;
  float cratio, cpl, ct, lux, maxlux;
  unsigned long beginmillis;
  bool isNew;
  int couleur;
  
private:
  struct tcs_agc {
    tcs34725Gain_t ag;
    tcs34725IntegrationTime_t at;
    uint16_t mincnt;
    uint16_t maxcnt;
  };
  static const tcs_agc agc_lst[];
  uint16_t agc_cur;

  void setGainTime(void);  
  Adafruit_TCS34725 tcs;    
};