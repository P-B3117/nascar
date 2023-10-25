#include "sensor_couleur.h"

// Autorange class for TCS34725

//
// Gain/time combinations to use and the min/max limits for hysteresis 
// that avoid saturation. They should be in order from dim to bright. 
//
// Also set the first min count and the last max count to 0 to indicate 
// the start and end of the list. 
//
const tcs34725::tcs_agc tcs34725::agc_lst[] = {
  { TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_700MS,     0, 47566 },
  { TCS34725_GAIN_16X, TCS34725_INTEGRATIONTIME_154MS,  3171, 63422 },
  { TCS34725_GAIN_4X,  TCS34725_INTEGRATIONTIME_154MS, 15855, 63422 },
  { TCS34725_GAIN_1X,  TCS34725_INTEGRATIONTIME_2_4MS,   248,     0 }
};
tcs34725::tcs34725() : agc_cur(0), isAvailable(0), isSaturated(0) {
}

// initialize the sensor
boolean tcs34725::begin(void) {
  tcs = Adafruit_TCS34725(agc_lst[agc_cur].at, agc_lst[agc_cur].ag);
  if ((isAvailable = tcs.begin())) 
    setGainTime();
  return(isAvailable);
}

// Set the gain and integration time
void tcs34725::setGainTime(void) {
  tcs.setGain(agc_lst[agc_cur].ag);
  tcs.setIntegrationTime(agc_lst[agc_cur].at);
  atime = int(agc_lst[agc_cur].at);
  atime_ms = ((256 - atime) * 2.4);  
  switch(agc_lst[agc_cur].ag) {
  case TCS34725_GAIN_1X: 
    againx = 1; 
    break;
  case TCS34725_GAIN_4X: 
    againx = 4; 
    break;
  case TCS34725_GAIN_16X: 
    againx = 16; 
    break;
  case TCS34725_GAIN_60X: 
    againx = 60; 
    break;
  }        
}

// Retrieve data from the sensor and do the calculations
int tcs34725::getData(void) {
  int couleur;
  // read the sensor and autorange if necessary
  tcs.getRawData(&r, &g, &b, &c);
  while(1) {
    if (agc_lst[agc_cur].maxcnt && c > agc_lst[agc_cur].maxcnt) 
      agc_cur++;
    else if (agc_lst[agc_cur].mincnt && c < agc_lst[agc_cur].mincnt)
      agc_cur--;
    else break;

    setGainTime(); 
    delay((256 - atime) * 2.4 * 2); // shock absorber
    tcs.getRawData(&r, &g, &b, &c);
    break;    
  }

  // DN40 calculations
 /* ir = (r + g + b > c) ? (r + g + b - c) / 2 : 0;
  r_comp = r - ir;
  g_comp = g - ir;
  b_comp = b - ir;
  c_comp = c - ir;   
  cratio = float(ir) / float(c);*/
 // if (c < 10000){
   // couleur = JAUNE;
 // }
  if (c > 10000){
  if (r > g && r > b){
    couleur=ROUGE;
    Serial.println("rouge");
      }
  if (b > g && b > r){
    couleur = BLEU;
    Serial.println("bleu");

  }
  if (g > b && g > r){
    couleur=VERT;
    Serial.println("vert");

  }
  
  }
  else if (c < 10000){
   if (b<1000){
    couleur=JAUNE;
    Serial.println("jaune");

   }
   else if (b>1000){
   couleur=BLANC;
  Serial.println("blanc");

   }
  }
  Serial.println(couleur);

  return couleur;
}