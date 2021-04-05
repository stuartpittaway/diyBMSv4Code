// Currentsensing Options
#define sensingNone     0
#define sensingA0       1
#define sensingINA228   2

// what Currentsensing method used?
#define CURRENTSENSING  sensingA0

// How often the sensing is mesaured
#define sensingIntervalInSeconds    5

// 20000mA / 512 ADC = 39.0625
#define mAPerADC                (float)(-39.0625F)

