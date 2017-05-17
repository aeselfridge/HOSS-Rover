/*************************************
* Author:        Alec Selfridge
* Filename:      SHARP_IR.h
* Date Created:  11/03/2016
* Last Modified: 02/15/2017
* Device:        SHARP GP2Y0A41SK0F
                 LPC1768 (mbed LPC1768)
**************************************/
#ifndef SHARP_IR_H
#define SHARP_IR_H 

// units
#define METRIC   0
#define IMPERIAL 1
// polynomial coefficients
#define a0       30.24776
#define a1      -57.00180
#define a2       49.66785
#define a3      -21.14833
#define a4        3.51488
// misc
#define ADCRES 4096.0
#define ADCV      3.3
#define CMTOIN    2.54
#define NUM_AVG  30

class SHARP_IR
{
  private:
    double         avgDist;
    unsigned short ADCchannel;     // which ADC to use (0-5)
    unsigned short rawReading; 
    float          objectDistance; 
    float          alertDistance;
    bool           unit;           // 0 = metric (cm), 1 = imperial (in) (metric default)
  
    void  configADC();
    void  readADC();
    void  fitData();
  
  public:
    SHARP_IR(unsigned short channel);
    void  init();
    void  init(float AD);
    void  update();
    void  avg();
    float getDistance();
    void  setAlertDistance(float distance);
    bool  objectDetected();
    void  setImperial();
    void  setMetric();
};
 
#endif
