/*
Airdata - Compute Airspeed and Altitude from Pitot-Static Measurements

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-28 - Chris Regan - Created
*/

#ifndef AIRDATA_H_
#define AIRDATA_H_

#include <stdint.h>
#include "global-defs.hxx"


class Filt{ // Filter Class - FIXIT - Move into a seperate .hxx
 public:

  Filt() {};
  ~Filt() {};
  void Init();
  float LowPass1(float meas);

 private:
  float x_, a_, b_, c_;
};

struct AirdataStruct {
  float presStatic_Pa;
  float presDiff_Pa;

  float temp_C;

  float alt_m;
  float altBias_m;
  float altFilt_m;

  float presDiffBias_Pa;
  float vIas_mps;
  float vIasFilt_mps;
};

class Airdata {
 public:
  Airdata();   // Constructor
  ~Airdata() {};  // Destructor
  void Init();    // Initialize Bias Estimate and Filters

  void BiasEst(); // Compute the Airdata biases on startup
  AirdataStruct Compute(const PitotData pitotData);

 private:
  float kP0_Pa_, kK1_m_, kK2_nd_, kK3_mps_, kK4_nd_;
  AirdataStruct airdata_;

  Filt filtVel_, filtAlt_;

  uint16_t biasCount_;
  float altBiasPrev_m_;
  float presDiffBiasPrev_Pa_;

  float ComputeAlt(float presStatic_Pa);
  float ComputeAirspeed(float presDiff_Pa);

  float FiltAlt(float alt_m);
  float FiltAirspeed(float vIas_mps);

};

#endif // AIRDATA_H_
