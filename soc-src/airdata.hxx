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
  float LowPass1(const float& meas);

 private:
  float x_, a_, b_, c_;
};

struct AirdataOut {
  float temp_C;

  float alt_m;
  float altFilt_m;

  float vIas_mps;
  float vIasFilt_mps;
};

struct AirdataLog {
  float temp_C;

  float alt_m;
  float altFilt_m;

  float vIas_mps;
  float vIasFilt_mps;
};

class Airdata {
 public:
  Airdata();   // Constructor
  ~Airdata() {};  // Destructor
  void Init();    // Initialize Bias Estimate and Filters

  void BiasEst(); // Compute the Airdata biases on startup
  AirdataOut Compute(const PitotData& pitotData);
  
  AirdataLog Log(const AirdataOut& airdataOut);


 private:
  float kP0_Pa_, kK1_m_, kK2_nd_, kK3_mps_, kK4_nd_;
  AirdataOut airdata_;

  Filt filtVel_, filtAlt_;

  uint16_t biasCount_;
  float presStatic_Pa_;
  float presDiff_Pa_;
  float altBias_m_, altBiasPrev_m_;
  float presDiffBias_Pa_, presDiffBiasPrev_Pa_;

  float ComputeAlt(const float& presStatic_Pa);
  float ComputeAirspeed(const float& presDiff_Pa);

  float FiltAlt(const float& alt_m);
  float FiltAirspeed(const float& vIas_mps);

};

#endif // AIRDATA_H_
