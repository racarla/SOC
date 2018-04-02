/*
Airdata - Compute Airspeed and Altitude from Pitot-Static Measurements

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-28 - Chris Regan - Created
*/

#include "airdata.hxx"

Airdata::Airdata() {
  // Initialize the filters
  Filt filtVel_, filtAlt_;
}

void Airdata::Init() {
  kP0_Pa_ = 101325.1600806359; // Standard pressure
  kK1_m_ = 44330.739888;       // Constant for computing altitude from pressures
  kK2_nd_ = 0.1903;            // Constant for computing altitude from pressures
  kK3_mps_ = 760.4262731;      // Constant for computing airspeed from pressures
  kK4_nd_ = 0.285714285714286 ;// Constant for computing airspeed from pressures

  biasCount_ = 0;
  altBiasPrev_m_ = 0.0;
  presDiffBiasPrev_Pa_ = 0.0;

  // Initialize Filters
  filtAlt_.Init();
  filtVel_.Init();
}



AirdataOut Airdata::Compute(const PitotData& pitotData) {

  presStatic_Pa_ = pitotData.Static.Pressure_Pa;
  presDiff_Pa_ = pitotData.Diff.Pressure_Pa;

  airdata_.temp_C = 0.5 * (pitotData.Static.Temp_C + pitotData.Diff.Temp_C);

  airdata_.alt_m = ComputeAlt(presStatic_Pa_);
  airdata_.altFilt_m = FiltAlt(airdata_.alt_m);

  airdata_.vIas_mps = ComputeAirspeed(presDiff_Pa_);
  airdata_.vIasFilt_mps = FiltAirspeed(airdata_.vIas_mps);

  return airdata_;
}

float Airdata::ComputeAlt(const float& presStatic_Pa) {

  // Compute pressure altitude; bias removal results in AGL altitude
  float alt_m = kK1_m_ * (1 - pow((presStatic_Pa) / kP0_Pa_ , kK2_nd_)) - altBias_m_;

  return alt_m;
}

float Airdata::FiltAlt(const float& alt_m) {
  // Filter altitude and airspeed signals
  float altFilt_m = filtAlt_.LowPass1(alt_m);

  return altFilt_m;
}

float Airdata::ComputeAirspeed(const float& presDiff_Pa) {

  // Compute Indicated Airspeed (IAS). This equation accounts for compressibility effects. Sensor bias is removed prior to calculation
  float vIas_mps = copysign(kK3_mps_ * sqrt(fabs(pow(fabs((presDiff_Pa - presDiffBias_Pa_) / kP0_Pa_ + 1), kK4_nd_) - 1)), presDiff_Pa);

  return vIas_mps;
}

float Airdata::FiltAirspeed(const float& vIas_mps) {
  // Filter airspeed
  float vIasFilt_mps = filtVel_.LowPass1(vIas_mps);

  return vIasFilt_mps;
}


void Airdata::BiasEst(){
	biasCount_++;

  // Compute the biases as a moving average
	altBias_m_ = altBiasPrev_m_ * (1 - 1/biasCount_) + airdata_.alt_m * (1 / biasCount_);
  presDiffBias_Pa_ = presDiffBiasPrev_Pa_ * (1 - 1/biasCount_) + presDiff_Pa_ * (1 / biasCount_);

  // Store the moving average states
	altBiasPrev_m_ = altBias_m_;
  presDiffBiasPrev_Pa_ = presDiffBias_Pa_;
}


void Filt::Init() {
  x_ = 0.0; // Filter state

  // Coefficients come from a discretized low pass filter at 2 Hz (50 Hz frame)
  // matlab: freq = 2 * (2*pi); sysc = tf(freq, [1, freq]); ss(c2d(sysc, 0.02));
  a_ = 0.7778;
  b_ = 0.5000;
  c_ = 0.4445;
}

float Filt::LowPass1(const float& meas)
{
	x_ = (a_ * x_) + (b_ * meas); // State update equation
  float y = c_ * x_;

	return y;
}

AirdataLog Airdata::Log(const AirdataOut& airdataOut)
{
  AirdataLog airdataLog;

  airdataLog.temp_C = airdataOut.temp_C;

  airdataLog.alt_m = airdataOut.alt_m;
  airdataLog.altFilt_m = airdataOut.altFilt_m;

  airdataLog.vIas_mps = airdataOut.vIas_mps;
  airdataLog.vIasFilt_mps = airdataOut.vIasFilt_mps;

  return airdataLog;
}
