/*
Classes and Functions for Waveform Generation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
2017-11-15 - Chris Regan - Added Classes and Eigen for full vectorization
2018-03-05 - Chris Regan - Renamed to WaveGenFunc
*/

#include "WaveSys.hxx"
#include <iostream>

// Waveform Configuration
void WaveSys::Config(const ObjJson &objJson)
{
  // Get the Wave type from the JSON object
  assert(objJson.HasMember("iWave"));
  iWave_ = objJson["iWave"].GetInt();
  std::cout << iWave_ << std::endl;

  // Get the Wave type from the JSON object
  assert(objJson.HasMember("Type"));
  sWaveType_ = objJson["Type"].GetString();

  // Hash the Type string into an enumeration member
  if (sWaveType_ == "Pulse") {
    eWaveClass_ = kDisc;
    eWaveType_ = kPulse;
  } else if (sWaveType_ == "Doublet") {
    eWaveClass_ = kDisc;
    eWaveType_ = kDoublet;
  } else if (sWaveType_ == "Doublet121") {
    eWaveClass_ = kDisc;
    eWaveType_ = kDoublet121;
  } else if (sWaveType_ == "Doublet3211") {
    eWaveClass_ = kDisc;
    eWaveType_ = kDoublet3211;
  } else if (sWaveType_ == "ChirpLin") {
    eWaveClass_ = kChirp;
    eWaveType_ = kChirpLin;
  } else if (sWaveType_ == "OMS") {
    eWaveClass_ = kMultisine;
    eWaveType_ = kOMS;
  } else {
    printf("Wave: %d\t Unknown Type: %s\n", iWave_, sWaveType_); // Print error message
  }

  // Call the Config method for the class of waveform
  switch (eWaveClass_){
    case kDisc:
      WaveDisc::Config(objJson);
    case kChirp:
      WaveChirp::Config(objJson);
    case kMultisine:
      WaveMultisine::Config(objJson);
  }
}

float WaveSys::Run(float tCurr_s)
{
  float wave_nd = 0.0;
  std::cout << "Run" << std::endl;

  if ((tCurr_s >= 0) && (tCurr_s < tDur_s_)) {
    // Call the Run method for the class of waveform
    switch (eWaveClass_){
      case kDisc:
        wave_nd = WaveDisc::Run(tCurr_s);
        break;
      case kChirp:
        wave_nd = WaveChirp::Run(tCurr_s);
        break;
      case kMultisine:
        wave_nd = WaveMultisine::Run(tCurr_s);
        break;
    }
  }

  return wave_nd;
}



// Discrete Waveform Configuration
void WaveDisc::Config(const ObjJson &objJson)
{
  assert(objJson.HasMember("tPulse_s"));
  tPulse_s_ = objJson["tPulse_s"].GetFloat();

  assert(objJson.HasMember("amp_nd"));
  amp_nd_ = objJson["amp_nd"].GetFloat();

  tDur_s_ = 0;
  switch (eWaveType_){
    case kPulse:
      tDur_s_ = 1 * tPulse_s_;
    case kDoublet:
      tDur_s_ = 2 * tPulse_s_;
    case kDoublet121:
      tDur_s_ = 4 * tPulse_s_;
    case kDoublet3211:
      tDur_s_ = 7 * tPulse_s_;
  }

  // Print the Config
  if (kVerboseConfig) {
    assert(objJson.HasMember("iWave"));
    printf("iWave: %d\t", iWave_);
    printf("Type: %s\t", sWaveType_);
    printf("tPulse_s: %d\t", tPulse_s_);
    printf("amp_nd: %d\t", tPulse_s_);
  }
}

float WaveDisc::Run(float tCurr_s)
{
  float wave_nd = 0.0;

  // Call the Run method for the class of waveform
  switch (eDiscType_){
    case kPulse:
      wave_nd = WavePulse(tCurr_s, tPulse_s_, amp_nd_);
      break;
    case kDoublet:
      wave_nd = WaveDoublet(tCurr_s, tPulse_s_, amp_nd_);
      break;
    case kDoublet121:
      wave_nd = WaveDoublet121(tCurr_s, tPulse_s_, amp_nd_);
      break;
    case kDoublet3211:
      wave_nd = WaveDoublet3211(tCurr_s, tPulse_s_, amp_nd_);
      break;
  }

  return wave_nd;
}


/* Chirp Waveform */
void WaveChirp::Config(const ObjJson &objJson)
{
  assert(objJson.HasMember("tDur_s"));
  tDur_s_ = objJson["tDur_s"].GetFloat();

  assert(objJson.HasMember("freq_rps"));
  assert(objJson["freq_rps"].IsArray());
  freqStart_rps_ = objJson["freq_rps"][0].GetFloat();
  freqEnd_rps_ = objJson["freq_rps"][1].GetFloat();

  assert(objJson.HasMember("amp_nd"));
  assert(objJson["amp_nd"].IsArray());
  ampStart_nd_ = objJson["amp_nd"][0].GetFloat();
  ampEnd_nd_ = objJson["amp_nd"][1].GetFloat();

  // Print the Config
  if (kVerboseConfig) {
    printf("iWave: %d\t", iWave_);
    printf("Type: %s\t", sWaveType_);
    printf("tDur_s: %d\t", tDur_s);
    printf("freq_rps: [%d, %d]\t", freqStart_rps_, freqEnd_rps_);
    printf("amp_nd: [%d, %d]\n", ampStart_nd_, ampEnd_nd_);
  }

}

float WaveChirp::Run(float tCurr_s)
{
  float wave_nd = 0.0;

  // Call the Run method for the class of waveform
  switch (eWaveType_){
    case kLinear:
      wave_nd = WaveGenChirpLinear(tCurr_s, tDur_s_, freqVecStart_rps_, freqVecEnd_rps_, ampVecStart_nd_, ampVecEnd_nd_);
      break;
    }

  return wave_nd;
}


// MultiSine Waveform
void WaveMultisine::Config(const ObjJson &objJson)
{
  assert(objJson.HasMember("tDur_s"));
  tDur_s_ = objJson["tDur_s"].GetFloat();

  assert(objJson.HasMember("freq_rps"));
  freq_rps_ = Json2Eigen_VecFloat(objJson["freq_rps"]);

  assert(objJson.HasMember("phase_deg"));
  phase_rad_ = Json2Eigen_VecFloat(objJson["phase_deg"]);

  assert(objJson.HasMember("amp_nd"));
  amp_nd_ = Json2Eigen_VecFloat(objJson["amp_nd"]);

  // Print the Config
  if (kVerboseConfig) {
    printf("iWave: %d\t", iWave_);
    printf("Type: %s\t", sWaveType_);
    printf("tDur_s: %d\t", tDur_s);
    printf("freq_rps: %d\n", freq_rps_);
    printf("phase_deg: %d\n", phase_rad_);
    printf("amp_nd: %d\n", amp_nd_);
  }
}

float WaveMultisine::Run(float tCurr_s)
{
  float wave_nd = 0.0;

  // Call the Run method for the class of waveform
  switch (eWaveType_){
    case kOMS:
      wave_nd = WaveGenMultisine(tCurr_s, freq_rps_, phase_rad_, amp_nd_);
      break;
  }

  return wave_nd;
}
