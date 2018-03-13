/*
Classes and Functions for Waveform Generation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
2017-11-15 - Chris Regan - Added Classes and Eigen for full vectorization
2018-03-05 - Chris Regan - Renamed to WaveGenFunc
*/

#include "WaveSys.hxx"

// Create and Configure a map of waveform systems
void WaveFactory::Config(const ObjJson &objJson, WaveSysMap *waveSysMap) {
  // Iterate through each of the WaveSys entities, Create a Map of WaveSys Classes
  assert(objJson.IsObject()); // objJson is an object, iterate through each member
  for (ObjJson::ConstMemberIterator iObj = objJson.MemberBegin(); iObj != objJson.MemberEnd(); ++iObj) {

    std::string waveNameStr = iObj->name.GetString(); // Name of System
    if (kVerboseConfig) std::cout << "Name: " << waveNameStr <<  "\t"; // Print the System Name

    const ObjJson &objCurr = objJson[iObj->name.GetString()];

    // System Map build-up
    WaveSysPtr waveSysPtr;
    WaveFactory::ConfigInst(objCurr, &waveSysPtr); // Pointer to Derived Class

    // Add instance to the System Map
    waveSysMap->insert(std::make_pair(waveNameStr, waveSysPtr));
  }
}

// Configuration of a single instance of a waveform
void WaveFactory::ConfigInst(const ObjJson &objJson, WaveSysPtr *waveSysPtr) {
  // Get the Wave type from the JSON object
  assert(objJson.HasMember("Class"));
  std::string waveClassStr = objJson["Class"].GetString();
  if (kVerboseConfig) std::cout << "Class: " << waveClassStr <<  "\t"; // Print the System Type

  // Hash the Type string into enumeration members
  EnumWaveClass eWaveClass = kDisc;
  if (waveClassStr == "Discrete") {
    eWaveClass = kDisc;
  } else if (waveClassStr == "Chirp") {
    eWaveClass = kChirp;
  } else if (waveClassStr == "Multisine") {
    eWaveClass = kMultisine;
  } else {
    std::cout << "Unknown Class: " << waveClassStr << std::endl; // Print error message
  }

  // Create a pointer to the proper class of waveform
  switch (eWaveClass){
    case kDisc:
      *waveSysPtr = std::make_shared<WaveDisc>();
      break;
    case kChirp:
      *waveSysPtr = std::make_shared<WaveChirp>();
      break;
    case kMultisine:
      *waveSysPtr = std::make_shared<WaveMultisine>();
      break;
  }

  // Call the Config method
  (*waveSysPtr)->Config(objJson);
}


// Discrete Waveform Configuration
void WaveDisc::Config(const ObjJson &objJson) {
  // Get the Wave type from the JSON object
  assert(objJson.HasMember("Type"));
  std::string waveTypeStr = objJson["Type"].GetString();
  if (kVerboseConfig) std::cout << "Type: " << waveTypeStr <<  "\t"; // Print the System Type

  // Hash the Type string into enumeration members
  if (waveTypeStr == "Pulse") {
    eWaveDiscType_ = kPulse;
  } else if (waveTypeStr == "Doublet") {
    eWaveDiscType_ = kDoublet;
  } else if (waveTypeStr == "121") {
    eWaveDiscType_ = kDoublet121;
  } else if (waveTypeStr == "3211") {
    eWaveDiscType_ = kDoublet3211;
  } else {
    std::cout << "Unknown Type: " << waveTypeStr << std::endl; // Print error message
  }

  assert(objJson.HasMember("tPulse_s"));
  tPulse_s_ = objJson["tPulse_s"].GetFloat();

  assert(objJson.HasMember("amp_nd"));
  amp_nd_ = objJson["amp_nd"].GetFloat();

  tDur_s_ = 0;
  switch (eWaveDiscType_){
    case kPulse:
      tDur_s_ = 1 * tPulse_s_;
      break;
    case kDoublet:
      tDur_s_ = 2 * tPulse_s_;
      break;
    case kDoublet121:
      tDur_s_ = 4 * tPulse_s_;
      break;
    case kDoublet3211:
      tDur_s_ = 7 * tPulse_s_;
      break;
  }

  // Print the Config
  if (kVerboseConfig) {
    std::cout << "tPulse_s: " << tPulse_s_ << "\t";
    std::cout << "amp_nd: " << tPulse_s_ << std::endl;
  }
}

void WaveDisc::Run(const float &tCurr_s, float *wave_nd) {
  *wave_nd = 0.0;

  if ((tCurr_s >= 0) && (tCurr_s < tDur_s_)) {
    // Call the Run method for the class of waveform
    switch (eWaveDiscType_){
      case kPulse:
        WavePulse(tCurr_s, tPulse_s_, amp_nd_, wave_nd);
        break;
      case kDoublet:
        WaveDoublet(tCurr_s, tPulse_s_, amp_nd_, wave_nd);
        break;
      case kDoublet121:
        WaveDoublet121(tCurr_s, tPulse_s_, amp_nd_, wave_nd);
        break;
      case kDoublet3211:
        WaveDoublet3211(tCurr_s, tPulse_s_, amp_nd_, wave_nd);
        break;
    }
  }
}


/* Chirp Waveform */
void WaveChirp::Config(const ObjJson &objJson) {
  // Get the Wave type from the JSON object
  assert(objJson.HasMember("Type"));
  std::string waveTypeStr = objJson["Type"].GetString();
  if (kVerboseConfig) std::cout << "Type: " << waveTypeStr <<  "\t"; // Print the System Type

  // Hash the Type string into enumeration members
  if (waveTypeStr == "Linear") {
    eWaveChirpType_ = kLinear;
  } else {
    std::cout << "Unknown Type: " << waveTypeStr << std::endl; // Print error message
  }

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
    std::cout << "tDur_s: \t" << tDur_s_ << "\t";
    std::cout << "freq_rps: [" << freqStart_rps_ << "," << freqEnd_rps_ << "]" << "\t";
    std::cout << "amp_nd: [" << ampStart_nd_ << "," << ampEnd_nd_ << "]" << std::endl;
  }
}

void WaveChirp::Run(const float &tCurr_s, float *wave_nd) {
  *wave_nd = 0.0;

  if ((tCurr_s >= 0) && (tCurr_s < tDur_s_)) {
    // Call the Run method for the class of waveform
    switch (eWaveChirpType_){
      case kLinear:
        WaveChirpLinear(tCurr_s, tDur_s_, freqStart_rps_, freqEnd_rps_, ampStart_nd_, ampEnd_nd_, wave_nd);
        break;
    }
  }
}


// MultiSine Waveform
void WaveMultisine::Config(const ObjJson &objJson) {
  // Get the Wave type from the JSON object
  assert(objJson.HasMember("Type"));
  std::string waveTypeStr = objJson["Type"].GetString();
  if (kVerboseConfig) std::cout << "Type: " << waveTypeStr <<  "\t"; // Print the System Type

  // Hash the Type string into enumeration members
  if (waveTypeStr == "OMS") {
    eWaveMultisineType_ = kOMS;
  } else {
    std::cout << "Unknown Type: " << waveTypeStr << std::endl; // Print error message
  }

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
    std::cout << "tDur_s: " << tDur_s_ << "\t";
    std::cout << "freq_rps: <" << freq_rps_.size() << ">\t";
    std::cout << "phase_deg: <" << phase_rad_.size() << ">\t";
    std::cout << "amp_nd: <" << amp_nd_.size() << ">" << std::endl;
  }
}

void WaveMultisine::Run(const float &tCurr_s, float *wave_nd) {
  *wave_nd = 0.0;

  if ((tCurr_s >= 0) && (tCurr_s < tDur_s_)) {
    // Call the Run method for the class of waveform
    switch (eWaveMultisineType_){
      case kOMS:
        WaveMultisineOms(tCurr_s, freq_rps_, phase_rad_, amp_nd_, wave_nd);
        break;
    }
  }
}
