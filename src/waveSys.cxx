/*
Classes and Functions for Waveform Generation

See: LICENSE.md for Copyright and License Agreement
*/

#include "waveSys.hxx"

// Create and Configure a map of waveform systems
void WaveFactory::Config(const ObjJson &objJson, SysMap *sysMap) {
  // Iterate through each of the WaveSys entities, Create a Map of WaveSys Classes
  assert(objJson.IsObject()); // objJson is an object, iterate through each member
  for (ObjJson::ConstMemberIterator iObj = objJson.MemberBegin(); iObj != objJson.MemberEnd(); ++iObj) {

    std::string nameStr = iObj->name.GetString(); // Name of System
    if (kVerboseConfig) std::cout << "Name: " << nameStr <<  "\t"; // Print the System Name

    const ObjJson &objCurr = objJson[iObj->name.GetString()];

    // System Map build-up
    SysPtr sysPtr;
    ConfigInst(objCurr, &sysPtr); // Pointer to Derived Class

    // Add instance to the System Map
    sysMap->insert(std::make_pair(nameStr, sysPtr));
  }
}

// Configuration of a single instance of a waveform
void WaveFactory::ConfigInst(const ObjJson &objJson, SysPtr *sysPtr) {
  // Get the Wave type from the JSON object
  assert(objJson.HasMember("Type"));
  std::string typeStr = objJson["Type"].GetString();
  if (kVerboseConfig) std::cout << "Type: " << typeStr <<  "\t"; // Print the System SubType

  // Hash the SubType string into enumeration members
  EnumType eType = kDisc;
  if (typeStr == "Discrete") {
    eType = kDisc;
  } else if (typeStr == "Chirp") {
    eType = kChirp;
  } else if (typeStr == "Multisine") {
    eType = kMultisine;
  } else {
    std::cout << "Unknown Type: " << typeStr << std::endl; // Print error message
  }

  // Create a pointer to the proper class of waveform
  switch (eType){
    case kDisc:
      *sysPtr = std::make_shared<WaveDisc>();
      break;
    case kChirp:
      *sysPtr = std::make_shared<WaveChirp>();
      break;
    case kMultisine:
      *sysPtr = std::make_shared<WaveMultisine>();
      break;
  }

  // Call the Config method
  (*sysPtr)->Config(objJson);
}


// Discrete Waveform Configuration
void WaveDisc::Config(const ObjJson &objJson) {
  // Get the Wave type from the JSON object
  assert(objJson.HasMember("SubType"));
  std::string typeStr = objJson["SubType"].GetString();
  if (kVerboseConfig) std::cout << "SubType: " << typeStr <<  "\t"; // Print the System SubType

  // Hash the SubType string into enumeration members
  if (typeStr == "Pulse") {
    eType_ = kPulse;
  } else if (typeStr == "Doublet") {
    eType_ = kDoublet;
  } else if (typeStr == "121") {
    eType_ = kDoublet121;
  } else if (typeStr == "3211") {
    eType_ = kDoublet3211;
  } else {
    std::cout << "Unknown SubType: " << typeStr << std::endl; // Print error message
  }

  assert(objJson.HasMember("tPulse_s"));
  tPulse_s_ = objJson["tPulse_s"].GetFloat();

  assert(objJson.HasMember("amp_nd"));
  amp_nd_ = objJson["amp_nd"].GetFloat();

  tDur_s_ = 0;
  switch (eType_){
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
    switch (eType_){
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
  assert(objJson.HasMember("SubType"));
  std::string typeStr = objJson["SubType"].GetString();
  if (kVerboseConfig) std::cout << "SubType: " << typeStr <<  "\t"; // Print the System SubType

  // Hash the SubType string into enumeration members
  if (typeStr == "Linear") {
    eType_ = kLinear;
  } else {
    std::cout << "Unknown SubType: " << typeStr << std::endl; // Print error message
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
    switch (eType_){
      case kLinear:
        WaveChirpLinear(tCurr_s, tDur_s_, freqStart_rps_, freqEnd_rps_, ampStart_nd_, ampEnd_nd_, wave_nd);
        break;
    }
  }
}


// MultiSine Waveform
void WaveMultisine::Config(const ObjJson &objJson) {
  // Get the Wave type from the JSON object
  assert(objJson.HasMember("SubType"));
  std::string typeStr = objJson["SubType"].GetString();
  if (kVerboseConfig) std::cout << "SubType: " << typeStr <<  "\t"; // Print the System SubType

  // Hash the SubType string into enumeration members
  if (typeStr == "OMS") {
    eType_ = kOMS;
  } else {
    std::cout << "Unknown SubType: " << typeStr << std::endl; // Print error message
  }

  assert(objJson.HasMember("tDur_s"));
  tDur_s_ = objJson["tDur_s"].GetFloat();

  assert(objJson.HasMember("freq_rps"));
  freq_rps_.conservativeResize(kMaxWaveElem);
  Json2Eigen_VecFloat(objJson["freq_rps"], &freq_rps_);

  assert(objJson.HasMember("phase_deg"));
  phase_rad_.conservativeResize(kMaxWaveElem);
  Json2Eigen_VecFloat(objJson["phase_deg"], &phase_rad_);

  assert(objJson.HasMember("amp_nd"));
  amp_nd_.conservativeResize(kMaxWaveElem);
  Json2Eigen_VecFloat(objJson["amp_nd"], &amp_nd_);

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
    switch (eType_){
      case kOMS:
        WaveMultisineOms(tCurr_s, freq_rps_, phase_rad_, amp_nd_, wave_nd);
        break;
    }
  }
}
