/*
Ctrl System - Configures and Executes Controllers

See: LICENSE.md for Copyright and License Agreement
*/

#include <iostream>

#include "CtrlSys.hxx"

// Create and Configure a map describing the groups of control systems
void CtrlSys::ConfigGroup(const ObjJson &objJson, SysGroupMap *sysGroupMap) {
  // Iterate through each of the CtrlSys entities, Create a Map of CtrlSys Classes
  assert(objJson.IsObject()); // objJson is an object, iterate through each member
  Json2Stl_MapVecString(objJson, sysGroupMap);
}


// Create and Configure a map of Ctrl systems
void CtrlSys::ConfigDef(const ObjJson &objJson, SysDefMap *sysDefMap) {
  // Iterate through each of the CtrlSys entities, Create a Map of CtrlSys Classes
  assert(objJson.IsObject()); // objJson is an object, iterate through each member
  for (ObjJson::ConstMemberIterator iObj = objJson.MemberBegin(); iObj != objJson.MemberEnd(); ++iObj) {

    std::string nameStr = iObj->name.GetString(); // Name of System
    if (kVerboseConfig) std::cout << "Name: " << nameStr << std::endl; // Print the System Name

    const ObjJson &objCurr = objJson[iObj->name.GetString()];

    // System Map build-up
    SysDefVec sysDefVec;
    ConfigDefVec(objCurr, &sysDefVec); // Pointer to Derived Class

    // Add instance to the System Map
    sysDefMap->insert(std::make_pair(nameStr, sysDefVec));
  }
}

// Configuration of a single instance of a control system, each instance can consist of multiple controllers
void CtrlSys::ConfigDefVec(const ObjJson &objJson, SysDefVec *sysDefVec) {

  // Iterate through each of the CtrlSys entities, Create a Map of CtrlSys Classes
  assert(objJson.IsArray()); // objJson is an object, iterate through each member
  rapidjson::SizeType numElemJson = (uint8_t) objJson.Size();
  for (rapidjson::SizeType i = 0; i < numElemJson; i++) {
    if (kVerboseConfig) std::cout << "Ctrl #: " << i;
    const ObjJson &objCurr = objJson[i];

    SysInstPtr sysInstPtr;
    ConfigDefInst(objCurr, &sysInstPtr);

    sysDefVec->emplace_back(sysInstPtr);
  }
}

// Configuration of a single instance of a controllers
void CtrlSys::ConfigDefInst(const ObjJson &objJson, SysInstPtr *sysInstPtr) {
  // Get the sytem type from the JSON object
  assert(objJson.HasMember("Type"));
  std::string typeStr = objJson["Type"].GetString();
  if (kVerboseConfig) std::cout << "\tType: " << typeStr << std::endl; // Print the System Type

  // Hash the Type string into enumeration members
  EnumType eType = kNone;

  if (typeStr == "None") {
    eType = kNone;
  } else if ((typeStr == "PID2") | (typeStr == "PID2") | (typeStr == "Pass") | (typeStr == "Manual")) {
    eType = kPid2;
  } else if (typeStr == "PI+Damp") {
    eType = kPiDamp;
  } else if (typeStr == "SS") {
    eType = kSS;
  } else {
    std::cout << "Unknown Type: " << typeStr << std::endl; // Print error message
  }

  // Create a pointer to the proper class of waveform
  switch (eType){
    case kNone:
      *sysInstPtr = std::make_shared<CtrlNone>();
      break;
    case kPid2:
      *sysInstPtr = std::make_shared<CtrlPid2>();
      break;
    case kPiDamp:
      *sysInstPtr = std::make_shared<CtrlPiDamp>();
      break;
    case kSS:
      *sysInstPtr = std::make_shared<CtrlSS>();
      break;
  }

  // Call the Config method
  (*sysInstPtr)->Config(objJson);
}


void CtrlPid2::Config(const ObjJson &objJson) {
    // Set all the Default Values
    float refScale = 1.0;
    float cmdMin = -1000.0;
    float cmdMax = 1000.0;
    float Kp = 0.0;
    float Ki = 0.0;
    float Kd = 0.0;
    float b = 1.0;
    float c = 1.0;

    // Load Values defined in Json
    if(objJson.HasMember("refScale")) refScale = objJson["refScale"].GetFloat();
    if(objJson.HasMember("Kp")) Kp = objJson["Kp"].GetFloat();
    if(objJson.HasMember("Ki")) Ki = objJson["Ki"].GetFloat();
    if(objJson.HasMember("Kd")) Kd = objJson["Kd"].GetFloat();
    if(objJson.HasMember("b")) b = objJson["Ki"].GetFloat();
    if(objJson.HasMember("c")) c = objJson["Kd"].GetFloat();

    if(objJson.HasMember("cmdRng")) {
      assert(objJson["cmdRng"].IsArray());
      cmdMin = objJson["cmdRng"][0].GetFloat();
      cmdMax = objJson["cmdRng"][1].GetFloat();
    }

    // Configure the Controller
    ctrlFuncPid2_.Config(Kp, Ki, Kd, b, c, refScale, cmdMin, cmdMax);

    // Print the Config
    if (kVerboseConfig) {
      std::cout << "\trefScale: " << refScale << "\t";
      std::cout << "Kp: " << Kp << "\t";
      std::cout << "Ki: " << Ki << "\t";
      std::cout << "Kd: " << Kd << "\t";
      std::cout << "b: " << b << "\t";
      std::cout << "c: " << c << "\t";
      std::cout << "cmdRng: [" << cmdMin << "," << cmdMax << "]" << std::endl;
    }

    // Configure Signal Definitions for controller
    // Default values
    float refVal = 0.0;
    std::pair<std::string, std::string> refStr = {"None", "None"};

    float measVal = 0.0;
    std::pair<std::string, std::string> measStr = {"None", "None"};

    float outVal = 0.0;
    std::pair<std::string, std::string> outStr = {"None", "None"};

}

// FIXIT
void CtrlPid2::Run(const float &dt_s, float *cmd) {

  // pull values from sigStruct
  float ref = 1.0;
  float meas = 0.0;
  *cmd = 0.0;

  ctrlFuncPid2_.mode_ = kCtrlEngage;
  ctrlFuncPid2_.Run(ref, meas, dt_s, cmd);

  // put cmd back into sigStruct
}
