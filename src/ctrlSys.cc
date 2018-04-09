/*
Ctrl System - Configures and Executes Controllers

See: LICENSE.md for Copyright and License Agreement
*/

#include <iostream>

#include "ctrlSys.hxx"

// Create and Configure a map describing the groups of control systems
void CtrlSys::ConfigGroup(const ObjJson &objJson, SysGroupMap *sysGroupMap) {
  // Iterate through each of the CtrlSys entities, Create a Map of CtrlSys Classes
  assert(objJson.IsObject()); // objJson is an object, iterate through each member

  // Create Map of Classes for the Control Group
  Json2Stl_MapVecString(objJson, sysGroupMap);
}


// Create and Configure a map of Ctrl systems
void CtrlSys::ConfigDef(const ObjJson &objJson, SysDefMap *sysDefMap, DefinitionTree *signalTreePtr) {
  // Iterate through each of the CtrlSys entities, Create a Map of CtrlSys Classes
  assert(objJson.IsObject()); // objJson is an object, iterate through each member
  for (ObjJson::ConstMemberIterator iObj = objJson.MemberBegin(); iObj != objJson.MemberEnd(); ++iObj) {

    std::string nameStr = iObj->name.GetString(); // Name of System
    if (kVerboseConfig) std::cout << "Name: " << nameStr << std::endl; // Print the System Name

    const ObjJson &objCurr = objJson[iObj->name.GetString()];

    // System Map build-up
    SysDefVec sysDefVec;
    ConfigDefVec(objCurr, &sysDefVec, signalTreePtr); // Pointer to Derived Class

    // Add instance to the System Map
    sysDefMap->insert(std::make_pair(nameStr, sysDefVec));
  }
}

// Configuration of a single instance of a control system, each instance can consist of multiple controllers
void CtrlSys::ConfigDefVec(const ObjJson &objJson, SysDefVec *sysDefVec, DefinitionTree *signalTreePtr) {

  // Iterate through each of the CtrlSys entities, Create a Map of CtrlSys Classes
  assert(objJson.IsArray()); // objJson is an object, iterate through each member
  rapidjson::SizeType numElemJson = (uint8_t) objJson.Size();
  for (rapidjson::SizeType i = 0; i < numElemJson; i++) {
    if (kVerboseConfig) std::cout << "\tCtrl #: " << i;
    const ObjJson &objCurr = objJson[i];

    SysInstPtr sysInstPtr;
    ConfigDefInst(objCurr, &sysInstPtr, signalTreePtr);

    sysDefVec->emplace_back(sysInstPtr);
  }
}

// Configuration of a single instance of a controllers
void CtrlSys::ConfigDefInst(const ObjJson &objJson, SysInstPtr *sysInstPtr, DefinitionTree *signalTreePtr) {
  // Get the sytem type from the JSON object
  assert(objJson.HasMember("Desc"));
  std::string descStr = objJson["Desc"].GetString();

  assert(objJson.HasMember("Type"));
  std::string typeStr = objJson["Type"].GetString();

  if (kVerboseConfig) {
    std::cout << "  Desc: " << descStr << "  Type: " << typeStr << std::endl; // Print the System Type
  }

  // Hash the Type string into enumeration members
  EnumType eType = kNone;

  if (typeStr == "None") {
    eType = kNone;
  } else if (typeStr == "Const") {
    eType = kConst;
  } else if (typeStr == "Sum") {
    eType = kSum;
  } else if (typeStr == "Gain") {
    eType = kGain;
  } else if ((typeStr == "PID2")) {
    eType = kPid2;
  } else if (typeStr == "SS") {
    eType = kSS;
  } else {
    std::cout << "\nUnknown Type: " << typeStr << std::endl; // Print error message
  }

  // Create a pointer to the proper class of controller
  switch (eType){
    case kNone:
      *sysInstPtr = std::make_shared<CtrlNone>();
      break;
    case kConst:
      *sysInstPtr = std::make_shared<CtrlConst>();
      break;
    case kSum:
      *sysInstPtr = std::make_shared<CtrlSum>();
      break;
    case kGain:
      *sysInstPtr = std::make_shared<CtrlGain>();
      break;
    case kPid2:
      *sysInstPtr = std::make_shared<CtrlPid2>();
      break;
    case kSS:
      *sysInstPtr = std::make_shared<CtrlSS>();
      break;
  }

  // Call the Config method, this will make local copies of the controller specific parameters
  std::string defPath = descStr;
  (*sysInstPtr)->Config(objJson);

  // Parse the Signal descriptions

  float *targetPtr; // FIXIT need to point to SOMETHING!

  std::string defSignal = "RefSignal";
  ConfigSignal(objJson, defSignal, targetPtr, signalTreePtr);

  defSignal = "MeasSignal";
  ConfigSignal(objJson, defSignal, targetPtr, signalTreePtr);

  defSignal = "OutSignal";
  ConfigSignal(objJson, defSignal, targetPtr, signalTreePtr);
}

// FIXIT!! replace "RefSignal" with defSignal
void CtrlSys::ConfigSignal(const ObjJson &objJson, const std::string &defSignal, float *targetPtr, DefinitionTree *signalTreePtr) {
  if (objJson.HasMember(defSignal.c_str())) {
    assert(objJson[defSignal.c_str()].IsArray());

    // Loop through elements of the Signal Json ArraySize, get the Strings
    VecString vecString;
    Json2Stl_VecString(objJson[defSignal.c_str()], &vecString);

    for (uint8_t i = 0; i < vecString.size(); i++) {
      signalTreePtr->InitMember(vecString[i], targetPtr, "NaN", true, false);
    }

    if (kVerboseConfig) {
      std::cout << "\t\t" << defSignal << ": ";
      for (uint8_t i = 0; i < vecString.size(); i++) {
        std::cout << vecString[i] << "  ";
      }
      std::cout << std::endl;
    }
  }
}


// Controller Type Const
void CtrlConst::Config(const ObjJson &objJson) {
    // Set all the Default Values
    float val = 0.0;

    // Load Values defined in Json
    if(objJson.HasMember("Const")) val = objJson["Const"].GetFloat();

    // Configure the Controller
    val_ = val;

    // Print the Config
    if (kVerboseConfig) {
      std::cout << "\t\tConst: " << val << std::endl;
    }
}

// FIXIT
void CtrlConst::Run(DefinitionTree *signalTreePtr) {
  // put into definition tree
  
}

// Controller Type Sum
void CtrlSum::Config(const ObjJson &objJson) {

}

// FIXIT
void CtrlSum::Run(DefinitionTree *signalTreePtr) {
  // put into definition tree

}

// Controller Type Gain
void CtrlGain::Config(const ObjJson &objJson) {
    // Set all the Default Values
    float val = 1.0;

    // Load Values defined in Json
    if(objJson.HasMember("Gain")) val = objJson["Gain"].GetFloat();

    // Configure the Controller
    val_ = val;

    // Print the Config
    if (kVerboseConfig) {
      std::cout << "\t\tGain: " << val << std::endl;
    }
}

// FIXIT
void CtrlGain::Run(DefinitionTree *signalTreePtr) {
  // put into definition tree

}

void CtrlPid2::Config(const ObjJson &objJson) {
    // Set all the Default Values
    float cmdMin = -1000.0;
    float cmdMax = 1000.0;
    float Kp = 0.0;
    float Ki = 0.0;
    float Kd = 0.0;
    float b = 1.0;
    float c = 1.0;

    // Load Values defined in Json
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
    ctrlFuncPid2_.Config(Kp, Ki, Kd, b, c, cmdMin, cmdMax);

    // Print the Config
    if (kVerboseConfig) {
      std::cout << "\t\tKp: " << Kp << "  ";
      std::cout << "Ki: " << Ki << "  ";
      std::cout << "Kd: " << Kd << "  ";
      std::cout << "b: " << b << "  ";
      std::cout << "c: " << c << "  ";
      std::cout << "cmdRng: [" << cmdMin << "," << cmdMax << "]" << std::endl;
    }
}

// FIXIT
void CtrlPid2::Run(DefinitionTree *signalTreePtr) {

  // pull values from sigStruct
  // float ref = 1.0;
  // float meas = 0.0;
  // *cmd = 0.0;
  //
  // ctrlFuncPid2_.mode_ = kCtrlEngage;
  // ctrlFuncPid2_.Run(ref, meas, dt_s, cmd);

  // put cmd back into definition tree
}
