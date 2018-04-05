
/*
Excite System Manager - Defines Excitations, Computes commands

See: LICENSE.md for Copyright and License Agreement
*/

#include "exciteSys.hxx"

const float kD2R = M_PI / 180.0;

void ExciteFactory::Config(const ObjJson &objJson, const SysWaveMap &sysWaveMap, const SignalMap &signalMap, SysMap sysMap) {

  // Create a Map of System Classes
  SysMap sysMap;

  // Iterate through each of the ExciteSys entities, Create a Map of ExciteSys Classes
  assert(objJson.IsObject()); // objJson is an object, iterate through each member
  for (ObjJson::ConstMemberIterator iObj = objJson.MemberBegin(); iObj != objJson.MemberEnd(); ++iObj) {

    std::string nameStr = iObj->name.GetString(); // Name of System
    if (kVerboseConfig) std::cout << "Name: " << nameStr <<  "\t"; // Print the System Name

    const ObjJson &objCurr = objJson[iObj->name.GetString()];

    // System Map build-up
    SysPtr sysPtr = ExciteFactory::ConfigInst(objCurr); // Pointer to Derived Class

    // Add instance to the System Map
    sysMap.insert(std::make_pair(nameStr, sysPtr));
  }




  return sysMap;


}


"1": {"Inject": [{"CtrlAlloc": "Pitch"}], "WaveSys": ["1"], "amp_nd": [0.0698132], "tStart_s": [1.0], "tEndPad_s": [1.0]},
"2": {"Inject": [{"CtrlAlloc": "Roll"}], "WaveSys": ["2"], "amp_nd": [0.0698132], "tStart_s": [1.0], "tEndPad_s": [1.0]},


typedef VecStringPair std::vector<std::pair<std::string,std::string>>;

void ExciteFactory::ConfigInst(const ObjJson &objJson) {
    // Get the Injection type from the JSON object
    assert(objJson.HasMember("Inject"));
    VecStringPair injectStrVec = Json2Stl_VecStringPair(objJson["Inject"]); // JSON to VecStrPair
    // if (kVerboseConfig) std::cout << "Inject: " << injectStrVec <<  "\t";

    // Get the Injection type from the JSON object
    assert(objJson.HasMember("WaveSys"));
    VecString waveStrVec = Json2Stl_VecString(objJson["WaveSys"]); // JSON to VecStr
    // if (kVerboseConfig) std::cout << "WaveSys: " << waveStr <<  "\t";


    // Get the Injection amplitude from the JSON object
    assert(objJson.HasMember("amp_nd"));
    VecFloat ampVec_nd = Json2Stl_VecFloat(objJson["amp_nd"]); // JSON to VecStr
    // if (kVerboseConfig) std::cout << "ampVec_nd: " << ampVec_nd <<  "\t"; // Print the System Type

    // Get the Injection amplitude from the JSON object
    assert(objJson.HasMember("tStart_s"));
    VecFloat tStartVec_s = Json2Stl_VecFloat(objJson["tStart_s"]); // JSON to VecStr
    // if (kVerboseConfig) std::cout << "tStartVec_s: " << tStartVec_s <<  "\t"; // Print the System Type

    // Get the Injection amplitude from the JSON object
    assert(objJson.HasMember("tEndPad_s"));
    VecFloat tEndPadVec_s = Json2Stl_VecFloat(objJson["tEndPad_s"]); // JSON to VecStr
    // if (kVerboseConfig) std::cout << "tEndPadVec_s: " << tEndPadVec_s <<  "\t"; // Print the System Type


    // Create a pointer to the class
    SysPtr sysPtr = std::make_shared<ExciteSys>();; // Create pointer to Base Class, Cast to inherited Class

    // Call the Config method
    sysPtr->Config(objJson);

    // Return
    return sysPtr;

}

void ExciteSys::Config(const ObjJson &objJson) {

}

void ExciteSys::Run(const float& t_s, ExciteSysOut exciteSysOut)
{
  exciteSysOut.exciteMode = exciteMode;
  exciteSysOut.indxTest = indxTest;
  exciteSysOut.tExcite_s = 0.0;

  exciteSysOut.cmdExcite.setZero(4);
  VecChan cmdExciteTemp(4);
  cmdExciteTemp.setZero();

  if (exciteMode == 1) {
    for () {
        tCurr_s =
        waveSysMap[waveSelect]->Run(tCurr_s, wave_nd);

    }


    exciteSysOut.tExcite_s = t_s - tEngage_s_;
  }

}


ExciteSysLog ExciteSys::Log(const ExciteSysOut& ExciteSysOut)
{
  ExciteSysLog exciteSysLog;

  exciteSysLog.exciteMode = ExciteSysOut.exciteMode;
  exciteSysLog.tExcite_s = ExciteSysOut.tExcite_s;

  for (int i = 0; i < kMaxExciteChan; i++) {
    exciteSysLog.cmdExcite[i] = ExciteSysOut.cmdExcite[i];
  }

  return exciteSysLog;
}
