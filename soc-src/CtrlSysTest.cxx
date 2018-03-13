/*
Simple control system tester

See: LICENSE.md for Copyright and License Agreement

g++-5 -std=c++11 -Wall -O3 -g -I../soc-includes CtrlSys.cxx CtrlSysTest.cxx -o CtrlSysTest
./CtrlSysTest

*/

#include <iostream>
#include "CtrlSys.hxx"


int main(void)  /* Program tester */
{
  // Load JSON config file
  std::ifstream ConfigFile("../../thor_Exp.json");
  std::string ConfigBuffer((std::istreambuf_iterator<char>(ConfigFile)),std::istreambuf_iterator<char>());

  // Parse JSON config file
  rapidjson::StringStream jsonConfig(ConfigBuffer.c_str()); // Create string stream
  rapidjson::Document objConfigDom; // Define document object
  objConfigDom.ParseStream(jsonConfig); // Parse the stream
  assert(objConfigDom.IsObject()); // Check that the JSON object was created

  // Creat an object for sub-tree
  const ObjJson &objCtrlMgr = objConfigDom["CtrlMgr"];
  assert(objCtrlMgr.HasMember("CtrlSys"));
  const ObjJson &objCtrlSys = objCtrlMgr["CtrlSys"];
  assert(objCtrlSys.HasMember("Define"));
  const ObjJson &objCtrlDef = objCtrlSys["Define"];


  // Create the Configuration
  // Create a Map of WaveSys Classes
  CtrlSys::SysDefMap ctrlDefMap;
  CtrlSys::ConfigDef(objCtrlDef, &ctrlDefMap);
  std::cout << "Configuration Complete!!" << std::endl;
  std::cout << ctrlDefMap.size() << std::endl;

  // Run the ctrl
  std::string ctrlSelect = "ScasBaseline";

  float tCurr_s = 0.0;
  float cmd = 0.0;
  float ref = 0.0;
  float meas = 0.0;

  uint8_t numSelect = ctrlDefMap[ctrlSelect].size();

    // ctrlDefMap[ctrlSelect]->Run(tCurr_s, &cmd);

  for (tCurr_s = 0.0; tCurr_s < 2; tCurr_s += 0.2) {
    std::cout << tCurr_s << "\t";
    for (uint8_t vecSelect = 0; vecSelect < numSelect; vecSelect++) {
      ctrlDefMap[ctrlSelect][vecSelect]->Run(tCurr_s, &cmd);
      std::cout << cmd << "\t";
    }
    std::cout << std::endl;
  }
}
