/*
Simple control system tester

See: LICENSE.md for Copyright and License Agreement

g++-5 -std=c++11 -Wall -O3 -g -I../soc-includes Utilities.cxx CtrlFunc.cxx CtrlSys.cxx CtrlSysTest.cxx -o CtrlSysTest
./CtrlSysTest

*/

#include <iostream>
#include "CtrlSys.hxx"


int main(void)  /* Program tester */
{
  // Load JSON config file
  std::ifstream ConfigFile("../thor_Exp.json");
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

  // Create the Control Definitions
  assert(objCtrlSys.HasMember("Define"));
  const ObjJson &objCtrlDef = objCtrlSys["Define"];

  // Create a Map of Ctrl Classes
  CtrlSys::SysDefMap ctrlDefMap;
  CtrlSys::ConfigDef(objCtrlDef, &ctrlDefMap);
  std::cout << "Control Definitions Complete!!\tDefined: " << ctrlDefMap.size() << std::endl;

  // Create the Control Groups
  assert(objCtrlSys.HasMember("Group"));
  const ObjJson &objCtrlGroup = objCtrlSys["Group"];

  // Create a Map of strings, the strings are keys to accessing the Defined controllers
  CtrlSys::SysGroupMap ctrlGroupMap;
  CtrlSys::ConfigGroup(objCtrlGroup, &ctrlGroupMap);
  std::cout << "Control Groups Complete!!\tDefined: " << ctrlGroupMap.size() << std::endl;


  // Run the ctrl
  float tCurr_s = 0.0;
  float cmd = 0.0;
  float ref = 0.0;
  float meas = 0.0;

  std::string groupSelect = "Baseline";

  for (tCurr_s = 0.0; tCurr_s < 2; tCurr_s += 0.2) {

    std::cout << tCurr_s << "\t";

    uint8_t numSet = ctrlGroupMap[groupSelect].size();
    for (uint8_t iSet = 0; iSet < numSet; iSet++) {

      std::string ctrlSelect = ctrlGroupMap[groupSelect][iSet];

      uint8_t numSelect = ctrlDefMap[ctrlSelect].size();

      // Run each of the controllers defined in the set, in defined sequence
      for (uint8_t vecSelect = 0; vecSelect < numSelect; vecSelect++) {
        ctrlDefMap[ctrlSelect][vecSelect]->Run(tCurr_s, &cmd);
        std::cout << cmd << "\t";
      }
    }
    std::cout << std::endl;
  }
}
