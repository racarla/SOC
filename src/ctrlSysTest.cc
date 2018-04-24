/*
Simple control system tester

See: LICENSE.md for Copyright and License Agreement

g++-7 -std=c++17 -Wall -O3 -g -I../includes definition-tree.cc configFunc.cc ctrlFunc.cc ctrlSys.cc ctrlSysTest.cc -o ctrlSysTest
./ctrlSysTest

*/

#include <iostream>

const uint8_t kMaxGuidCmd = 6;
const uint8_t kMaxScasCmd = kMaxGuidCmd;
const uint8_t kMaxCtrlEff = 20;

#include "ctrlSys.hxx"

typedef Eigen::Matrix<float, -1, 1, 0, kMaxGuidCmd, 1> VecGuid;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxScasCmd, 1> VecScas;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxCtrlEff, 1> VecEff;

#include "configuration.hxx"
DefinitionTree signalTree;

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
  CtrlSys::ConfigDef(objCtrlDef, &ctrlDefMap, &signalTree);
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

  // for (tCurr_s = 0.0; tCurr_s < 2; tCurr_s += 0.2) {
  //
  //   std::cout << tCurr_s << "\t";
  //
  //   uint8_t numSet = ctrlGroupMap[groupSelect].size();
  //   for (uint8_t iSet = 0; iSet < numSet; iSet++) {
  //
  //     std::string ctrlSelect = ctrlGroupMap[groupSelect][iSet];
  //
  //     uint8_t numSelect = ctrlDefMap[ctrlSelect].size();
  //
  //     // Run each of the controllers defined in the set, in defined sequence
  //     for (uint8_t vecSelect = 0; vecSelect < numSelect; vecSelect++) {
  //       ctrlDefMap[ctrlSelect][vecSelect]->Run(&signalTree);
  //       std::cout << cmd << "\t";
  //     }
  //   }
  //   std::cout << std::endl;
  // }
}
