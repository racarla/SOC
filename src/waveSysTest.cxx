/*
Simple wave system tester
Create a map of wave systems, configure, run

g++-5 -std=c++11 -Wall -O3 -g -I../includes waveFunc.cxx utilities.cxx waveSys.cxx waveSysTest.cxx -o waveSysTest
./waveSysTest

See: LICENSE.md for Copyright and License Agreement

*/

#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include "waveSys.hxx"

#ifndef kVerboseConfig
#define kVerboseConfig 1
#endif

uint8_t iWave_ = 0;

int main(void) {

  // Load JSON config file
  std::ifstream ConfigFile("../thor_Exp.json");
  std::string ConfigBuffer((std::istreambuf_iterator<char>(ConfigFile)),std::istreambuf_iterator<char>());

  // Parse JSON config file
  rapidjson::StringStream jsonConfig(ConfigBuffer.c_str()); // Create string stream
  rapidjson::Document objConfigDom; // Define document object
  objConfigDom.ParseStream(jsonConfig); // Parse the stream
  assert(objConfigDom.IsObject()); // Check that the JSON object was created

  // Creat an object for sub-tree
  const ObjJson &objMissMgr = objConfigDom["MissionMgr"];
  assert(objMissMgr.HasMember("WaveSys"));
  const ObjJson &objWaveSys = objMissMgr["WaveSys"];

  // Create the Configuration
  // Create a Map of WaveSys Classes
  WaveFactory::SysMap waveSysMap;
  WaveFactory::Config(objWaveSys, &waveSysMap);
  std::cout << "Configuration Complete!!" << std::endl;
  std::cout << waveSysMap.size() << std::endl;


  // Run the waves
  std::string waveSelect = "4";
  float tCurr_s = 0.0;
  float wave_nd = 0.0;
  for (tCurr_s = 0.0; tCurr_s < 2; tCurr_s += 0.2) {
    waveSysMap[waveSelect]->Run(tCurr_s, &wave_nd);
    std::cout << tCurr_s << "\t" << wave_nd << std::endl;
  }
}
