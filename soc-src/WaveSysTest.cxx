/*
Simple wave system tester
Create a map of wave systems, configure, run

g++-5 -std=c++11 -Wall -O0 -g -I../soc-includes WaveGenFunc.cxx Utilities.cxx WaveSys.cxx WaveSysTest.cxx -o WaveSysTest

g++-5 -std=c++11 -Wall -O3 -g -I../soc-includes WaveGenFunc.cxx Utilities.cxx WaveSys.cxx WaveSysTest.cxx -o WaveSysTest

./WaveSysTest

See: LICENSE.md for Copyright and License Agreement

*/

#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include "WaveSys.hxx"

#ifndef kVerboseConfig
#define kVerboseConfig 1
#endif

uint8_t iWave_ = 0;

int main(void) {

  // Load JSON config file
  std::ifstream ConfigFile("../../thor_Exp.json");
  std::string ConfigBuffer((std::istreambuf_iterator<char>(ConfigFile)),std::istreambuf_iterator<char>());

  // Parse JSON config file
  rapidjson::StringStream jsonConfig(ConfigBuffer.c_str()); // Create string stream
  rapidjson::Document objConfigDom; // Define document object
  objConfigDom.ParseStream(jsonConfig); // Parse the stream
  assert(objConfigDom.IsObject()); // Check that the JSON object was created

  // Creat an object for Control sub-tree
  const ObjJson &objMissMgr = objConfigDom["MissionMgr"];

  // Pull the Vehicle Definitions, convert them into a Map of Map floats.
  assert(objMissMgr.HasMember("WaveSys")); // Check that VehDef exists
  const ObjJson &objWaveSys = objMissMgr["WaveSys"]; // Create Signals Object
  assert(objWaveSys.IsObject()); // WaveSys is an array, iterate through each

  // Create the Configuration
  // Create a Map of WaveSys Classes
  WaveSysMap waveSysMap;
  waveSysMap = WaveFactory::Config(objWaveSys);

  std::cout << "Configuration Complete!!" << std::endl;

  // Run the waves
  std::string waveSelect = "4";
  float tCurr_s = 0.0;
  float wave_nd = 0.0;
  for (tCurr_s = 0.0; tCurr_s < 20; tCurr_s += 0.02) {
    waveSysMap[waveSelect]->Run(tCurr_s, wave_nd);
    std::cout << tCurr_s << "\t" << wave_nd << std::endl;
  }
}
