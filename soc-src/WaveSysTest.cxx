/*
Simple wave system tester
Create a vector of wave systems, configure, run

See: LICENSE.md for Copyright and License Agreement

*/

#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include "WaveSys.hxx"

#define kVerboseConfig 1

typedef std::vector <std::shared_ptr<WaveSys>> WaveVec;

WaveVec Config(const ObjJson &objJson);

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
  assert(objWaveSys.IsArray()); // WaveSys is an array, iterate through each

  // Create the Configuration
  // Create a Vector of WaveSys Classes
  WaveVec waveVec = Config(objWaveSys);

  std::cout << "Done!!" << std::endl;

  // Run the waves
  float tCurr_s = 0.0;
  for (tCurr_s = 0.0; tCurr_s < 2; tCurr_s += 0.2) {
    float wave_nd = waveVec[1]->Run(tCurr_s);
    std::cout << tCurr_s << "\t" << wave_nd << std::endl;
  }
}


WaveVec Config(const ObjJson &objJson)
{
  // Create a Vector of WaveSys Classes
  WaveVec waveVec;

  // Iterate through each of the WaveSys entities, Create a Vector of WaveSys Classes
  rapidjson::SizeType numWave_ = objJson.Size(); // Number of loop closures
  for (rapidjson::SizeType iWave = 0; iWave < numWave_; ++iWave) {
    // Get the Wave type from the JSON object

    assert(objJson[iWave].HasMember("iWave"));
    iWave_ = objJson[iWave]["iWave"].GetInt();
    assert((int) iWave_ == iWave); // Warning - The iWave in JSON should be uniformly increasing

    // waveVec is built by adding the current class to back of the Vector
    waveVec.emplace_back(new WaveSys());

    // Call the Class Config method, need to cast the pointer to the proper derived class.
    waveVec[iWave]->Config(objJson[iWave]);

    // Cast the pointer to the proper derived class, if required

  }

  return waveVec;
}
