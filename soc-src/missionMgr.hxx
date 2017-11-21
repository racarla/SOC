/*
Mission Manager - Translate Transmitter and C2 Link inputs into usable sources for the control systems

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-15 - Chris Regan - Created
*/


#ifndef MISSION_MGR_HXX_
#define MISSION_MGR_HXX_

#include <stdio.h>
#include <stdint.h>

#include "global-defs.hxx"
#include "cntrlMgr.hxx"



// Mission Status Structure
struct MissionMode {
  float time_s;   // Mission time
  int frame_cnt;   // Mission frame counter

  bool autoEngage;   // Mission autoEngage flag
  CntrlMode cntrlMode;   // Mission controller mode

  bool trigArm; // Arm flag for the trigger
  bool trigEngage; // Engage flag for the trigger

  bool testArm;    // Arm flag for the test system
  bool testEngage; // Engage flag for the test system
  uint8_t indxTest; // Index number of the current test
  uint8_t numTest; // Number of test points
};

class MissionMgr {
 public:
  MissionMgr() {};   // Constructor
  ~MissionMgr() {};  // Destructor
  void Init();    // Initialize controllers and excitations
  MissionMode ModeMgr(const FmuData FmuDataRef); // Manage mode 
  void Reset();   // Resets the controllers and test points with in the class

 private:
  MissionMode missionMode_;
  float time_s_;
  int frame_cnt_;

  bool autoEngage_;     // Mission autoEngage flag
  CntrlMode cntrlMode_; // Controller mode

  //ExcitDataDef excitList // Excitation Set
  uint8_t indxTrigPersist_;
  static const uint8_t threshTrigPersist_ = 4;

  bool trigArm_; // Arm flag for the trigger
  bool trigEngage_; // Engage flag for the trigger

  bool testArm_;     // Arm flag for the test system
  bool testEngage_;  // Engage flag for the test system
  uint8_t indxTest_; // Index number of the current test
  uint8_t numTest_;  // Number of test points
};


#endif //MISSION_MGR_HXX_
