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
struct MissMgrOut {
  float time_s;   // Mission time
  uint frame_cnt;   // Mission frame counter

  bool autoEngage;   // Mission autoEngage flag
  CntrlMode cntrlMode;   // Mission controller mode

  bool testArm;    // Arm flag for the test system
  bool testEngage; // Engage flag for the test system
  uint8_t indxTest; // Index number of the current test
  uint8_t numTest; // Number of test points

  float timeSens_ms;
  float timeMissMgr_ms;
  float timeSensProc_ms;
  float timeExciteMgr_ms;
  float timeGuidMgr_ms;
  float timeCntrlMgr_ms;
  float timeCmd_ms;
};

struct MissMgrLog {
  float time_s;
  uint frame_cnt;

  bool autoEngage;
  CntrlMode cntrlMode;

  bool testArm;
  bool testEngage;
  uint8_t indxTest;

  float timeSens_ms;
  float timeMissMgr_ms;
  float timeSensProc_ms;
  float timeExciteMgr_ms;
  float timeGuidMgr_ms;
  float timeCntrlMgr_ms;
  float timeCmd_ms;
};

class MissMgr {
 public:
  MissMgr() {};   // Constructor
  ~MissMgr() {};  // Destructor
  void Init();    // Initialize controllers and excitations
  MissMgrOut ModeMgr(const FmuData& FmuDataRef); // Manage mode 
  void Reset();   // Resets the controllers and test points with in the class

  MissMgrLog Log(const MissMgrOut& missMgrOut);

 private:
  MissMgrOut missMgrData_;
  float time_s_;
  uint frame_cnt_;

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
