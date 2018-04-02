/*
Mission Manager - Translate Transmitter and C2 Link inputs into usable sources for the control systems

See: LICENSE.md for Copyright and License Agreement

*/

#ifndef MISSION_MGR_HXX_
#define MISSION_MGR_HXX_

#include <stdio.h>
#include <stdint.h>

#include "cntrlMgr.hxx"


class MissMgr {
 public:
  MissMgr() {};   // Constructor
  ~MissMgr() {};  // Destructor
  void Config();    // Configure controllers and excitations
  MissMgrOut ModeMgr(const FmuData& FmuDataRef); // Manage mode
  void Reset();   // Resets the controllers and test points with in the class

  MissMgrLog Log(const MissMgrOut& missMgrOut);
  // Mission Status Structure
  struct Out {
    float t_s;   // Mission time
    uint frame_cnt;   // Mission frame counter

    bool autoEngage;   // Mission autoEngage flag, FMU/SOC master switch

    uint8_t iTest; // Index number of the current test
    bool armTest;    // Arm flag for the test system
    bool engageTest; // Engage flag for the test system
    uint8_t numTest; // Number of test points

    uint8_t iCtrl; // Index number of the current controller
    CntrlMode cntrlMode;   // Mission controller mode

    uint8_t iExcite; // Index number of the current excitation
    bool engageExcite; // Engage flag for the test system

    float tDurSens_ms;
    float tDurNav_ms;
    float tDurExcite_ms;
    float tDurCntrl_ms;
    float tCmd_ms;
    float tFrame_ms;
  };

  struct Log {
    float t_s;
    uint frame_cnt;

    bool autoEngage;

    uint8_t iTest;
    bool armTest;
    bool engageTest;
    uint8_t numTest;

    uint8_t iCtrl;
    CntrlMode cntrlMode;

    uint8_t iExcite;
    bool engageExcite;

    float tDurSens_ms;
    float tDurNav_ms;
    float tDurExcite_ms;
    float tDurCntrl_ms;
    float tCmd_ms;
    float tFrame_ms;
  };
 private:
  MissMgrOut missMgrData_;

  //ExcitDataDef excitList // Excitation Set
  uint8_t indxTrigPersist_;
  static const uint8_t threshTrigPersist_ = 4;

  bool trigArm_; // Arm flag for the trigger
  bool trigEngage_; // Engage flag for the trigger
};


#endif //MISSION_MGR_HXX_
