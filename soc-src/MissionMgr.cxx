/*
Mission Manager - Define the mission parameters, control system modes

See: LICENSE.md for Copyright and License Agreement

*/

#include "MissionMgr.hxx"

void MissMgr::Config(const ObjJson &objJson)
{
  // Configure Waveforms
  assert(ObjJson.HasMember("WaveSys")); // Check that WaveSys exists
  const ObjJson &objWave = ObjJson["WaveSys"]; // JSON object of WaveSys
  assert(objWave.IsObject()); // WaveSys is a list of objects, iterate through each

  // Create a Map of WaveSys Classes
  WaveSysMap waveSysMap = WaveFactory::Config(objWave);
  numWave_ = waveSysMap.size(); // Get the number of waves defined

  // Configure Excitations

  // Configure Test Points

}

void MissMgr::Init()
{
  // Initialize timers
  t_s_ = -1;

  // Initialize frame counter
  frame_cnt_ = 0;

  // Initialize the Test index
  iTest_ = 1;

  // Reset the Controllers and Test Points
  Reset();
}

MissMgrOut MissMgr::ModeMgr(const FmuData& FmuDataRef)
{
  // Get the current time
  t_s_ = (float) FmuDataRef.Time_us / 1e6;
  frame_cnt_++;

  // Get the Pilot commands associated with mode control
  uint8_t pilotAuxModeIndx = 0; // AuxInputs associated with Controller Mode Select
  uint8_t pilotAuxTestIndx = 1; // AuxInputs associated with Controller Mode Select
  uint8_t pilotAuxTrigIndx = 2; // AuxInputs associated with Controller Mode Select

  float modeSel = FmuDataRef.SbusRx[0].AuxInputs[pilotAuxModeIndx];
  float testSwitch = FmuDataRef.SbusRx[0].AuxInputs[pilotAuxTestIndx];
  float testTrig = FmuDataRef.SbusRx[0].AuxInputs[pilotAuxTrigIndx];

  // Determine Auto Enable State
  if ((FmuDataRef.SbusRx[0].AutoEnabled == 1) & (FmuDataRef.SbusRx[0].Failsafe == 0)) {
    autoEngage_ = 1;

    // Change Controller modes based on cntrlSel Switch Position
    if (autoEngage_ == 1) {
      if (modeSel <= -0.5){
        cntrlMode_ = kCntrlStandby;
        armTest_ = 0;
        engageTest_ = 0;
      } else if ((modeSel > -0.5) & (modeSel < 0.5)) {
        cntrlMode_ = kCntrlInit;
        armTest_ = 0;
        engageTest_ = 0;
      } else if (modeSel > 0.5) {
        cntrlMode_ = kCntrlEngage;
        armTest_ = 1;
      }
    } // Change Controller


    // Manually Increment and Decrement the Test Point Index
    // The mode must be the baseline controller, with no tests engaged
    // The testSwitch indicates whether to increment or decrement the iTest
    // If the testSwitch is in the center position testTrig engages the testPoint
    // The testTrig signal needs to trigger one engagement for each activation
    // A trigger persistence counter and an enable flag are used to ensure only one activation

    // Control the Arming and Engaging of the trigger command
    if (testTrig > 0.5) {

      indxTrigPersist_++; // Increment the persistence counter
      trigEngage_ = 0;    // Clear the trigger engage

      if ((indxTrigPersist_ >= threshTrigPersist_) & (trigArm_ == 1)) { // Set the trigEngage_
        trigArm_ = 0;
        trigEngage_ = 1;
      }

    } else { // Clear the Trigger Persistence Counter, arm trigger, clear engagement
      indxTrigPersist_ = 0;
      trigArm_ = 1;
      trigEngage_ = 0;
    } // If Trigger Arm and Engage


    // If the Trigger has been engaged
    if (trigEngage_ == 1) {
      if ((armTest_ == 1)  & (testSwitch > -0.5) & (testSwitch < 0.5)) {
        if (engageTest_== 0) {
          engageTest_ = 1;
        } else if (engageTest_ == 1) {
          engageTest_ = 0;
        }
      } else if ((testSwitch <= -0.5) & (engageTest_== 0)) {
        iTest_--;
        if (iTest_ <= 1) iTest_ = 1;

      } else if ((testSwitch >= 0.5) & (engageTest_== 0))  {
        iTest_++;
        if (iTest_ >= numTest_) iTest_ = numTest_;
      }
    } else if ((testSwitch <= -0.5) | (testSwitch >= 0.5)) { // Exit engageTest if the select switch isn't in middle position
      engageTest_ = 0;
    } // If Trigger

  } else {
    Reset(); // Reset the Controllers

  } // Auto Engage

  // Mission Mode Structure
  missMgrData_.t_s = t_s_;   // Mission time
  missMgrData_.frame_cnt = frame_cnt_;   // Mission frame counter
  missMgrData_.autoEngage = autoEngage_;   // Mission autoEngage flag

  missMgrData_.cntrlMode = cntrlMode_;
  missMgrData_.numTest = numTest_; // Number of test points

  missMgrData_.armTest = armTest_;
  missMgrData_.engageTest = engageTest_; // Flag to engage excitation
  missMgrData_.iTest = iTest_; // Index Number of the Test

  // Return
  return missMgrData_;
}

void MissMgr::Reset()
{
  // Reset the Auto Engagement Flag
  autoEngage_ = 0;

  // Controller mode
  cntrlMode_ = kCntrlStandby;

  // Reset the Test Point mode control
  indxTrigPersist_ = 0;
  trigArm_ = 1;
  trigEngage_ = 0;

  armTest_ = 0;
  engageTest_ = 0;
}


MissMgrLog MissMgr::Log(const MissMgrOut& missMgrOut)
{
  MissMgrLog missMgrLog;

  missMgrLog.t_s = missMgrOut.t_s;
  missMgrLog.frame_cnt = missMgrOut.frame_cnt;



    bool autoEngage;   // Mission autoEngage flag, FMU/SOC master switch

    uint8_t iTest; // Index number of the current test
    bool armTest;    // Arm flag for the test system
    bool engageTest; // Engage flag for the test system
    uint8_t numTest; // Number of test points

    uint8_t iCtrl; // Index number of the current controller
    CntrlMode cntrlMode;   // Mission controller mode

    uint8_t iExcite; // Index number of the current excitation
    bool engageExcite; // Engage flag for the test system



  missMgrLog.autoEngage = missMgrOut.autoEngage;

  missMgrLog.iTest = missMgrOut.iTest;
  missMgrLog.armTest = missMgrOut.armTest;
  missMgrLog.engageTest = missMgrOut.engageTest;

  missMgrLog.iCtrl = missMgrOut.iCtrl;
  missMgrLog.cntrlMode = missMgrOut.cntrlMode;

  missMgrLog.iExcite = missMgrOut.iExcite;
  missMgrLog.engageExcite = missMgrOut.engageExcite;

  missMgrLog.tDurSens_ms = missMgrOut.tDurSens_ms;
  missMgrLog.tDurNav_ms = missMgrOut.tDurNav_ms;
  missMgrLog.tDurExcite_ms = missMgrOut.tDurExcite_ms;
  missMgrLog.tDurCntrl_ms = missMgrOut.tDurCntrl_ms;
  missMgrLog.tCmd_ms = missMgrOut.tCmd_ms;
  missMgrLog.tFrame_ms = missMgrOut.tFrame_ms;

  return missMgrLog;
}
