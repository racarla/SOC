/*
Mission Manager - Define the mission parameters, control system modes

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-15 - Chris Regan - Created
*/

#include "missionMgr.hxx"

void MissionMgr::Init()
{
  numTest_ = 19; // FIXIT - Hardcoded

  // Reset the Controllers and Test Points
  Reset();
}

MissionMode MissionMgr::ModeMgr(const FmuData FmuDataRef)
{
  // Get the current time
  time_s_ = (float) FmuDataRef.Time_us / 1e6;
  frame_cnt_++;

  // Get the Pilot commands associated with mode control
  uint8_t pilotAuxModeIndx = 0; // AuxInputs associated with Controller Mode Select
  uint8_t pilotAuxTestIndx = 1; // AuxInputs associated with Controller Mode Select
  uint8_t pilotAuxTrigIndx = 2; // AuxInputs associated with Controller Mode Select

  float modeSel = FmuDataRef.SbusRx[0].AuxInputs[pilotAuxModeIndx];
  float testSwitch = FmuDataRef.SbusRx[0].AuxInputs[pilotAuxTestIndx];
  float testTrig = FmuDataRef.SbusRx[0].AuxInputs[pilotAuxTrigIndx];

  // Determine Auto Enable State
  if (FmuDataRef.SbusRx[0].AutoEnabled == 1) {
    autoEngage_ = 1;

    // Change Controller modes based on cntrlSel Switch Position
    if (autoEngage_ == 1) {
      if (modeSel <= -0.5){
        cntrlMode_ = kCntrlStandby;
        testArm_ = 0;
        testEngage_ = 0;
      } else if ((modeSel > -0.5) & (modeSel < 0.5)) {
        cntrlMode_ = kCntrlInit;
        testArm_ = 0;
        testEngage_ = 0;
      } else if (modeSel > 0.5) {
        cntrlMode_ = kCntrlEngage;
        testArm_ = 1;
      }
    } // Change Controller


    // Manually Increment and Decrement the Test Point Index
    // The mode must be the baseline controller, with no tests engaged
    // The testSwitch indicates whether to increment or decrement the indxTest
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
      if ((testArm_ == 1)  & (testSwitch > -0.5) & (testSwitch < 0.5)) {
        testEngage_ = 1;

      } else if ((testArm_ == 0) & (testSwitch < -0.5)) {
        indxTest_--;
//        if (indxTest_ <= 1) indxTest_ = 1;

      } else if ((testArm_ == 0) & (testSwitch > 0.5))  {
        indxTest_++;
//        if (indxTest_ >= numTest_) indxTest_ = numTest_;
      }
    } // If Trigger

  } else {
    Reset(); // Reset the Controllers

  } // Auto Engage

  // Mission Mode Structure
  missionMode_.time_s = time_s_;   // Mission time
  missionMode_.frame_cnt = frame_cnt_;   // Mission frame counter
  missionMode_.autoEngage = autoEngage_;   // Mission autoEngage flag

  missionMode_.cntrlMode = cntrlMode_;
  missionMode_.numTest = numTest_; // Number of test points

  missionMode_.trigArm = trigArm_;
  missionMode_.trigEngage = trigEngage_;

  missionMode_.testArm = testArm_;
  missionMode_.testEngage = testEngage_; // Flag to engage excitation
  missionMode_.indxTest = indxTest_; // Index Number of the Test

  // Return 
  return missionMode_;
}

void MissionMgr::Reset()
{
  // Initialize timers 
  time_s_ = -1;

  // Initialize frame counter
  frame_cnt_ = 0;

  // Reset the Auto Engagement Flag 
  autoEngage_ = 0;

  // Controller mode
  cntrlMode_ = kCntrlStandby;

  // Reset the Test Point mode control
  indxTrigPersist_ = 0;
  trigArm_ = 1;
  trigEngage_ = 0;

  testArm_ = 0;
  testEngage_ = 0;
  indxTest_ = 1;
}
