/*
mission.cc
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2018 Bolder Flight Systems
Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "mission.hxx"

/* configures the mission manager given a JSON value and registers data with global defs */
void MissionManager::Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr) {
  // get the engage switch configuration
  if (Config.HasMember("Fmu-Soc-Switch")) {
    const rapidjson::Value& TempSwitch = Config["Fmu-Soc-Switch"];
    if (TempSwitch.HasMember("Source")) {
      if (DefinitionTreePtr->GetValuePtr<float*>(TempSwitch["Source"].GetString())) {
        config_.SocEngageSwitch.SourcePtr = DefinitionTreePtr->GetValuePtr<float*>(TempSwitch["Source"].GetString());
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Fmu-Soc-Switch source not found in global data."));
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Fmu-Soc-Switch configuration does not define a source."));
    }
    if (TempSwitch.HasMember("Threshold")) {
      config_.SocEngageSwitch.Threshold = TempSwitch["Threshold"].GetFloat();
    }
    if (TempSwitch.HasMember("Gain")) {
      config_.SocEngageSwitch.Gain = TempSwitch["Gain"].GetFloat();
    }
  }

  // get the research switch configuration
  if (Config.HasMember("Control-Select-Switch")) {
    const rapidjson::Value& TempSwitch = Config["Control-Select-Switch"];
    if (TempSwitch.HasMember("Source")) {
      if (DefinitionTreePtr->GetValuePtr<float*>(TempSwitch["Source"].GetString())) {
        config_.CtrlSelectSwitch.SourcePtr = DefinitionTreePtr->GetValuePtr<float*>(TempSwitch["Source"].GetString());
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Control-Select-Switch source not found in global data."));
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Control-Select-Switch configuration does not define a source."));
    }
    if (TempSwitch.HasMember("Threshold")) {
      config_.CtrlSelectSwitch.Threshold = TempSwitch["Threshold"].GetFloat();
    }
    if (TempSwitch.HasMember("Gain")) {
      config_.CtrlSelectSwitch.Gain = TempSwitch["Gain"].GetFloat();
    }
  }

  // get the excitation switch configuration
  if (Config.HasMember("Trigger-Switch")) {
    const rapidjson::Value& TempSwitch = Config["Trigger-Switch"];
    if (TempSwitch.HasMember("Source")) {
      if (DefinitionTreePtr->GetValuePtr<float*>(TempSwitch["Source"].GetString())) {
        config_.TriggerSwitch.SourcePtr = DefinitionTreePtr->GetValuePtr<float*>(TempSwitch["Source"].GetString());
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Trigger-Switch source not found in global data."));
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Trigger-Switch configuration does not define a source."));
    }
    if (TempSwitch.HasMember("Threshold")) {
      config_.TriggerSwitch.Threshold = TempSwitch["Threshold"].GetFloat();
    }
    if (TempSwitch.HasMember("Gain")) {
      config_.TriggerSwitch.Gain = TempSwitch["Gain"].GetFloat();
    }
  }

  // get the test point switch configuration
  if (Config.HasMember("Test-Increment-Switch")) {
    const rapidjson::Value& TempSwitch = Config["Test-Increment-Switch"];
    if (TempSwitch.HasMember("Source")) {
      if (DefinitionTreePtr->GetValuePtr<float*>(TempSwitch["Source"].GetString())) {
        config_.TestSelectIncrementSwitch.SourcePtr = DefinitionTreePtr->GetValuePtr<float*>(TempSwitch["Source"].GetString());
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Test-Increment-Switch source not found in global data."));
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Test-Increment-Switch configuration does not define a source."));
    }
    if (TempSwitch.HasMember("Threshold")) {
      config_.TestSelectIncrementSwitch.Threshold = TempSwitch["Threshold"].GetFloat();
    }
    if (TempSwitch.HasMember("Gain")) {
      config_.TestSelectIncrementSwitch.Gain = TempSwitch["Gain"].GetFloat();
    }
  }

  if (Config.HasMember("Test-Decrement-Switch")) {
    const rapidjson::Value& TempSwitch = Config["Test-Decrement-Switch"];
    if (TempSwitch.HasMember("Source")) {
      if (DefinitionTreePtr->GetValuePtr<float*>(TempSwitch["Source"].GetString())) {
        config_.TestSelectDecrementSwitch.SourcePtr = DefinitionTreePtr->GetValuePtr<float*>(TempSwitch["Source"].GetString());
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Test-Decrement-Switch source not found in global data."));
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Test-Decrement-Switch configuration does not define a source."));
    }
    if (TempSwitch.HasMember("Threshold")) {
      config_.TestSelectDecrementSwitch.Threshold = TempSwitch["Threshold"].GetFloat();
    }
    if (TempSwitch.HasMember("Gain")) {
      config_.TestSelectDecrementSwitch.Gain = TempSwitch["Gain"].GetFloat();
    }
  }

  // build a map of the test point data
  if (Config.HasMember("Test-Points")) {
    const rapidjson::Value& TestPoints = Config["Test-Points"];
    assert(TestPoints.IsArray());
    NumberOfTestPoints_ = TestPoints.Size();
    for (auto &TestPoint : TestPoints.GetArray()) {
      if (TestPoint.HasMember("Test-ID")&&TestPoint.HasMember("Sensor-Processing")&&TestPoint.HasMember("Control")&&TestPoint.HasMember("Excitation")) {
        TestPoints_[TestPoint["Test-ID"].GetString()].ID = TestPoint["Test-ID"].GetString();
        TestPoints_[TestPoint["Test-ID"].GetString()].SensorProcessing = TestPoint["Sensor-Processing"].GetString();
        TestPoints_[TestPoint["Test-ID"].GetString()].Control = TestPoint["Control"].GetString();
        TestPoints_[TestPoint["Test-ID"].GetString()].Excitation = TestPoint["Excitation"].GetString();
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Test-ID, Sensor-Processing, Control, or Excitation not included in test point definition."));
      }
    }

    // initialize the next test point index
    NextTestPointIndex_ = CurrentTestPointIndex_ + 1;

    if (NextTestPointIndex_ >= NumberOfTestPoints_) {
      NextTestPointIndex_ = 0;
    }
  }

  // setting the baseline controller
  if (Config.HasMember("Baseline-Controller")) {
    config_.BaselineController = Config["Baseline-Controller"].GetString();
    std::cout << config_.BaselineController << std::endl;
  }

}

/* runs the mission manager */
void MissionManager::Run() {
  // Switch processing

  // FMU / SOC switch logic
  float SocEngageSwitchVal = (*config_.SocEngageSwitch.SourcePtr) * config_.SocEngageSwitch.Gain;
  bool SocEngageCheck = (SocEngageSwitchVal > config_.SocEngageSwitch.Threshold);

  if ((SocEngage_ != true) && (SocEngageCheck == true)) {
    SocEngagePersistenceCounter_++;
    if (SocEngagePersistenceCounter_ > PersistenceThreshold_) {
      SocEngage_ = true;
      SocEngagePersistenceCounter_ = 0;
    }
  } else if ((SocEngage_ != false) && (SocEngageCheck == false)) {
    SocEngagePersistenceCounter_++;
    if (SocEngagePersistenceCounter_ > PersistenceThreshold_) {
      SocEngage_ = false;
      SocEngagePersistenceCounter_ = 0;
    }
  } else {
    SocEngagePersistenceCounter_ = 0;
  }
// std::cout << SocEngageSwitchVal << "\t" << SocEngageCheck << "\t" << SocEngagePersistenceCounter_ << "\t" << SocEngage_ << "\t" << std::flush;

  // Control law select switch logic
  float CtrlSelectSwitchVal = (*config_.CtrlSelectSwitch.SourcePtr) * config_.CtrlSelectSwitch.Gain;
  bool CtrlSelectCheck = CtrlSelectSwitchVal > config_.CtrlSelectSwitch.Threshold;

  if (SocEngage_ == true) {
    if ((CtrlSelect_ != true) && (CtrlSelectCheck == true)) {
      CtrlSelectPersistenceCounter_++;
      if (CtrlSelectPersistenceCounter_ > PersistenceThreshold_) {
        CtrlSelect_ = true;
        CtrlSelectPersistenceCounter_ = 0;
      }
    } else if ((CtrlSelect_ != false) && (CtrlSelectCheck == false)) {
      CtrlSelectPersistenceCounter_++;
      if (CtrlSelectPersistenceCounter_ > PersistenceThreshold_) {
        CtrlSelect_ = false;
        CtrlSelectPersistenceCounter_ = 0;
      }
    } else {
      CtrlSelectPersistenceCounter_ = 0;
    }
  } else {
    CtrlSelect_ = false;
    CtrlSelectPersistenceCounter_ = 0;
  }
// std::cout << CtrlSelectSwitchVal << "\t" << CtrlSelectCheck << "\t" << CtrlSelectPersistenceCounter_ << "\t" << CtrlSelect_ << "\t" << std::flush;

  // Test point select logic
  float TestSelectDecrementSwitchVal = (*config_.TestSelectDecrementSwitch.SourcePtr) * config_.TestSelectDecrementSwitch.Gain;
  bool TestSelectDecrementCheck = TestSelectDecrementSwitchVal > config_.TestSelectDecrementSwitch.Threshold;
  float TestSelectIncrementSwitchVal = (*config_.TestSelectIncrementSwitch.SourcePtr) * config_.TestSelectIncrementSwitch.Gain;
  bool TestSelectIncrementCheck = TestSelectIncrementSwitchVal > config_.TestSelectIncrementSwitch.Threshold;

  bool TestSelectExciteCheck;

  // If the switch is not in Increment or Decrement, then set to Excite
  if ((TestSelectIncrementCheck == false) && (TestSelectDecrementCheck == false)) {
    TestSelectExciteCheck = true;
  } else {
    TestSelectExciteCheck = 0;
  }

  if ((TestSelect_ != 1) && (TestSelectIncrementCheck == true)) {
    TestSelectIncrementPersistenceCounter_++;
    if (TestSelectIncrementPersistenceCounter_ > PersistenceThreshold_) {
      TestSelect_ = 1;
      TestSelectIncrementPersistenceCounter_ = 0;
    }
  } else if ((TestSelect_ != -1) && (TestSelectDecrementCheck == true)) {
    TestSelectDecrementPersistenceCounter_++;
    if (TestSelectDecrementPersistenceCounter_ > PersistenceThreshold_) {
      TestSelect_ = -1;
      TestSelectDecrementPersistenceCounter_ = 0;
    }
  } else if ((TestSelect_ != 0) && (TestSelectExciteCheck == true)) {
    TestSelectExcitePersistenceCounter_++;
    if (TestSelectExcitePersistenceCounter_ > PersistenceThreshold_) {
      TestSelect_ = 0;
      TestSelectExcitePersistenceCounter_ = 0;
    }
  } else {
    TestSelectIncrementPersistenceCounter_ = 0;
    TestSelectDecrementPersistenceCounter_ = 0;
    TestSelectExcitePersistenceCounter_ = 0;
  }

  // Triggger switch logic
  float TriggerValue = (*config_.TriggerSwitch.SourcePtr) * config_.TriggerSwitch.Gain;
  bool TriggerCheck = TriggerValue > config_.TriggerSwitch.Threshold;
  // int TriggerPersistenceCounter_;
  // bool Trigger_;

  if ((Trigger_ != true) && (TriggerCheck == true)) {
    if (TriggerLatch_ == false) {
      TriggerPersistenceCounter_++;
      if (TriggerPersistenceCounter_ > PersistenceThreshold_) {
        Trigger_ = true;
        TriggerPersistenceCounter_ = 0;
        TriggerLatch_ = true;
      }
    }
  } else { // Clear the counter, clear the latch, let the mode controller "clear" the trigger event
    TriggerPersistenceCounter_ = 0;
    TriggerLatch_ = false;
    // Trigger_ = false;
  }
// std::cout << TriggerValue << "\t" << TriggerCheck << "\t" << (int) TriggerPersistenceCounter_ << "\t" << (int) PersistenceThreshold_ << "\t" << (bool) (TriggerPersistenceCounter_ > PersistenceThreshold_)  << "\t" << Trigger_ << "\t\t" << std::flush;


std::cout << SocEngage_ << "\t" << CtrlSelect_ << "\t" << TestSelect_ << "\t" << Trigger_ << "\t" << std::flush;
  // Mode Control Logic

  // Test Selection
  if (TestSelect_ == 0) { // Excitation selected
    if (Trigger_ == true) {
      if (EnagagedExcitation_ == "None") { // Engage the Excitation
        EnagagedExcitation_ = TestPoints_[std::to_string(CurrentTestPointIndex_)].Excitation;
      } else { // Dis-Engage the Excitation
        EnagagedExcitation_ = "None";
      }
      Trigger_ = false;
    }
  } else if (TestSelect_ == 1) { // Increment selected
    EnagagedExcitation_ = "None";

    if (Trigger_ == true) { // Increment the Test Point, switches engaged controller
      CurrentTestPointIndex_ = NextTestPointIndex_;
      NextTestPointIndex_ = CurrentTestPointIndex_ + 1;

      if (NextTestPointIndex_ >= NumberOfTestPoints_) {
        NextTestPointIndex_ = 0;
      }
      Trigger_ = false;
    }
  } else if (TestSelect_ == -1) { // Decrement selected
    EnagagedExcitation_ = "None";

    if (Trigger_ == true) { // Decrement the Test Point to 0, switches engaged controller
      CurrentTestPointIndex_ = 0;
      NextTestPointIndex_ = CurrentTestPointIndex_ + 1;
      Trigger_ = false;
    }
  }

  // SOC Controller and SensorProcessing Mode Switching
  if (SocEngage_ == true) {
    if (CtrlSelect_ == true) { // SOC Research
      EngagedSensorProcessing_ = TestPoints_[std::to_string(CurrentTestPointIndex_)].SensorProcessing;
      EnagagedController_ = TestPoints_[std::to_string(CurrentTestPointIndex_)].Control;
      ArmedController_ = TestPoints_[std::to_string(NextTestPointIndex_)].Control;
      // EnagagedExcitation_ = "None";

    } else { // In SOC Baseline, arm the next controller, no excitation
      EngagedSensorProcessing_ = "Baseline";
      EnagagedController_ = config_.BaselineController;
      ArmedController_ = TestPoints_[std::to_string(CurrentTestPointIndex_)].Control;
      EnagagedExcitation_ = "None";
    }

  } else { // FMU Mode
    EngagedSensorProcessing_ = "Baseline";
    EnagagedController_ = "Fmu";
    ArmedController_ = config_.BaselineController;
    EnagagedExcitation_ = "None";
  }



std::cout << CurrentTestPointIndex_ << "\t" << NextTestPointIndex_ << "\t" << EnagagedController_ << "\t" << ArmedController_ << "\t" << EnagagedExcitation_ << std::endl;
}

/* returns the string of the sensor processing group that is engaged */
std::string MissionManager::GetEnagagedSensorProcessing() {
  return EngagedSensorProcessing_;
}

/* returns the string of the control group that is engaged */
std::string MissionManager::GetEnagagedController() {
  return EnagagedController_;
}

/* returns the string of the control group that is armed */
std::string MissionManager::GetArmedController() {
  return ArmedController_;
}

/* returns the string of the excitation group that is engaged */
std::string MissionManager::GetEnagagedExcitation() {
  return EnagagedExcitation_;
}
