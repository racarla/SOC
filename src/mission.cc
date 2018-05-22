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
    const rapidjson::Value& EngageSwitch = Config["Fmu-Soc-Switch"];
    if (EngageSwitch.HasMember("Source")) {
      if (DefinitionTreePtr->GetValuePtr<float*>(EngageSwitch["Source"].GetString())) {
        config_.SocEngageSwitch.SourcePtr = DefinitionTreePtr->GetValuePtr<float*>(EngageSwitch["Source"].GetString());
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Engage switch source not found in global data."));
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Engage switch configuration does not define a source."));
    }
    if (EngageSwitch.HasMember("Threshold")) {
      config_.SocEngageSwitch.Threshold = EngageSwitch["Threshold"].GetFloat();
    }
    if (EngageSwitch.HasMember("Gain")) {
      config_.SocEngageSwitch.Gain = EngageSwitch["Gain"].GetFloat();
    }
  } 
  // get the research switch configuration
  if (Config.HasMember("Baseline-Research-Switch")) {
    const rapidjson::Value& EngageSwitch = Config["Baseline-Research-Switch"];
    if (EngageSwitch.HasMember("Source")) {
      if (DefinitionTreePtr->GetValuePtr<float*>(EngageSwitch["Source"].GetString())) {
        config_.ResearchEngageSwitch.SourcePtr = DefinitionTreePtr->GetValuePtr<float*>(EngageSwitch["Source"].GetString());
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Engage switch source not found in global data."));
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Engage switch configuration does not define a source."));
    }
    if (EngageSwitch.HasMember("Threshold")) {
      config_.ResearchEngageSwitch.Threshold = EngageSwitch["Threshold"].GetFloat();
    }
    if (EngageSwitch.HasMember("Gain")) {
      config_.ResearchEngageSwitch.Gain = EngageSwitch["Gain"].GetFloat();
    }
  }   
  // get the excitation switch configuration
  if (Config.HasMember("Excitation-Switch")) {
    const rapidjson::Value& EngageSwitch = Config["Excitation-Switch"];
    if (EngageSwitch.HasMember("Source")) {
      if (DefinitionTreePtr->GetValuePtr<float*>(EngageSwitch["Source"].GetString())) {
        config_.ExcitationEngageSwitch.SourcePtr = DefinitionTreePtr->GetValuePtr<float*>(EngageSwitch["Source"].GetString());
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Engage switch source not found in global data."));
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Engage switch configuration does not define a source."));
    }
    if (EngageSwitch.HasMember("Threshold")) {
      config_.ExcitationEngageSwitch.Threshold = EngageSwitch["Threshold"].GetFloat();
    }
    if (EngageSwitch.HasMember("Gain")) {
      config_.ExcitationEngageSwitch.Gain = EngageSwitch["Gain"].GetFloat();
    }
  }  
  // get the test point switch configuration
  if (Config.HasMember("Test-Increment-Switch")) {
    const rapidjson::Value& EngageSwitch = Config["Test-Increment-Switch"];
    if (EngageSwitch.HasMember("Source")) {
      if (DefinitionTreePtr->GetValuePtr<float*>(EngageSwitch["Source"].GetString())) {
        config_.TestPointIncrementSwitch.SourcePtr = DefinitionTreePtr->GetValuePtr<float*>(EngageSwitch["Source"].GetString());
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Engage switch source not found in global data."));
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Engage switch configuration does not define a source."));
    }
    if (EngageSwitch.HasMember("Threshold")) {
      config_.TestPointIncrementSwitch.Threshold = EngageSwitch["Threshold"].GetFloat();
    }
    if (EngageSwitch.HasMember("Gain")) {
      config_.TestPointIncrementSwitch.Gain = EngageSwitch["Gain"].GetFloat();
    }
  }  
  if (Config.HasMember("Test-Decrement-Switch")) {
    const rapidjson::Value& EngageSwitch = Config["Test-Decrement-Switch"];
    if (EngageSwitch.HasMember("Source")) {
      if (DefinitionTreePtr->GetValuePtr<float*>(EngageSwitch["Source"].GetString())) {
        config_.TestPointDecrementSwitch.SourcePtr = DefinitionTreePtr->GetValuePtr<float*>(EngageSwitch["Source"].GetString());
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Engage switch source not found in global data."));
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Engage switch configuration does not define a source."));
    }
    if (EngageSwitch.HasMember("Threshold")) {
      config_.TestPointDecrementSwitch.Threshold = EngageSwitch["Threshold"].GetFloat();
    }
    if (EngageSwitch.HasMember("Gain")) {
      config_.TestPointDecrementSwitch.Gain = EngageSwitch["Gain"].GetFloat();
    }
  } 
  // build a map of the test point data
  if (Config.HasMember("Test-Points")) {
    const rapidjson::Value& TestPoints = Config["Test-Points"];
    NumberOfTestPoints_ = TestPoints.Size();
    for (auto &TestPoint : TestPoints.GetArray()) {
      if (TestPoint.HasMember("Test-ID")&&TestPoint.HasMember("Sensor-Processing")&&TestPoint.HasMember("Control")&&TestPoint.HasMember("Excitation")) {
        TestPoints_[TestPoint["Test-ID"].GetString()].SensorProcessing = TestPoint["Sensor-Processing"].GetString();
        TestPoints_[TestPoint["Test-ID"].GetString()].Control = TestPoint["Control"].GetString();
        TestPoints_[TestPoint["Test-ID"].GetString()].Excitation = TestPoint["Excitation"].GetString();
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Test-ID, Sensor-Processing, Control, or Excitation not included in test point definition."));
      }
    }
  } 
  // setting the baseline controller
  if (Config.HasMember("Baseline-Controller")) {
    config_.BaselineController = Config["Baseline-Controller"].GetString();
  }
}

/* runs the mission manager */
void MissionManager::Run() {
  // FMU / SOC switch logic
  if (*config_.SocEngageSwitch.SourcePtr*config_.SocEngageSwitch.Gain > config_.SocEngageSwitch.Threshold) {
    SocEngageSwitchPersistenceCounter_++;
  } else {
    SocEngageSwitchPersistenceCounter_ = 0;
  }
  // In SOC baseline, no excitation, arm the next controller
  if (SocEngageSwitchPersistenceCounter_ > PersistenceThreshold_) {
    EngagedSensorProcessing_ = "Baseline";
    EnagagedController_ = config_.BaselineController;
    ArmedController_ = ArmedController_ = TestPoints_[std::to_string(NextTestPointIndex_)].Control;
    EnagagedExcitation_ = "None";
  } else {
    // In FMU, no excitation, arm the baseline controller
    EngagedSensorProcessing_ = "Baseline";
    EnagagedController_ = "Fmu";
    ArmedController_ = config_.BaselineController;
    EnagagedExcitation_ = "None";
  }
  // Research control law logic
  if (*config_.ResearchEngageSwitch.SourcePtr*config_.ResearchEngageSwitch.Gain > config_.ResearchEngageSwitch.Threshold) {
    ResearchEngageSwitchPersistenceCounter_++;
  } else {
    ResearchEngageSwitchPersistenceCounter_ = 0;
  }
  // SOC Research sensor processing and control, no excitation, arm the next controller
  if (ResearchEngageSwitchPersistenceCounter_ > PersistenceThreshold_) {
    if (!ResearchEngageLatch_) {
      ResearchEngageLatch_ = true;
      CurrentTestPointIndex_ = NextTestPointIndex_;
    }
    EngagedSensorProcessing_ = TestPoints_[std::to_string(CurrentTestPointIndex_)].SensorProcessing;
    EnagagedController_ = TestPoints_[std::to_string(CurrentTestPointIndex_)].Control;
    ArmedController_ = config_.BaselineController;
    EnagagedExcitation_ = "None";
    // In SOC baseline, no excitation, arm the next controller
  } else {
    ResearchEngageLatch_ = false;
  }
  // Excitation engage logic
  if (*config_.ExcitationEngageSwitch.SourcePtr*config_.ExcitationEngageSwitch.Gain > config_.ExcitationEngageSwitch.Threshold) {
    ExcitationEngageSwitchPersistenceCounter_++;
  } else {
    ExcitationEngageSwitchPersistenceCounter_ = 0;
  }
  // Excitation engaged
  if (ExcitationEngageSwitchPersistenceCounter_ > PersistenceThreshold_) {
    EnagagedExcitation_ = TestPoints_[std::to_string(CurrentTestPointIndex_)].Excitation;
    // Excitation not engaged
  } else {
    EnagagedExcitation_ = "None";
  }
  // Test point select logic
  if (*config_.TestPointIncrementSwitch.SourcePtr*config_.TestPointIncrementSwitch.Gain > config_.TestPointIncrementSwitch.Threshold) {
    if (!TestPointIndexLatch_) {
      TestPointIndexLatch_ = true;
      NextTestPointIndex_++;
      if (NextTestPointIndex_>=NumberOfTestPoints_) {
        NextTestPointIndex_ = 0;
      }
    }
  }
  else if (*config_.TestPointDecrementSwitch.SourcePtr*config_.TestPointDecrementSwitch.Gain < config_.TestPointDecrementSwitch.Threshold) {
    if (!TestPointIndexLatch_) {
      TestPointIndexLatch_ = true;
      if (NextTestPointIndex_==0) {
        NextTestPointIndex_ = NumberOfTestPoints_-1;
      } else {
        NextTestPointIndex_--;
      }
    }
  } else {
    TestPointIndexLatch_ = false;
  }
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
