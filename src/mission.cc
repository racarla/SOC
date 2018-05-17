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
  if (Config.HasMember("Engage-Switch")) {
    const rapidjson::Value& EngageSwitch = Config["Engage-Switch"];
    if (EngageSwitch.HasMember("Source")) {
      if (DefinitionTreePtr->GetValuePtr<float*>(EngageSwitch["Source"].GetString())) {
        config_.EngageSwitch.SourcePtr = DefinitionTreePtr->GetValuePtr<float*>(EngageSwitch["Source"].GetString());
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Engage switch source not found in global data."));
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Engage switch configuration does not define a source."));
    }
    if (EngageSwitch.HasMember("Threshold")) {
      config_.EngageSwitch.Threshold = EngageSwitch["Threshold"].GetFloat();
    }
    if (EngageSwitch.HasMember("Gain")) {
      config_.EngageSwitch.Gain = EngageSwitch["Gain"].GetFloat();
    }
    EngageSwitchDefined_ = true;
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
}

/* runs the mission manager */
void MissionManager::Run() {
  // determine if the research control law is engaged
  // check the engage switch state and use a persistence counter to filter
  // out noise
  if ((EngageSwitchDefined_)&&(NumberOfTestPoints_ > 0)) {
    if (*config_.EngageSwitch.SourcePtr*config_.EngageSwitch.Gain > config_.EngageSwitch.Threshold) {
      PersistenceCounter_++;
    } else {
      PersistenceCounter_ = 0;
    }
    // run research control law
    if (PersistenceCounter_ > PersistenceThreshold_) {
      if (!TestPointIndexLatch_) {
        EngagedSensorProcessing_ = TestPoints_[std::to_string(TestPointIndex_)].SensorProcessing;
        EnagagedController_ = TestPoints_[std::to_string(TestPointIndex_)].Control;
        EnagagedExcitation_ = TestPoints_[std::to_string(TestPointIndex_)].Excitation;
        TestPointIndexLatch_ = true;
        TestPointIndex_++;
        if (TestPointIndex_>=NumberOfTestPoints_) {
          TestPointIndex_ = 0;
        }
      }
    // else engage baseline
    } else {
      EngagedSensorProcessing_ = "Baseline";
      EnagagedController_ = "Baseline";
      EnagagedExcitation_ = "None";
      TestPointIndexLatch_ = false;
    }
    // arm the next control law
    ArmedController_ = TestPoints_[std::to_string(TestPointIndex_)].Control;
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
